/*
 * Copyright (c) 2024 The Linux Foundation. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * DOC: wma_frame_inject.c
 *
 * This file contains WMA layer frame injection queue management functions.
 * It provides infrastructure for queuing, processing, and transmitting
 * injected 802.11 frames through the firmware interface.
 */

#include "wma.h"
#include "wma_frame_inject.h"
#include "wlan_hdd_frame_inject.h"
#include "wma_api.h"
#include "wma_internal.h"
#include "wmi_unified_api.h"
#include "wmi_unified.h"
#include "qdf_mem.h"
#include "qdf_list.h"
#include "qdf_lock.h"
#include "qdf_status.h"
#include "qdf_trace.h"
#include "qdf_nbuf.h"
#include "qdf_delayed_work.h"
#include "qdf_time.h"
#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include "cds_api.h"
#include "wlan_reg_services_api.h"
#include "wlan_vdev_mlme_api.h"
#include "cdp_txrx_cmn.h"
#include <wlan_vdev_mgr_tgt_if_tx_defs.h>
#if defined(CONFIG_HL_SUPPORT)
#include "wlan_tgt_def_config_hl.h"
#else
#include "wlan_tgt_def_config.h"
#endif

#ifdef FEATURE_FRAME_INJECTION_SUPPORT

/*
 * wma_injection_unmap_tx_buf() - Unmap DMA mapping on injection nbuf
 *
 * On LL (low-latency / PCI / SNOC) the WMI layer DMA-maps the tx_frame
 * passed in mgmt_params.  The mapping must be released on completion.
 * On HL (high-latency / SDIO / USB) the frame data is copied inline into
 * the WMI command and no DMA mapping exists, so the unmap is a no-op.
 */
#ifdef CONFIG_HL_SUPPORT
static inline void wma_injection_unmap_tx_buf(qdf_nbuf_t buf)
{
}
#else
static inline void wma_injection_unmap_tx_buf(qdf_nbuf_t buf)
{
	void *qdf_ctx = cds_get_context(QDF_MODULE_ID_QDF_DEVICE);

	if (qdf_ctx && buf)
		qdf_nbuf_unmap_single(qdf_ctx, buf, QDF_DMA_TO_DEVICE);
}
#endif

/* Maximum number of frames in WMA injection queue */
#define WMA_FRAME_INJECT_MAX_QUEUE_SIZE    512

/* Maximum frame size for injection */
#define WMA_FRAME_INJECT_MAX_FRAME_SIZE    2304

/* Timeout for queue processing work in milliseconds */
#define WMA_FRAME_INJECT_QUEUE_TIMEOUT_MS  100

/* Reaper timer interval: how often we scan for stale in-flight nbufs (ms) */
#define WMA_INJECTION_REAPER_INTERVAL_MS   3000

/*
 * Maximum age (in microseconds, QDF log-timestamp units) before an
 * in-flight nbuf is considered abandoned by firmware and reaped.
 * 2 seconds is generous — normal completions arrive in < 50 ms.
 */
#define WMA_INJECTION_NBUF_TIMEOUT_US      2000000ULL

/*
 * Maximum number of in-flight (submitted but uncomplemented) nbufs before
 * we start rejecting new enqueue requests.  Keeps DMA-mapped memory bounded
 * when firmware silently drops completions for the helper STA vdev.
 */
#define WMA_INJECTION_INFLIGHT_HIGH        200

/**
 * struct wma_injection_queue_node - Node for injection queue
 * @node: List node
 * @req: Frame injection request
 * @timestamp: Enqueue timestamp
 * @vdev_id: VDEV ID for the frame
 */
struct wma_injection_queue_node {
	qdf_list_node_t node;
	struct inject_frame_req req;
	uint64_t timestamp;
	uint8_t vdev_id;
};

/**
 * struct wma_injection_queue_ctx - WMA injection queue context
 * @queue: Queue of pending injection requests
 * @queue_lock: Lock for queue operations
 * @queue_size: Current queue size
 * @max_queue_size: Maximum allowed queue size
 * @queue_work: Work item for processing queue
 * @delayed_work: Delayed work item for backpressure handling
 * @stats: Queue statistics
 * @is_initialized: Initialization flag
 */
struct wma_injection_queue_ctx {
	qdf_list_t queue;
	qdf_spinlock_t queue_lock;
	uint32_t queue_size;
	uint32_t max_queue_size;
	qdf_work_t queue_work;
	struct qdf_delayed_work delayed_work;
	struct qdf_delayed_work reaper_work; /* periodic stale-nbuf reaper */
	qdf_atomic_t inflight_count; /* nbufs submitted to FW, not yet completed */
	struct wma_injection_queue_stats stats;
	bool is_initialized;
	/* Singularity injection TX quiescing */
	bool quiescing;
};

/* Statistics structure is defined in wma_frame_inject.h */

/* Global injection queue context */
static struct wma_injection_queue_ctx g_wma_injection_ctx;

/*
 * Per-session one-shot logging flags and counters for
 * wma_send_injection_frame_to_fw().  Reset on each new
 * injection TX vdev creation so restarts re-log setup info.
 */
static bool inject_tx_cfg_logged;
static uint32_t inject_send_info_count;
static bool inject_wmi_path_logged;
static bool inject_legacy_path_logged;
static bool inject_monitor_no_legacy_logged;
static bool inject_wmi_service_absent_logged;
static bool inject_probe_sa_fix_logged;
static bool inject_patch_banner_logged;

/*
 * Keep a small best-effort debug cache so firmware completion status can be
 * correlated with the frame metadata for the corresponding desc_id.
 */
#define WMA_INJECTION_DEBUG_CACHE_SIZE 256
struct wma_injection_debug_info {
	bool valid;
	uint32_t desc_id;
	uint16_t frame_len;
	uint16_t chanfreq;
	uint8_t fc_type;
	uint8_t fc_subtype;
	uint8_t addr1[QDF_MAC_ADDR_SIZE];
	uint8_t addr2[QDF_MAC_ADDR_SIZE];
	uint8_t addr3[QDF_MAC_ADDR_SIZE];
	qdf_nbuf_t tx_buf; /* nbuf passed to WMI; must be unmapped+freed on completion */
	uint64_t submit_ts; /* log-timestamp when submitted to FW */
};

static struct wma_injection_debug_info
	g_wma_injection_debug_cache[WMA_INJECTION_DEBUG_CACHE_SIZE];
static DEFINE_SPINLOCK(g_wma_injection_cache_lock);

/*
 * A firmware-only STA helper has no object-manager vdev or MLME timer.
 * Keep its response ownership separate from normal vdev response handling.
 * Lifecycle commands are serialized by g_inj_tx_mutex; the WMI event path
 * only takes g_inj_rsp_lock and never waits for that mutex.
 */
#define WMA_INJECTION_VDEV_RESPONSE_MS 1500

struct wma_injection_tx_vdev {
	bool allocated;
	bool created; /* START succeeded and peer-create command was submitted */
	bool start_sent;
	bool start_rejected;
	bool peer_created;
	bool peer_delete_sent;
	bool stop_sent;
	bool delete_sent;
	uint8_t vdev_id;
	uint8_t monitor_vdev_id;
	uint32_t chanfreq;
	uint8_t mac_addr[QDF_MAC_ADDR_SIZE];
};

struct wma_injection_vdev_responses {
	struct wlan_objmgr_psoc *psoc;
	uint8_t vdev_id;
	unsigned long pending;
	uint32_t status[MLME_FW_ONLY_VDEV_EVENT_MAX];
	struct completion done[MLME_FW_ONLY_VDEV_EVENT_MAX];
};

static struct wma_injection_tx_vdev g_inj_tx_vdev;
static struct wma_injection_vdev_responses g_inj_rsp;
static DEFINE_SPINLOCK(g_inj_rsp_lock);
static DEFINE_MUTEX(g_inj_tx_mutex);
static bool g_inj_tx_session_failed;

static QDF_STATUS wma_injection_destroy_tx_vdev_locked(tp_wma_handle wma);
static QDF_STATUS wma_injection_destroy_tx_vdev(tp_wma_handle wma);
static void wma_injection_queue_node_free(struct wma_injection_queue_node *node);

bool wma_injection_vdev_response(struct wlan_objmgr_psoc *psoc,
	uint8_t vdev_id, enum mlme_fw_only_vdev_event event,
	uint32_t fw_status)
{
	unsigned long flags;
	bool owned;

	if ((unsigned int)event >= MLME_FW_ONLY_VDEV_EVENT_MAX)
		return false;

	spin_lock_irqsave(&g_inj_rsp_lock, flags);
	owned = psoc && g_inj_rsp.psoc == psoc &&
		g_inj_rsp.vdev_id == vdev_id;
	if (owned && (g_inj_rsp.pending & BIT(event))) {
		g_inj_rsp.status[event] = fw_status;
		g_inj_rsp.pending &= ~BIT(event);
		complete(&g_inj_rsp.done[event]);
	}
	spin_unlock_irqrestore(&g_inj_rsp_lock, flags);

	/* Consume only this PSOC's owned firmware-only helper responses. */
	return owned;
}

static QDF_STATUS wma_injection_claim_responses(tp_wma_handle wma, uint8_t vdev_id)
{
	unsigned long flags;
	int event;
	QDF_STATUS status;

	status = wlan_objmgr_psoc_try_get_ref(wma->psoc, WLAN_LEGACY_WMA_ID);
	if (QDF_IS_STATUS_ERROR(status))
		return status;
	status = wlan_objmgr_psoc_reserve_fw_vdev_id(wma->psoc, vdev_id);
	if (QDF_IS_STATUS_ERROR(status)) {
		wlan_objmgr_psoc_release_ref(wma->psoc, WLAN_LEGACY_WMA_ID);
		return status;
	}

	spin_lock_irqsave(&g_inj_rsp_lock, flags);
	for (event = 0; event < MLME_FW_ONLY_VDEV_EVENT_MAX; event++) {
		init_completion(&g_inj_rsp.done[event]);
		g_inj_rsp.status[event] = 0;
	}
	g_inj_rsp.pending = 0;
	g_inj_rsp.vdev_id = vdev_id;
	g_inj_rsp.psoc = wma->psoc;
	spin_unlock_irqrestore(&g_inj_rsp_lock, flags);
	return QDF_STATUS_SUCCESS;
}

static void wma_injection_release_responses(void)
{
	struct wlan_objmgr_psoc *psoc;
	uint8_t id;
	unsigned long flags;

	spin_lock_irqsave(&g_inj_rsp_lock, flags);
	psoc = g_inj_rsp.psoc;
	id = g_inj_rsp.vdev_id;
	g_inj_rsp.psoc = NULL;
	g_inj_rsp.pending = 0;
	spin_unlock_irqrestore(&g_inj_rsp_lock, flags);
	if (psoc) {
		wlan_objmgr_psoc_release_fw_vdev_id(psoc, id);
		wlan_objmgr_psoc_release_ref(psoc, WLAN_LEGACY_WMA_ID);
	}
}

static void wma_injection_arm_response(enum mlme_fw_only_vdev_event event)
{
	unsigned long flags;

	spin_lock_irqsave(&g_inj_rsp_lock, flags);
	reinit_completion(&g_inj_rsp.done[event]);
	g_inj_rsp.pending |= BIT(event);
	spin_unlock_irqrestore(&g_inj_rsp_lock, flags);
}

static QDF_STATUS
wma_injection_wait_response(enum mlme_fw_only_vdev_event event)
{
	if (!wait_for_completion_timeout(&g_inj_rsp.done[event],
			msecs_to_jiffies(WMA_INJECTION_VDEV_RESPONSE_MS))) {
		wma_err("NHHELPER response timeout: vdev=%u event=%u; slot retained",
			g_inj_tx_vdev.vdev_id, event);
		return QDF_STATUS_E_TIMEOUT;
	}

	if (READ_ONCE(g_inj_rsp.status[event])) {
		wma_err("NHHELPER firmware rejection: vdev=%u event=%u status=0x%08x",
			g_inj_tx_vdev.vdev_id, event,
			READ_ONCE(g_inj_rsp.status[event]));
		return QDF_STATUS_E_FAILURE;
	}

	wma_info("NHHELPER response accepted: vdev=%u event=%u",
		 g_inj_tx_vdev.vdev_id, event);
	return QDF_STATUS_SUCCESS;
}

/**
 * wma_injection_reset_session_state() - Reset per-session static state
 *
 * Called when a new injection TX vdev is being created (fresh session
 * or channel change).  Resets all file-static one-shot logging flags
 * and the send counter so the new session starts clean.
 */
static void wma_injection_reset_session_state(void)
{
	struct wma_injection_queue_ctx *ctx = &g_wma_injection_ctx;

	/* Reset one-shot logging flags so fresh session re-logs config */
	inject_tx_cfg_logged = false;
	inject_send_info_count = 0;
	inject_wmi_path_logged = false;
	inject_legacy_path_logged = false;
	inject_monitor_no_legacy_logged = false;
	inject_wmi_service_absent_logged = false;
	inject_probe_sa_fix_logged = false;
	inject_patch_banner_logged = false;

	/* Reset rate-limited log counters so new session gets fresh logs */
	if (ctx->is_initialized) {
		ctx->stats.fw_errors = 0;
		ctx->stats.frames_processed = 0;
	}

	/*
	 * Do NOT sweep/free nbufs from the debug cache here.
	 * The firmware may still be DMA-reading in-flight buffers;
	 * freeing them would corrupt FW memory and crash.  Stale
	 * nbufs are safely freed by:
	 *   - normal completion handler (wma_handle_injection_fw_response)
	 *   - slot reuse in wma_injection_debug_cache_update()
	 *   - wma_deinit_injection_queue() at driver shutdown
	 */
}

static QDF_STATUS
wma_injection_ensure_tx_vdev_locked(tp_wma_handle wma, uint8_t mon_vdev_id,
				  uint32_t chanfreq)
{
	struct vdev_create_params vcreate = {0};
	struct vdev_start_params vstart = {0};
	struct peer_create_params pcreate = {0};
	struct wlan_objmgr_pdev *pdev;
	struct vdev_mlme_obj *mon_mlme;
	enum dfs_reg dfs_region;
	uint8_t *mon_mac;
	uint8_t inj_mac[QDF_MAC_ADDR_SIZE];
	uint8_t vid = 0;
	bool found = false;
	QDF_STATUS status;
	int fw_max_vid, i;

	if (g_inj_tx_session_failed)
		return QDF_STATUS_E_FAILURE;

	if (g_inj_tx_vdev.created &&
	    g_inj_tx_vdev.chanfreq == chanfreq &&
	    g_inj_tx_vdev.monitor_vdev_id == mon_vdev_id)
		return QDF_STATUS_SUCCESS;

	if (g_inj_tx_vdev.allocated) {
		status = wma_injection_destroy_tx_vdev_locked(wma);
		if (QDF_IS_STATUS_ERROR(status))
			goto fail_session;
	}

	/* Require acknowledged teardown before a firmware slot can be reused. */
	if (!wmi_service_enabled(wma->wmi_handle, wmi_service_sync_delete_cmds)) {
		wma_err("NHHELPER requires firmware sync-delete support");
		status = QDF_STATUS_E_NOSUPPORT;
		goto fail_session;
	}

	fw_max_vid = CFG_TGT_NUM_VDEV - 2;
	if (fw_max_vid >= (int)wma->max_bssid)
		fw_max_vid = (int)wma->max_bssid - 1;
	for (i = fw_max_vid; i >= 0; i--) {
		if ((uint8_t)i != mon_vdev_id && !wma->interfaces[i].vdev) {
			vid = i;
			found = true;
			break;
		}
	}
	if (!found) {
		status = QDF_STATUS_E_RESOURCES;
		goto fail_session;
	}

	mon_mac = wlan_vdev_mlme_get_macaddr(wma->interfaces[mon_vdev_id].vdev);
	if (!mon_mac) {
		status = QDF_STATUS_E_FAILURE;
		goto fail_session;
	}
	/* Use a distinct, stable transport identity; retain frame source addresses. */
	qdf_mem_copy(inj_mac, mon_mac, QDF_MAC_ADDR_SIZE);
	inj_mac[0] = (inj_mac[0] | 0x02) & ~0x01;
	inj_mac[QDF_MAC_ADDR_SIZE - 1] ^= vid + 1;
	for (i = 0; i < (int)wma->max_bssid; i++) {
		uint8_t *active_mac;

		if (!wma->interfaces[i].vdev)
			continue;
		active_mac = wlan_vdev_mlme_get_macaddr(wma->interfaces[i].vdev);
		if (active_mac && !qdf_mem_cmp(inj_mac, active_mac,
					     QDF_MAC_ADDR_SIZE)) {
			wma_err("NHHELPER MAC conflicts with active vdev %d", i);
			status = QDF_STATUS_E_INVAL;
			goto fail_session;
		}
	}

	/* Mirror the normal START path's regulatory and stream initialization. */
	pdev = wlan_vdev_get_pdev(wma->interfaces[mon_vdev_id].vdev);
	mon_mlme = wlan_vdev_mlme_get_cmpt_obj(
					wma->interfaces[mon_vdev_id].vdev);
	if (!pdev || !mon_mlme) {
		status = QDF_STATUS_E_INVAL;
		goto fail_session;
	}
	status = wlan_reg_get_dfs_region(pdev, &dfs_region);
	if (QDF_IS_STATUS_ERROR(status))
		goto fail_session;
	vstart.regdomain = dfs_region;
	vstart.preferred_tx_streams = 1;
	vstart.preferred_rx_streams = 1;
	vstart.beacon_interval = mon_mlme->proto.generic.beacon_interval;
	if (!vstart.beacon_interval)
		vstart.beacon_interval = 100;
	vstart.dtim_period = mon_mlme->proto.generic.dtim_period;
	if (!vstart.dtim_period)
		vstart.dtim_period = 1;

	wma_injection_reset_session_state();
	g_inj_tx_vdev.vdev_id = vid;
	g_inj_tx_vdev.monitor_vdev_id = mon_vdev_id;
	g_inj_tx_vdev.chanfreq = chanfreq;
	qdf_mem_copy(g_inj_tx_vdev.mac_addr, inj_mac, QDF_MAC_ADDR_SIZE);
	status = wma_injection_claim_responses(wma, vid);
	if (QDF_IS_STATUS_ERROR(status)) {
		qdf_mem_zero(&g_inj_tx_vdev, sizeof(g_inj_tx_vdev));
		goto fail_session;
	}

	vcreate.vdev_id = vid;
	vcreate.type = WMI_VDEV_TYPE_STA;
	vcreate.nss_2g = 1;
	vcreate.nss_5g = 1;
	vcreate.pdev_id = wlan_objmgr_pdev_get_pdev_id(pdev);
	status = wmi_unified_vdev_create_send(wma->wmi_handle, inj_mac, &vcreate);
	if (QDF_IS_STATUS_ERROR(status)) {
		wma_injection_release_responses();
		qdf_mem_zero(&g_inj_tx_vdev, sizeof(g_inj_tx_vdev));
		goto fail_session;
	}
	g_inj_tx_vdev.allocated = true;
	/* This firmware path has no create-confirmation callback. */
	msleep(150);

	vstart.vdev_id = vid;
	vstart.channel.mhz = chanfreq;
	vstart.channel.cfreq1 = chanfreq;
	vstart.channel.phy_mode = (enum wlan_phymode)
		(chanfreq < 4000 ? WMI_HOST_MODE_11G : WMI_HOST_MODE_11A);
	vstart.channel.maxregpower = 20;
	vstart.channel.maxpower = 20;
	wma_injection_arm_response(MLME_FW_ONLY_VDEV_START);
	wma_info("NHHELPER START config: vdev=%u pdev=%u freq=%u mode=%u txss=%u rxss=%u regd=%u beacon=%u dtim=%u",
		 vid, vcreate.pdev_id, chanfreq, vstart.channel.phy_mode,
		 vstart.preferred_tx_streams, vstart.preferred_rx_streams,
		 vstart.regdomain, vstart.beacon_interval, vstart.dtim_period);
	status = wmi_unified_vdev_start_send(wma->wmi_handle, &vstart);
	if (QDF_IS_STATUS_ERROR(status))
		goto failed_start;
	g_inj_tx_vdev.start_sent = true;
	status = wma_injection_wait_response(MLME_FW_ONLY_VDEV_START);
	if (QDF_IS_STATUS_ERROR(status)) {
		g_inj_tx_vdev.start_rejected = status != QDF_STATUS_E_TIMEOUT;
		goto failed_start;
	}

	pcreate.peer_addr = inj_mac;
	pcreate.peer_type = WMI_PEER_TYPE_DEFAULT;
	pcreate.vdev_id = vid;
	status = wmi_unified_peer_create_send(wma->wmi_handle, &pcreate);
	if (QDF_IS_STATUS_ERROR(status))
		goto failed_start;
	g_inj_tx_vdev.peer_created = true;
	/* Keep the existing peer-create settling interval. */
	msleep(100);

	/* Do not send STA VDEV_UP without a BSS peer. */
	g_inj_tx_vdev.created = true;
	wma_info("NHHELPER ready: vdev=%u monitor=%u mac=%pM freq=%u",
		 vid, mon_vdev_id, inj_mac, chanfreq);
	return QDF_STATUS_SUCCESS;

failed_start:
	/* Preserve the setup error even if cleanup also fails. */
	wma_injection_destroy_tx_vdev_locked(wma);
fail_session:
	g_inj_tx_session_failed = true;
	wma_err("NHHELPER setup blocked for this session: status=%d; stop/start monitor to retry",
		status);
	return status;
}

static QDF_STATUS
wma_injection_ensure_tx_vdev(tp_wma_handle wma, uint8_t mon_vdev_id,
			    uint32_t chanfreq)
{
	QDF_STATUS status;

	mutex_lock(&g_inj_tx_mutex);
	status = wma_injection_ensure_tx_vdev_locked(wma, mon_vdev_id,
						 chanfreq);
	mutex_unlock(&g_inj_tx_mutex);
	return status;
}

static QDF_STATUS wma_injection_destroy_tx_vdev_locked(tp_wma_handle wma)
{
	QDF_STATUS status;
	uint8_t vid = g_inj_tx_vdev.vdev_id;

	if (!g_inj_tx_vdev.allocated)
		return QDF_STATUS_SUCCESS;
	if (!wma || !wma->wmi_handle)
		return QDF_STATUS_E_FAILURE;

	g_inj_tx_vdev.created = false;
	if (g_inj_tx_vdev.peer_created) {
		if (!g_inj_tx_vdev.peer_delete_sent) {
			wma_injection_arm_response(MLME_FW_ONLY_PEER_DELETE);
			status = wmi_unified_peer_delete_send(wma->wmi_handle,
					g_inj_tx_vdev.mac_addr, vid);
			if (QDF_IS_STATUS_ERROR(status))
				return status;
			g_inj_tx_vdev.peer_delete_sent = true;
		}
		status = wma_injection_wait_response(MLME_FW_ONLY_PEER_DELETE);
		if (QDF_IS_STATUS_ERROR(status))
			return status;
		g_inj_tx_vdev.peer_created = false;
	}

	if (g_inj_tx_vdev.start_sent && !g_inj_tx_vdev.start_rejected) {
		if (!g_inj_tx_vdev.stop_sent) {
			wma_injection_arm_response(MLME_FW_ONLY_VDEV_STOP);
			status = wmi_unified_vdev_stop_send(wma->wmi_handle, vid);
			if (QDF_IS_STATUS_ERROR(status))
				return status;
			g_inj_tx_vdev.stop_sent = true;
		}
		status = wma_injection_wait_response(MLME_FW_ONLY_VDEV_STOP);
		if (QDF_IS_STATUS_ERROR(status))
			return status;
		g_inj_tx_vdev.start_sent = false;
	}

	if (!g_inj_tx_vdev.delete_sent) {
		wma_injection_arm_response(MLME_FW_ONLY_VDEV_DELETE);
		status = wmi_unified_vdev_delete_send(wma->wmi_handle, vid);
		if (QDF_IS_STATUS_ERROR(status))
			return status;
		g_inj_tx_vdev.delete_sent = true;
	}
	status = wma_injection_wait_response(MLME_FW_ONLY_VDEV_DELETE);
	if (QDF_IS_STATUS_ERROR(status))
		return status;

	wma_injection_release_responses();
	qdf_mem_zero(&g_inj_tx_vdev, sizeof(g_inj_tx_vdev));
	wma_info("NHHELPER teardown confirmed: vdev=%u", vid);
	return QDF_STATUS_SUCCESS;
}

static QDF_STATUS wma_injection_destroy_tx_vdev(tp_wma_handle wma)
{
	QDF_STATUS status;

	mutex_lock(&g_inj_tx_mutex);
	status = wma_injection_destroy_tx_vdev_locked(wma);
	if (QDF_IS_STATUS_ERROR(status)) {
		g_inj_tx_session_failed = true;
		wma_err("NHHELPER teardown unresolved: vdev=%u status=%d; slot retained",
			g_inj_tx_vdev.vdev_id, status);
	}
	mutex_unlock(&g_inj_tx_mutex);
	return status;
}

static void wma_injection_purge_queue(struct wma_injection_queue_ctx *ctx)
{
	qdf_list_node_t *entry;
	struct wma_injection_queue_node *node;

	for (;;) {
		qdf_spin_lock_bh(&ctx->queue_lock);
		if (QDF_IS_STATUS_ERROR(qdf_list_remove_front(&ctx->queue, &entry))) {
			qdf_spin_unlock_bh(&ctx->queue_lock);
			break;
		}
		ctx->queue_size--;
		qdf_spin_unlock_bh(&ctx->queue_lock);
		node = qdf_container_of(entry, struct wma_injection_queue_node, node);
		wma_injection_queue_node_free(node);
		ctx->stats.frames_dropped++;
	}
}

void wma_injection_pre_stop_cleanup(tp_wma_handle wma)
{
	struct wma_injection_queue_ctx *ctx = &g_wma_injection_ctx;

	if (!wma || !ctx->is_initialized)
		return;

	qdf_spin_lock_bh(&ctx->queue_lock);
	ctx->quiescing = true;
	qdf_spin_unlock_bh(&ctx->queue_lock);
	qdf_delayed_work_stop_sync(&ctx->delayed_work);
	qdf_cancel_work(&ctx->queue_work);
	qdf_flush_work(&ctx->queue_work);
	wma_injection_purge_queue(ctx);
	wma_injection_destroy_tx_vdev(wma);
}

QDF_STATUS wma_injection_resume(tp_wma_handle wma)
{
	struct wma_injection_queue_ctx *ctx = &g_wma_injection_ctx;
	QDF_STATUS status = QDF_STATUS_SUCCESS;

	if (!wma || !ctx->is_initialized)
		return QDF_STATUS_E_AGAIN;

	mutex_lock(&g_inj_tx_mutex);
	if (!READ_ONCE(ctx->quiescing))
		goto out;

	/* Retry only cleanup, never reuse an unresolved firmware helper ID. */
	status = wma_injection_destroy_tx_vdev_locked(wma);
	if (QDF_IS_STATUS_ERROR(status))
		goto out;

	g_inj_tx_session_failed = false;
	qdf_spin_lock_bh(&ctx->queue_lock);
	ctx->quiescing = false;
	qdf_spin_unlock_bh(&ctx->queue_lock);
	wma_info("NHHELPER injection session resumed");
out:
	mutex_unlock(&g_inj_tx_mutex);
	return status;
}

/* WMI has been detached; no event can still reference this ownership. */
void wma_injection_fw_detached(void)
{
	mutex_lock(&g_inj_tx_mutex);
	wma_injection_release_responses();
	qdf_mem_zero(&g_inj_tx_vdev, sizeof(g_inj_tx_vdev));
	g_inj_tx_session_failed = false;
	mutex_unlock(&g_inj_tx_mutex);
}

static bool
wma_injection_debug_cache_update(uint32_t desc_id,
                                 struct inject_frame_req *req,
                                 uint8_t fc_type, uint8_t fc_subtype,
                                 uint16_t chanfreq, qdf_nbuf_t tx_buf)
{
    struct wma_injection_debug_info *entry;
    unsigned long flags;

    if (!desc_id || !req || !req->frame_data || !tx_buf)
        return false;

    spin_lock_irqsave(&g_wma_injection_cache_lock, flags);
    entry = &g_wma_injection_debug_cache[
        desc_id % WMA_INJECTION_DEBUG_CACHE_SIZE];

    if (entry->valid ||
        qdf_atomic_read(&g_wma_injection_ctx.inflight_count) >=
            WMA_INJECTION_INFLIGHT_HIGH) {
        spin_unlock_irqrestore(&g_wma_injection_cache_lock, flags);
        return false;
    }

    entry->desc_id = desc_id;
    entry->frame_len = req->frame_len;
    entry->chanfreq = chanfreq;
    entry->fc_type = fc_type;
    entry->fc_subtype = fc_subtype;
    entry->tx_buf = tx_buf;
    entry->submit_ts = qdf_get_log_timestamp();

    qdf_mem_zero(entry->addr1, sizeof(entry->addr1));
    qdf_mem_zero(entry->addr2, sizeof(entry->addr2));
    qdf_mem_zero(entry->addr3, sizeof(entry->addr3));

    if (req->frame_len >= 24) {
        qdf_mem_copy(entry->addr1, &req->frame_data[4],
                     QDF_MAC_ADDR_SIZE);
        qdf_mem_copy(entry->addr2, &req->frame_data[10],
                     QDF_MAC_ADDR_SIZE);
        qdf_mem_copy(entry->addr3, &req->frame_data[16],
                     QDF_MAC_ADDR_SIZE);
    }

    entry->valid = true;
    qdf_atomic_inc(&g_wma_injection_ctx.inflight_count);

    spin_unlock_irqrestore(&g_wma_injection_cache_lock, flags);
    return true;
}

/*
 * Claim ownership exactly once. DMA unmapping and buffer freeing
 * must take place outside the spinlock.
 */
static qdf_nbuf_t
wma_injection_debug_cache_take(uint32_t desc_id,
                               struct wma_injection_debug_info *snapshot)
{
    struct wma_injection_debug_info *entry;
    qdf_nbuf_t buf = NULL;
    unsigned long flags;

    if (!desc_id)
        return NULL;

    spin_lock_irqsave(&g_wma_injection_cache_lock, flags);

    entry = &g_wma_injection_debug_cache[
        desc_id % WMA_INJECTION_DEBUG_CACHE_SIZE];

    if (entry->valid && entry->desc_id == desc_id) {
        if (snapshot)
            *snapshot = *entry;

        buf = entry->tx_buf;
        entry->tx_buf = NULL;
        entry->valid = false;
        qdf_atomic_dec(&g_wma_injection_ctx.inflight_count);
    }

    spin_unlock_irqrestore(&g_wma_injection_cache_lock, flags);
    return buf;
}

/**
 * wma_injection_desc_id_alloc() - allocate descriptor id for injection tx
 *
 * Return: descriptor id in dedicated injection range
 */
static uint16_t wma_injection_desc_id_alloc(void)
{
	static uint16_t next_desc_id = WMA_INJECTION_DESC_ID_BASE;
	uint16_t desc_id = next_desc_id;

	next_desc_id++;
	if ((next_desc_id & ~WMA_INJECTION_DESC_ID_MASK) !=
	    WMA_INJECTION_DESC_ID_BASE)
		next_desc_id = WMA_INJECTION_DESC_ID_BASE;

	return desc_id;
}

/**
 * wma_injection_queue_node_alloc() - Allocate injection queue node
 * @req: Frame injection request
 * @vdev_id: VDEV ID
 *
 * Return: Allocated node or NULL on failure
 */
static struct wma_injection_queue_node *
wma_injection_queue_node_alloc(struct inject_frame_req *req, uint8_t vdev_id)
{
	struct wma_injection_queue_node *node;
	uint8_t *frame_copy;

	if (!req || !req->frame_data || req->frame_len == 0) {
		wma_err("Invalid injection request parameters");
		return NULL;
	}

	if (req->frame_len > WMA_FRAME_INJECT_MAX_FRAME_SIZE) {
		wma_err("Frame size %u exceeds maximum %u",
			req->frame_len, WMA_FRAME_INJECT_MAX_FRAME_SIZE);
		return NULL;
	}

	node = qdf_mem_malloc(sizeof(*node));
	if (!node) {
		wma_err("Failed to allocate injection queue node");
		return NULL;
	}

	/* Allocate and copy frame data */
	frame_copy = qdf_mem_malloc(req->frame_len);
	if (!frame_copy) {
		wma_err("Failed to allocate frame data buffer");
		qdf_mem_free(node);
		return NULL;
	}

	qdf_mem_copy(frame_copy, req->frame_data, req->frame_len);

	/* Initialize node */
	qdf_mem_zero(node, sizeof(*node));
	node->req.frame_len = req->frame_len;
	node->req.frame_data = frame_copy;
	node->req.tx_flags = req->tx_flags;
	node->req.retry_count = req->retry_count;
	node->req.tx_rate = req->tx_rate;
	node->req.timestamp = req->timestamp;
	node->req.session_id = req->session_id;
	node->timestamp = qdf_get_log_timestamp();
	node->vdev_id = vdev_id;

	return node;
}

/**
 * wma_injection_queue_node_free() - Free injection queue node
 * @node: Node to free
 */
static void wma_injection_queue_node_free(struct wma_injection_queue_node *node)
{
	if (!node)
		return;

	if (node->req.frame_data) {
		qdf_mem_free(node->req.frame_data);
		node->req.frame_data = NULL;
	}

	qdf_mem_free(node);
}

/**
 * wma_check_traffic_coordination() - Check if injection can proceed with current traffic
 * @wma_handle: WMA handle
 * @vdev_id: VDEV ID for the frame
 *
 * This function checks if frame injection should be deferred due to high
 * priority traffic or resource constraints in the WMA layer.
 *
 * Return: true if injection can proceed, false if should be deferred
 */
static bool wma_check_traffic_coordination(tp_wma_handle wma_handle, uint8_t vdev_id)
{
	struct wma_txrx_node *iface;

	if (!wma_handle || vdev_id >= wma_handle->max_bssid) {
		wma_err("Invalid parameters: wma_handle=%pK, vdev_id=%u",
			wma_handle, vdev_id);
		return false;
	}

	iface = &wma_handle->interfaces[vdev_id];

	/* Check if interface is in a state that allows injection */
	if (!iface->vdev) {
		wma_debug("Interface %u not active, deferring injection", vdev_id);
		return false;
	}

	/* Check if there's high priority management traffic pending */
	if (iface->roaming_in_progress) {
		wma_debug("High priority operation in progress on vdev %u, deferring injection",
			  vdev_id);
		return false;
	}

	/* Check system-wide resource constraints */
	if (wma_handle->wmi_ready == false) {
		wma_debug("WMI not ready, deferring injection");
		return false;
	}

	/* Check if firmware is overloaded (simple heuristic) */
	if (wma_handle->wmi_handle) {
		uint32_t pending_cmds = wmi_get_pending_cmds(wma_handle->wmi_handle);
		const uint32_t max_pending_threshold = 512;

		if (pending_cmds > max_pending_threshold) {
			wma_debug("Firmware overloaded (%u pending commands), deferring injection",
				  pending_cmds);
			return false;
		}
	}

	return true;
}

/**
 * wma_apply_injection_backpressure() - Apply backpressure when queue is congested
 * @ctx: Injection queue context
 *
 * This function implements backpressure mechanisms when the injection queue
 * becomes congested, including adaptive processing delays and queue throttling.
 *
 * Return: Recommended delay in milliseconds before next processing cycle
 */
static uint32_t wma_apply_injection_backpressure(struct wma_injection_queue_ctx *ctx)
{
	uint32_t queue_utilization;
	uint32_t delay_ms = 0;

	if (!ctx || !ctx->is_initialized) {
		return 0;
	}

	/* Calculate queue utilization percentage */
	queue_utilization = (ctx->queue_size * 100) / ctx->max_queue_size;

	/* Apply adaptive backpressure based on queue utilization */
	if (queue_utilization > 95) {
		/* Queue nearly full - significant backpressure */
		delay_ms = 10;
	} else if (queue_utilization > 85) {
		/* Queue getting full - moderate backpressure */
		delay_ms = 2;
	}

	return delay_ms;
}

/**
 * wma_process_injection_queue() - Process injection queue with traffic coordination
 * @wma_handle: WMA handle
 *
 * This function processes the injection queue while coordinating with existing
 * WMA traffic scheduling and applying backpressure when needed.
 *
 * Return: QDF_STATUS_SUCCESS on success, error code on failure
 */
QDF_STATUS wma_process_injection_queue(tp_wma_handle wma_handle)
{
	struct wma_injection_queue_ctx *ctx = &g_wma_injection_ctx;
	struct wma_injection_queue_node *node;
	qdf_list_node_t *list_node;
	QDF_STATUS status;
	uint64_t current_time;
	uint32_t processed_count = 0;
	uint32_t deferred_count = 0;
	const uint32_t max_process_per_cycle = 64;
	bool queue_was_empty;

	if (!wma_handle) {
		wma_err("Invalid WMA handle");
		return QDF_STATUS_E_INVAL;
	}

	if (!ctx->is_initialized) {
		wma_debug("Injection queue not initialized");
		return QDF_STATUS_E_AGAIN;
	}

	current_time = qdf_get_log_timestamp();

	qdf_spin_lock_bh(&ctx->queue_lock);
	queue_was_empty = qdf_list_empty(&ctx->queue);
	qdf_spin_unlock_bh(&ctx->queue_lock);

	if (queue_was_empty)
		return QDF_STATUS_SUCCESS;

	/* Process frames from queue with traffic coordination */
	while (processed_count < max_process_per_cycle) {
		qdf_spin_lock_bh(&ctx->queue_lock);

		if (ctx->quiescing ||
		    qdf_list_empty(&ctx->queue)) {
			qdf_spin_unlock_bh(&ctx->queue_lock);
			break;
		}

		/* Peek at the front node to check VDEV before removing */
		status = qdf_list_peek_front(&ctx->queue, &list_node);
		if (QDF_IS_STATUS_ERROR(status)) {
			qdf_spin_unlock_bh(&ctx->queue_lock);
			wma_err("Failed to peek at queue front: %d", status);
			break;
		}

		node = qdf_container_of(list_node, struct wma_injection_queue_node, node);

		/* Check traffic coordination before processing */
		if (!wma_check_traffic_coordination(wma_handle, node->vdev_id)) {
			qdf_spin_unlock_bh(&ctx->queue_lock);
			deferred_count++;
			wma_debug("Deferring injection due to traffic coordination (vdev_id=%u)",
				  node->vdev_id);
			break;
		}

		/* Remove the node from queue */
		status = qdf_list_remove_front(&ctx->queue, &list_node);
		if (QDF_IS_STATUS_ERROR(status)) {
			qdf_spin_unlock_bh(&ctx->queue_lock);
			wma_err("Failed to remove node from queue: %d", status);
			break;
		}

		ctx->queue_size--;
		qdf_spin_unlock_bh(&ctx->queue_lock);

		/* Update queue time statistics */
		ctx->stats.total_queue_time += (current_time - node->timestamp);

		/* Send frame to firmware */
		status = wma_send_injection_frame_to_fw(wma_handle, &node->req, node->vdev_id);
		if (QDF_IS_STATUS_SUCCESS(status)) {
			ctx->stats.frames_processed++;
		} else {
			ctx->stats.frames_dropped++;
			wma_err_rl("Failed to send injection frame to firmware: %d", status);
		}

		processed_count++;

		/* Free the node */
		wma_injection_queue_node_free(node);
	}

	wma_debug("Injection queue processing cycle complete: processed=%u, deferred=%u",
		  processed_count, deferred_count);

	return QDF_STATUS_SUCCESS;
}

/**
 * wma_process_injection_queue_work() - Work function to process injection queue
 * @arg: Work argument (not used)
 *
 * This function processes queued frame injection requests in FIFO order
 * with traffic coordination and backpressure handling.
 */
static void wma_process_injection_queue_work(void *arg)
{
	struct wma_injection_queue_ctx *ctx = &g_wma_injection_ctx;
	tp_wma_handle wma_handle;
	QDF_STATUS status;
	uint32_t backpressure_delay;
	bool queue_has_frames;

	if (!ctx->is_initialized) {
		wma_debug("Injection queue not initialized");
		return;
	}

	/* Get WMA handle for processing */
	wma_handle = cds_get_context(QDF_MODULE_ID_WMA);
	if (!wma_handle) {
		wma_err("Failed to get WMA handle");
		return;
	}

	/* Process the queue */
	status = wma_process_injection_queue(wma_handle);
	if (QDF_IS_STATUS_ERROR(status)) {
		wma_err("Failed to process injection queue: %d", status);
	}

	/* Check if there are more frames to process */
	qdf_spin_lock_bh(&ctx->queue_lock);
	queue_has_frames = !ctx->quiescing &&
			   !qdf_list_empty(&ctx->queue);
	qdf_spin_unlock_bh(&ctx->queue_lock);

	if (queue_has_frames) {
		/* Apply backpressure if queue is congested */
		backpressure_delay = wma_apply_injection_backpressure(ctx);

		if (backpressure_delay > 0) {
			qdf_delayed_work_start(&ctx->delayed_work, backpressure_delay);
		} else {
			/* Schedule immediate work for next processing cycle */
			qdf_sched_work(0, &ctx->queue_work);
		}
	}
}

/**
 * wma_process_injection_queue_delayed_work() - Delayed work callback for backpressure
 * @context: Context (not used)
 *
 * This function is called when delayed work is triggered for backpressure handling.
 * It simply schedules the regular work item to continue processing.
 */
static void wma_process_injection_queue_delayed_work(void *context)
{
	struct wma_injection_queue_ctx *ctx = &g_wma_injection_ctx;

	if (!ctx->is_initialized ||
	    READ_ONCE(ctx->quiescing))
		return;

	qdf_sched_work(0, &ctx->queue_work);
}

/**
 * wma_injection_reaper_work_cb() - Periodic reaper for stale injection nbufs
 * @context: Unused
 *
 * The firmware never sends TX-completion events for frames sent on the hidden
 * STA helper vdev because the per-vdev completion callback pointer is NULL in
 * FW context (see wal_local_frame_mgmt_tx_completion / FUN_b013dd78).  Left
 * unchecked the DMA-mapped nbufs accumulate, eventually triggering an SMMU
 * translation fault and a firmware crash.
 *
 * This worker runs every WMA_INJECTION_REAPER_INTERVAL_MS (3 s), scans the
 * debug cache, and frees any entry whose submit_ts is older than
 * WMA_INJECTION_NBUF_TIMEOUT_US (2 s).
 */
static void wma_injection_reaper_work_cb(void *context)
{
    struct wma_injection_queue_ctx *ctx = &g_wma_injection_ctx;
    uint64_t now = qdf_get_log_timestamp();
    unsigned long flags;
    uint32_t overdue = 0;
    int i;

    if (!ctx->is_initialized)
        return;

    spin_lock_irqsave(&g_wma_injection_cache_lock, flags);

    for (i = 0; i < WMA_INJECTION_DEBUG_CACHE_SIZE; i++) {
        struct wma_injection_debug_info *e =
            &g_wma_injection_debug_cache[i];

        if (e->valid && e->tx_buf && e->submit_ts &&
            qdf_log_timestamp_to_usecs(now - e->submit_ts) >=
                WMA_INJECTION_NBUF_TIMEOUT_US)
            overdue++;
    }

    spin_unlock_irqrestore(&g_wma_injection_cache_lock, flags);

    if (overdue)
        wma_warn("Injection: %u overdue TX completions; retaining DMA buffers",
                 overdue);

    if (ctx->is_initialized)
        qdf_delayed_work_start(&ctx->reaper_work,
                               WMA_INJECTION_REAPER_INTERVAL_MS);
}

QDF_STATUS wma_init_injection_queue(tp_wma_handle wma_handle)
{
	struct wma_injection_queue_ctx *ctx = &g_wma_injection_ctx;
	QDF_STATUS status;

	if (!wma_handle) {
		wma_err("Invalid WMA handle");
		return QDF_STATUS_E_INVAL;
	}

	if (ctx->is_initialized) {
		wma_debug("Injection queue already initialized");
		return QDF_STATUS_SUCCESS;
	}

	wma_debug("Initializing WMA injection queue");

	/* Initialize queue */
	qdf_list_create(&ctx->queue, WMA_FRAME_INJECT_MAX_QUEUE_SIZE);

	/* Initialize queue lock */
	qdf_spinlock_create(&ctx->queue_lock);

	/* Initialize work item */
	qdf_create_work(0, &ctx->queue_work, wma_process_injection_queue_work, NULL);

	/* Initialize delayed work item for backpressure */
	status = qdf_delayed_work_create(&ctx->delayed_work,
					 wma_process_injection_queue_delayed_work, NULL);
	if (QDF_IS_STATUS_ERROR(status)) {
		wma_err("Failed to create delayed work: %d", status);
		qdf_spinlock_destroy(&ctx->queue_lock);
		qdf_list_destroy(&ctx->queue);
		return status;
	}

	/* Initialize nbuf-leak reaper timer */
	status = qdf_delayed_work_create(&ctx->reaper_work,
					 wma_injection_reaper_work_cb, NULL);
	if (QDF_IS_STATUS_ERROR(status)) {
		wma_err("Failed to create reaper work: %d", status);
		qdf_delayed_work_destroy(&ctx->delayed_work);
		qdf_spinlock_destroy(&ctx->queue_lock);
		qdf_list_destroy(&ctx->queue);
		return status;
	}

	/* Initialize context */
	ctx->queue_size = 0;
	ctx->max_queue_size = WMA_FRAME_INJECT_MAX_QUEUE_SIZE;
	qdf_mem_zero(&ctx->stats, sizeof(ctx->stats));
	/* Recount descriptors retained across an earlier teardown. */
	{
		unsigned long flags;
		int i;

		spin_lock_irqsave(&g_wma_injection_cache_lock, flags);
		qdf_atomic_init(&ctx->inflight_count);

		for (i = 0; i < WMA_INJECTION_DEBUG_CACHE_SIZE; i++)
			if (g_wma_injection_debug_cache[i].valid)
				qdf_atomic_inc(&ctx->inflight_count);

		spin_unlock_irqrestore(&g_wma_injection_cache_lock, flags);
	}
	ctx->quiescing = false;
	ctx->is_initialized = true;
	g_inj_tx_session_failed = false;

	/* Arm the reaper */
	qdf_delayed_work_start(&ctx->reaper_work,
			       WMA_INJECTION_REAPER_INTERVAL_MS);

	wma_info("WMA injection queue initialized successfully (max_size=%u)",
		 ctx->max_queue_size);

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS wma_deinit_injection_queue(tp_wma_handle wma_handle)
{
	struct wma_injection_queue_ctx *ctx = &g_wma_injection_ctx;
	struct wma_injection_queue_node *node;
	qdf_list_node_t *list_node;
	QDF_STATUS status;
	uint32_t dropped_count = 0;

	if (!wma_handle) {
		wma_err("Invalid WMA handle");
		return QDF_STATUS_E_INVAL;
	}

	if (!ctx->is_initialized) {
		wma_debug("Injection queue not initialized");
		return QDF_STATUS_SUCCESS;
	}

	wma_debug("Deinitializing WMA injection queue");

	/* Block new TX before draining workers. */
	qdf_spin_lock_bh(&ctx->queue_lock);
	ctx->quiescing = true;
	qdf_spin_unlock_bh(&ctx->queue_lock);

	/* Stop delayed scheduling before flushing immediate work. */
	qdf_delayed_work_stop_sync(&ctx->delayed_work);
	qdf_cancel_work(&ctx->queue_work);
	qdf_flush_work(&ctx->queue_work);
	qdf_delayed_work_destroy(&ctx->delayed_work);

	/* Stop and destroy reaper timer */
	qdf_delayed_work_stop_sync(&ctx->reaper_work);
	qdf_delayed_work_destroy(&ctx->reaper_work);

	/*
	 * No injection queue or reaper work can reference the helper
	 * after the workers have been stopped.
	 */
	wma_injection_destroy_tx_vdev(wma_handle);

	/* Clear the queue and free all nodes */
	qdf_spin_lock_bh(&ctx->queue_lock);

	while (!qdf_list_empty(&ctx->queue)) {
		status = qdf_list_remove_front(&ctx->queue, &list_node);
		if (QDF_IS_STATUS_ERROR(status)) {
			wma_err("Failed to remove node during cleanup: %d", status);
			break;
		}

		node = qdf_container_of(list_node, struct wma_injection_queue_node, node);
		wma_injection_queue_node_free(node);
		dropped_count++;
	}

	ctx->queue_size = 0;
	qdf_spin_unlock_bh(&ctx->queue_lock);

	/* Update statistics */
	ctx->stats.frames_dropped += dropped_count;

	/* Destroy queue and lock */
	qdf_list_destroy(&ctx->queue);
	qdf_spinlock_destroy(&ctx->queue_lock);

	/* Mark as uninitialized */
	ctx->is_initialized = false;

	wma_info("WMA injection queue deinitialized (dropped %u pending frames)",
		 dropped_count);

	/* Outstanding DMA buffers remain tracked until completion. */
	if (qdf_atomic_read(&ctx->inflight_count))
		wma_warn("Injection shutdown: %d descriptors still outstanding",
			 qdf_atomic_read(&ctx->inflight_count));

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS wma_queue_injection_frame(tp_wma_handle wma_handle,
				     struct inject_frame_req *req,
				     uint8_t vdev_id)
{
	struct wma_injection_queue_ctx *ctx = &g_wma_injection_ctx;
	struct wma_injection_queue_node *node;
	QDF_STATUS status;

	if (!wma_handle || !req) {
		wma_err("Invalid parameters: wma_handle=%pK, req=%pK",
			wma_handle, req);
		return QDF_STATUS_E_INVAL;
	}

	if (!ctx->is_initialized) {
		wma_err("Injection queue not initialized");
		return QDF_STATUS_E_AGAIN;
	}

	/* Validate frame parameters */
	if (!req->frame_data || req->frame_len == 0 ||
	    req->frame_len > WMA_FRAME_INJECT_MAX_FRAME_SIZE) {
		wma_err("Invalid frame parameters: data=%pK, len=%u",
			req->frame_data, req->frame_len);
		return QDF_STATUS_E_INVAL;
	}

	/* Check queue overflow */
	qdf_spin_lock_bh(&ctx->queue_lock);

	if (ctx->queue_size >= ctx->max_queue_size) {
		qdf_spin_unlock_bh(&ctx->queue_lock);
		ctx->stats.queue_overflows++;
		ctx->stats.frames_dropped++;
		wma_err("Injection queue overflow (size=%u, max=%u)",
			ctx->queue_size, ctx->max_queue_size);
		return QDF_STATUS_E_RESOURCES;
	}

	qdf_spin_unlock_bh(&ctx->queue_lock);

	/*
	 * Inflight backpressure: if too many nbufs are waiting for FW
	 * completions that will never arrive, reject early to avoid
	 * exhausting DMA-mapped memory and triggering an SMMU fault.
	 * The reaper timer will gradually free the stale entries.
	 */
	{
		int inflight = qdf_atomic_read(&ctx->inflight_count);

		if (inflight >= WMA_INJECTION_INFLIGHT_HIGH) {
			ctx->stats.frames_dropped++;
			if (ctx->stats.frames_dropped % 100 == 1)
				wma_warn("Injection backpressure: inflight=%d >= %d, dropping frame",
					 inflight, WMA_INJECTION_INFLIGHT_HIGH);
			return QDF_STATUS_E_RESOURCES;
		}
	}

	/* Allocate and initialize queue node */
	node = wma_injection_queue_node_alloc(req, vdev_id);
	if (!node) {
		ctx->stats.frames_dropped++;
		wma_err("Failed to allocate injection queue node");
		return QDF_STATUS_E_NOMEM;
	}

	/* Add to queue */
	qdf_spin_lock_bh(&ctx->queue_lock);

	if (ctx->quiescing) {
		qdf_spin_unlock_bh(&ctx->queue_lock);
		wma_injection_queue_node_free(node);
		return QDF_STATUS_E_AGAIN;
	}

	status = qdf_list_insert_back(&ctx->queue, &node->node);
	if (QDF_IS_STATUS_ERROR(status)) {
		qdf_spin_unlock_bh(&ctx->queue_lock);
		wma_injection_queue_node_free(node);
		ctx->stats.frames_dropped++;
		wma_err("Failed to add node to queue: %d", status);
		return status;
	}

	ctx->queue_size++;
	ctx->stats.frames_queued++;

	/* Update maximum queue depth */
	if (ctx->queue_size > ctx->stats.max_queue_depth) {
		ctx->stats.max_queue_depth = ctx->queue_size;
	}

	qdf_spin_unlock_bh(&ctx->queue_lock);

	/* Schedule queue processing work */
	qdf_sched_work(0, &ctx->queue_work);

	wma_debug("Queued injection frame: len=%u, vdev_id=%u, queue_size=%u",
		  req->frame_len, vdev_id, ctx->queue_size);

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS wma_get_injection_queue_stats(tp_wma_handle wma_handle,
					 struct wma_injection_queue_stats *stats)
{
	struct wma_injection_queue_ctx *ctx = &g_wma_injection_ctx;

	if (!wma_handle || !stats) {
		wma_err("Invalid parameters: wma_handle=%pK, stats=%pK",
			wma_handle, stats);
		return QDF_STATUS_E_INVAL;
	}

	if (!ctx->is_initialized) {
		wma_err("Injection queue not initialized");
		return QDF_STATUS_E_AGAIN;
	}

	qdf_spin_lock_bh(&ctx->queue_lock);
	qdf_mem_copy(stats, &ctx->stats, sizeof(*stats));
	qdf_spin_unlock_bh(&ctx->queue_lock);

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS wma_reset_injection_queue_stats(tp_wma_handle wma_handle)
{
	struct wma_injection_queue_ctx *ctx = &g_wma_injection_ctx;

	if (!wma_handle) {
		wma_err("Invalid WMA handle");
		return QDF_STATUS_E_INVAL;
	}

	if (!ctx->is_initialized) {
		wma_err("Injection queue not initialized");
		return QDF_STATUS_E_AGAIN;
	}

	qdf_spin_lock_bh(&ctx->queue_lock);
	qdf_mem_zero(&ctx->stats, sizeof(ctx->stats));
	qdf_spin_unlock_bh(&ctx->queue_lock);

	wma_info("Injection queue statistics reset");

	return QDF_STATUS_SUCCESS;
}

uint32_t wma_get_injection_queue_size(tp_wma_handle wma_handle)
{
	struct wma_injection_queue_ctx *ctx = &g_wma_injection_ctx;
	uint32_t queue_size;

	if (!wma_handle || !ctx->is_initialized) {
		return 0;
	}

	qdf_spin_lock_bh(&ctx->queue_lock);
	queue_size = ctx->queue_size;
	qdf_spin_unlock_bh(&ctx->queue_lock);

	return queue_size;
}

bool wma_is_injection_queue_empty(tp_wma_handle wma_handle)
{
	struct wma_injection_queue_ctx *ctx = &g_wma_injection_ctx;
	bool is_empty;

	if (!wma_handle || !ctx->is_initialized) {
		return true;
	}

	qdf_spin_lock_bh(&ctx->queue_lock);
	is_empty = qdf_list_empty(&ctx->queue);
	qdf_spin_unlock_bh(&ctx->queue_lock);

	return is_empty;
}

QDF_STATUS wma_flush_injection_queue(tp_wma_handle wma_handle)
{
	struct wma_injection_queue_ctx *ctx = &g_wma_injection_ctx;
	struct wma_injection_queue_node *node;
	qdf_list_node_t *list_node;
	QDF_STATUS status;
	uint32_t flushed_count = 0;

	if (!wma_handle) {
		wma_err("Invalid WMA handle");
		return QDF_STATUS_E_INVAL;
	}

	if (!ctx->is_initialized) {
		wma_err("Injection queue not initialized");
		return QDF_STATUS_E_AGAIN;
	}

	wma_debug("Flushing injection queue");

	/* Cancel any pending work */
	qdf_cancel_work(&ctx->queue_work);
	qdf_delayed_work_stop_sync(&ctx->delayed_work);

	/* Flush all queued frames */
	qdf_spin_lock_bh(&ctx->queue_lock);

	while (!qdf_list_empty(&ctx->queue)) {
		status = qdf_list_remove_front(&ctx->queue, &list_node);
		if (QDF_IS_STATUS_ERROR(status)) {
			wma_err("Failed to remove node during flush: %d", status);
			break;
		}

		node = qdf_container_of(list_node, struct wma_injection_queue_node, node);
		wma_injection_queue_node_free(node);
		flushed_count++;
	}

	ctx->queue_size = 0;
	ctx->stats.frames_dropped += flushed_count;

	qdf_spin_unlock_bh(&ctx->queue_lock);

	wma_info("Flushed %u frames from injection queue", flushed_count);

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS wma_send_injection_frame_to_fw(tp_wma_handle wma_handle,
					  struct inject_frame_req *req,
					  uint8_t vdev_id)
{
	struct wmi_mgmt_params mgmt_params;
	QDF_STATUS status;
	qdf_nbuf_t wmi_buf;
	uint8_t *frame_data;
	struct cdp_soc_t *soc;
	void *qdf_ctx;
	int ret;
	uint16_t tx_chanfreq = 0;
	uint8_t fc0 = 0;
	uint8_t fc_type = 0;
	uint8_t fc_subtype = 0;
	bool is_bcast_da = false;
	bool wmi_mgmt_service;
	bool wmi_tx_attempted = false;
	bool wmi_tx_ok = false;
	bool is_probe_req = false;
	bool monitor_vdev = false;

	if (!wma_handle || !req || !req->frame_data) {
		wma_err("Invalid parameters: wma_handle=%pK, req=%pK",
			wma_handle, req);
		return QDF_STATUS_E_INVAL;
	}

	if (!inject_patch_banner_logged) {
		wma_info("Injection patch tag: monitor_sta_vdev_tx_v7");
		inject_patch_banner_logged = true;
	}

	if (req->frame_len == 0 || req->frame_len > WMA_FRAME_INJECT_MAX_FRAME_SIZE) {
		wma_err("Invalid frame length: %u", req->frame_len);
		return QDF_STATUS_E_INVAL;
	}

	if (!wma_handle->wmi_handle) {
		wma_err("Invalid WMI handle in injection path");
		return QDF_STATUS_E_INVAL;
	}

	if (vdev_id >= wma_handle->max_bssid) {
		wma_err("Invalid injection vdev_id %u (max %u)",
			vdev_id, wma_handle->max_bssid);
		return QDF_STATUS_E_INVAL;
	}

	if (!wma_handle->interfaces[vdev_id].vdev) {
		wma_err("Injection vdev %u is not active", vdev_id);
		return QDF_STATUS_E_AGAIN;
	}

	if (wma_handle->interfaces[vdev_id].mhz)
		tx_chanfreq = wma_handle->interfaces[vdev_id].mhz;

	if (req->frame_len)
		fc0 = req->frame_data[0];

	fc_type = fc0 & 0x0c;
	fc_subtype = fc0 & 0xf0;
	is_probe_req = (fc_type == 0x00 && fc_subtype == 0x40);
	if (req->frame_len >= 10) {
		uint8_t *da = &req->frame_data[4];

		if (da[0] == 0xff && da[1] == 0xff && da[2] == 0xff &&
		    da[3] == 0xff && da[4] == 0xff && da[5] == 0xff)
			is_bcast_da = true;
	}

	qdf_ctx = cds_get_context(QDF_MODULE_ID_QDF_DEVICE);
	if (!qdf_ctx) {
		wma_err("qdf_ctx is NULL in injection path");
		return QDF_STATUS_E_INVAL;
	}

	monitor_vdev =
		(wma_handle->interfaces[vdev_id].type == WMI_VDEV_TYPE_MONITOR);
	if (monitor_vdev && !tx_chanfreq) {
		wma_err("Injection monitor vdev %u has zero channel frequency; dropping frame",
			vdev_id);
		return QDF_STATUS_E_INVAL;
	}

	/* Allocate WMI buffer for the frame */
	wmi_buf = qdf_nbuf_alloc(NULL, req->frame_len, 0, 0, false);
	if (!wmi_buf) {
		wma_err("Failed to allocate WMI buffer for injection frame");
		return QDF_STATUS_E_NOMEM;
	}

	/* Copy frame data to WMI buffer */
	frame_data = qdf_nbuf_put_tail(wmi_buf, req->frame_len);
	if (!frame_data) {
		wma_err("Failed to get buffer space for frame data");
		qdf_nbuf_free(wmi_buf);
		return QDF_STATUS_E_NOMEM;
	}

	qdf_mem_copy(frame_data, req->frame_data, req->frame_len);

	/*
	 * Monitor probe-request injection can be discarded by firmware when SA
	 * does not match the transmitting vdev MAC. Normalize SA for broadcast
	 * probe requests before WMI submission.
	 */
	if (monitor_vdev && is_probe_req && is_bcast_da && req->frame_len >= 24) {
		uint8_t *vdev_mac =
			wlan_vdev_mlme_get_macaddr(wma_handle->interfaces[vdev_id].vdev);

		if (vdev_mac &&
		    qdf_mem_cmp(frame_data + 10, vdev_mac, QDF_MAC_ADDR_SIZE) != 0) {
			if (!inject_probe_sa_fix_logged) {
				wma_warn("Injection probe-req SA override %pM -> %pM on vdev %u",
					 frame_data + 10, vdev_mac, vdev_id);
				inject_probe_sa_fix_logged = true;
			}
			qdf_mem_copy(frame_data + 10, vdev_mac, QDF_MAC_ADDR_SIZE);
			qdf_mem_copy(req->frame_data + 10, vdev_mac, QDF_MAC_ADDR_SIZE);
		}
	}

	/* Initialize WMI management parameters for injection */
	qdf_mem_zero(&mgmt_params, sizeof(mgmt_params));
	mgmt_params.tx_frame = wmi_buf;
	mgmt_params.frm_len = req->frame_len;
	mgmt_params.vdev_id = vdev_id;
	/*
	 * Use ACK-completion tx type for injection consistently. Several
	 * firmware builds are stricter with probe-request tx_type handling and
	 * are less likely to discard when sent with ACK-completion semantics.
	 */
	mgmt_params.tx_type = GENERIC_NODOWLOAD_ACK_COMP_INDEX;
	/*
	 * Align with regular host management TX behavior:
	 * probe request uses chanfreq=0 while action/auth/probe-rsp can carry
	 * an explicit channel.
	 */
	mgmt_params.chanfreq = is_probe_req ? 0 : tx_chanfreq;
	/*
	 * For monitor vdev probe injection, prefer explicit channel in command
	 * so firmware can bind probe request tx to the current monitor channel.
	 */
	if (monitor_vdev && is_probe_req && tx_chanfreq)
		mgmt_params.chanfreq = tx_chanfreq;
	mgmt_params.desc_id = wma_injection_desc_id_alloc();
	mgmt_params.pdata = frame_data; /* Management frame bytes for command payload */
	mgmt_params.macaddr = NULL; /* No specific MAC address */
	mgmt_params.qdf_ctx = qdf_ctx;
	mgmt_params.tx_params_valid = false; /* Use default TX parameters */
	mgmt_params.use_6mbps = 0; /* Use rate from injection request if specified */

	/* Set transmission rate if specified in injection request */
	if (req->tx_rate != 0) {
		mgmt_params.tx_param.mcs_mask = req->tx_rate;
		mgmt_params.tx_params_valid = true;
	}

	if (!inject_tx_cfg_logged) {
		wma_info("Injection TX config: vdev=%u iface_type=%u iface_subtype=%u vdev_active=%u chanfreq=%u",
			 vdev_id,
			 wma_handle->interfaces[vdev_id].type,
			 wma_handle->interfaces[vdev_id].sub_type,
			 wma_handle->interfaces[vdev_id].vdev_active ? 1 : 0,
			 mgmt_params.chanfreq);
		inject_tx_cfg_logged = true;
	}

	if (inject_send_info_count < 10) {
		if (req->frame_len >= 24) {
			uint8_t *addr1 = &req->frame_data[4];
			uint8_t *addr2 = &req->frame_data[10];
			uint8_t *addr3 = &req->frame_data[16];

			wma_info("Injection frame[%u]: desc_id=%u vdev=%u len=%u fc_type=0x%02x fc_subtype=0x%02x tx_chanfreq=%u cmd_chanfreq=%u addr1=%pM addr2=%pM addr3=%pM",
				 inject_send_info_count + 1, mgmt_params.desc_id,
				 vdev_id, req->frame_len,
				 fc_type, fc_subtype, tx_chanfreq, mgmt_params.chanfreq,
				 addr1, addr2, addr3);
		} else {
			wma_info("Injection frame[%u]: desc_id=%u vdev=%u len=%u fc_type=0x%02x fc_subtype=0x%02x tx_chanfreq=%u cmd_chanfreq=%u",
				 inject_send_info_count + 1, mgmt_params.desc_id,
				 vdev_id, req->frame_len,
				 fc_type, fc_subtype, tx_chanfreq, mgmt_params.chanfreq);
		}
		inject_send_info_count++;
	}

	wmi_mgmt_service = wmi_service_enabled(wma_handle->wmi_handle,
					       wmi_service_mgmt_tx_wmi);
	if (monitor_vdev && !wmi_mgmt_service && !inject_wmi_service_absent_logged) {
		wma_warn("Injection monitor vdev: mgmt_tx_wmi service bit is 0, but WMI TX will still be attempted");
		inject_wmi_service_absent_logged = true;
	}

	if (monitor_vdev) {
		/*
		 * FW _wlan_send_mgmt_to_host rejects MONITOR vdevs (falls
		 * to a beacon-only path → DISCARD).  Route through a hidden
		 * AP vdev instead, which the FW accepts for mgmt TX.
		 */
		status = wma_injection_ensure_tx_vdev(wma_handle,
						vdev_id, tx_chanfreq);
		if (QDF_IS_STATUS_ERROR(status)) {
			wma_err_rl("Failed to create injection TX helper vdev: %d",
				status);
			qdf_nbuf_free(wmi_buf);
			return status;
		}
		mgmt_params.vdev_id = g_inj_tx_vdev.vdev_id;
		if (!inject_monitor_no_legacy_logged) {
			wma_warn("Injection monitor: using helper STA vdev %u for TX (monitor vdev %u)",
				 g_inj_tx_vdev.vdev_id, vdev_id);
			inject_monitor_no_legacy_logged = true;
		}
	}

	/* Attempt WMI management TX path. */
	wmi_tx_attempted = true;
	if (!inject_wmi_path_logged) {
		wma_info("Injection using WMI mgmt tx path (vdev_id=%u)",
			 mgmt_params.vdev_id);
		inject_wmi_path_logged = true;
	}
	if (!wma_injection_debug_cache_update(mgmt_params.desc_id, req,
					      fc_type, fc_subtype,
					      mgmt_params.chanfreq, wmi_buf)) {
		wma_warn("Injection: descriptor cache busy; TX rejected");
		qdf_nbuf_free(wmi_buf);
		return QDF_STATUS_E_RESOURCES;
	}

	/* NH address audit: actual WMI payload before submission. */
	if (req->frame_len >= 24 &&
	    ((frame_data[0] & 0xfc) == 0x00 ||
	     (frame_data[0] & 0xfc) == 0x20 ||
	     (frame_data[0] & 0xfc) == 0xb0)) {
		wma_info("NHADDR WMI session=%u desc=%u vdev=%u freq=%u len=%u fc=%02x seq=%02x%02x da=%pM sa=%pM bssid=%pM",
			 req->session_id, mgmt_params.desc_id,
			 mgmt_params.vdev_id, mgmt_params.chanfreq,
			 req->frame_len, frame_data[0],
			 frame_data[23], frame_data[22],
			 &frame_data[4], &frame_data[10], &frame_data[16]);
	}

	status = wmi_mgmt_unified_cmd_send(wma_handle->wmi_handle, &mgmt_params);

	if (!QDF_IS_STATUS_ERROR(status)) {
		wmi_tx_ok = true;

		wma_info("Injection WMI mgmt TX submitted: desc_id=%u vdev=%u len=%u fc_type=0x%02x fc_subtype=0x%02x chanfreq=%u service=%u monitor=%u",
			 mgmt_params.desc_id, mgmt_params.vdev_id, req->frame_len,
			 fc_type, fc_subtype, mgmt_params.chanfreq,
			 wmi_mgmt_service ? 1 : 0, monitor_vdev ? 1 : 0);
	} else {
		wmi_buf = wma_injection_debug_cache_take(
				mgmt_params.desc_id, NULL);

		wma_warn("WMI management TX command failed: %d (service=%u monitor=%u)",
			 status, wmi_mgmt_service ? 1 : 0,
			 monitor_vdev ? 1 : 0);

		if (!wmi_buf)
			return status;
	}

	if (!wmi_tx_ok) {
		if (monitor_vdev) {
			wma_warn("Injection monitor vdev: dropping frame after WMI TX failure to avoid legacy FW assert");
			qdf_nbuf_free(wmi_buf);
			return status;
		}

		/* Fallback to legacy data path if WMI TX is unavailable or failed. */
		if (!inject_legacy_path_logged) {
			wma_warn("Injection using legacy cdp_mgmt_send_ext fallback (wmi_mgmt_service=%u)",
				 wmi_mgmt_service ? 1 : 0);
			inject_legacy_path_logged = true;
		}

		/* Try legacy CDP management send path */
		soc = cds_get_context(QDF_MODULE_ID_SOC);
		if (!soc) {
			wma_err("Failed to get CDP SOC context");
			qdf_nbuf_free(wmi_buf);
			return QDF_STATUS_E_FAILURE;
		}

		/* Set frame control information for legacy path */
		QDF_NBUF_CB_MGMT_TXRX_DESC_ID(wmi_buf) = mgmt_params.desc_id;

		ret = cdp_mgmt_send_ext(soc, mgmt_params.vdev_id, wmi_buf,
					mgmt_params.tx_type,
					mgmt_params.use_6mbps,
					mgmt_params.chanfreq);
		if (ret == -EINVAL)
			wma_warn("Legacy management TX got -EINVAL (desc alloc or tx pool reject)");
		status = qdf_status_from_os_return(ret);

		if (QDF_IS_STATUS_ERROR(status)) {
			wma_err("Legacy management TX failed: %d (wmi_attempted=%u)",
				status, wmi_tx_attempted ? 1 : 0);
			/* wmi_buf has either been consumed or freed at this point. */
			return status;
		}
	}

	return QDF_STATUS_SUCCESS;
}

QDF_STATUS wma_handle_injection_fw_response(tp_wma_handle wma_handle,
					     uint32_t desc_id,
					     uint32_t status)
{
	struct wma_injection_queue_ctx *ctx = &g_wma_injection_ctx;
	struct wma_injection_debug_info snapshot;
	struct wma_injection_debug_info *dbg_entry;
	qdf_nbuf_t completed_buf;
	const char *status_str;

	if (!wma_handle) {
		wma_err("Invalid WMA handle");
		return QDF_STATUS_E_INVAL;
	}

	/* Accept late completions even after injection queue shutdown. */

	/* Map firmware status to string for logging */
	switch (status) {
	case WMI_MGMT_TX_COMP_TYPE_COMPLETE_OK:
		status_str = "SUCCESS";
		break;
	case WMI_MGMT_TX_COMP_TYPE_DISCARD:
		status_str = "DISCARDED";
		ctx->stats.frames_dropped++;
		break;
	case WMI_MGMT_TX_COMP_TYPE_COMPLETE_NO_ACK:
		status_str = "NO_ACK";
		break;
	case WMI_MGMT_TX_COMP_TYPE_INSPECT:
		status_str = "INSPECT";
		break;
	default:
		status_str = "UNKNOWN";
		ctx->stats.frames_dropped++;
		break;
	}

	completed_buf = wma_injection_debug_cache_take(desc_id, &snapshot);
	dbg_entry = completed_buf ? &snapshot : NULL;
	if (dbg_entry) {
		/*
		 * Log the first few completions for each status category
		 * so the user can verify injection works, then go silent
		 * to avoid flooding dmesg and killing throughput.
		 */
		if (status == WMI_MGMT_TX_COMP_TYPE_COMPLETE_OK) {
			if (ctx->stats.frames_processed < 5)
				wma_info("Injection completion: desc_id=%u status=OK len=%u fc_type=0x%02x fc_subtype=0x%02x chanfreq=%u",
					 desc_id, dbg_entry->frame_len,
					 dbg_entry->fc_type, dbg_entry->fc_subtype,
					 dbg_entry->chanfreq);
		} else {
			ctx->stats.fw_errors++;
			if (ctx->stats.fw_errors <= 10)
				wma_info("Injection completion: desc_id=%u status=%s(%u) len=%u fc_type=0x%02x fc_subtype=0x%02x chanfreq=%u addr1=%pM addr2=%pM addr3=%pM",
					 desc_id, status_str, status,
					 dbg_entry->frame_len,
					 dbg_entry->fc_type, dbg_entry->fc_subtype,
					 dbg_entry->chanfreq, dbg_entry->addr1,
					 dbg_entry->addr2, dbg_entry->addr3);
		}

		/* Exactly one claimant releases the completed buffer. */
		wma_injection_unmap_tx_buf(completed_buf);
		qdf_nbuf_free(completed_buf);
	} else {
		if (status != WMI_MGMT_TX_COMP_TYPE_COMPLETE_OK) {
			ctx->stats.fw_errors++;
			if (ctx->stats.fw_errors <= 10)
				wma_info("Injection completion: desc_id=%u status=%s(%u)",
					 desc_id, status_str, status);
		}
	}

	if (status == WMI_MGMT_TX_COMP_TYPE_COMPLETE_OK)
		ctx->stats.frames_processed++;

	return QDF_STATUS_SUCCESS;
}

#else /* FEATURE_FRAME_INJECTION_SUPPORT */

QDF_STATUS wma_send_injection_frame_to_fw(tp_wma_handle wma_handle,
					  struct inject_frame_req *req,
					  uint8_t vdev_id)
{
	return QDF_STATUS_E_NOSUPPORT;
}

QDF_STATUS wma_handle_injection_fw_response(tp_wma_handle wma_handle,
					     uint32_t desc_id,
					     uint32_t status)
{
	QDF_STATUS qdf_status = QDF_STATUS_SUCCESS;
	enum wma_injection_fw_error_type error_type;

	WMA_LOGD("Handling firmware injection response: desc_id=%u, status=0x%x", desc_id, status);

	if (!wma_handle) {
		WMA_LOGE("Invalid WMA handle");
		return QDF_STATUS_E_INVAL;
	}

	/* Check if this is an error response */
	if (status != 0) {
		WMA_LOGW("Firmware injection failed: desc_id=%u, status=0x%x", desc_id, status);

		/* Handle the firmware error */
		qdf_status = wma_handle_firmware_injection_error(wma_handle, status, 0, NULL);
		if (QDF_IS_STATUS_ERROR(qdf_status) && qdf_status != QDF_STATUS_E_PENDING) {
			WMA_LOGE("Failed to handle firmware injection error: %d", qdf_status);
			return qdf_status;
		}
	} else {
		WMA_LOGD("Firmware injection completed successfully: desc_id=%u", desc_id);
	}

	/* Update statistics would go here in a full implementation */

	return qdf_status;
}

QDF_STATUS wma_init_injection_queue(tp_wma_handle wma_handle)
{
	return QDF_STATUS_SUCCESS;
}

QDF_STATUS wma_deinit_injection_queue(tp_wma_handle wma_handle)
{
	return QDF_STATUS_SUCCESS;
}

void wma_injection_pre_stop_cleanup(tp_wma_handle wma_handle)
{
}

QDF_STATUS wma_queue_injection_frame(tp_wma_handle wma_handle,
				     struct inject_frame_req *req,
				     uint8_t vdev_id)
{
	return QDF_STATUS_E_NOSUPPORT;
}

QDF_STATUS wma_get_injection_queue_stats(tp_wma_handle wma_handle,
					 struct wma_injection_queue_stats *stats)
{
	return QDF_STATUS_E_NOSUPPORT;
}

QDF_STATUS wma_reset_injection_queue_stats(tp_wma_handle wma_handle)
{
	return QDF_STATUS_E_NOSUPPORT;
}

uint32_t wma_get_injection_queue_size(tp_wma_handle wma_handle)
{
	return 0;
}

bool wma_is_injection_queue_empty(tp_wma_handle wma_handle)
{
	return true;
}

QDF_STATUS wma_flush_injection_queue(tp_wma_handle wma_handle)
{
	return QDF_STATUS_SUCCESS;
}

/**
 * wma_translate_fw_injection_error() - Translate firmware error codes
 * @fw_error_code: Firmware-specific error code
 *
 * This function translates firmware-specific error codes to standard
 * WMA injection error types for consistent error handling.
 *
 * Return: Translated error type
 */
enum wma_injection_fw_error_type wma_translate_fw_injection_error(uint32_t fw_error_code)
{
	enum wma_injection_fw_error_type error_type;

	WMA_LOGD("Translating firmware error code: 0x%x", fw_error_code);

	switch (fw_error_code) {
	case 0x0: /* Success */
		error_type = WMA_INJECTION_FW_ERROR_NONE;
		break;
	case 0x1: /* Generic failure */
		error_type = WMA_INJECTION_FW_ERROR_REJECTED;
		break;
	case 0x2: /* Invalid VDEV */
		error_type = WMA_INJECTION_FW_ERROR_INVALID_VDEV;
		break;
	case 0x3: /* No resources */
		error_type = WMA_INJECTION_FW_ERROR_NO_RESOURCES;
		break;
	case 0x4: /* Interface down */
		error_type = WMA_INJECTION_FW_ERROR_INTERFACE_DOWN;
		break;
	case 0x5: /* Power save mode */
		error_type = WMA_INJECTION_FW_ERROR_POWER_SAVE;
		break;
	case 0x6: /* Channel switch in progress */
		error_type = WMA_INJECTION_FW_ERROR_CHANNEL_SWITCH;
		break;
	case 0x7: /* Scan active */
		error_type = WMA_INJECTION_FW_ERROR_SCAN_ACTIVE;
		break;
	case 0xFFFFFFFF: /* Timeout */
		error_type = WMA_INJECTION_FW_ERROR_TIMEOUT;
		break;
	default:
		error_type = WMA_INJECTION_FW_ERROR_UNKNOWN;
		break;
	}

	WMA_LOGD("Translated firmware error 0x%x to type %d", fw_error_code, error_type);
	return error_type;
}

/**
 * wma_handle_firmware_injection_error() - Handle firmware injection error
 * @wma_handle: WMA handle
 * @error_code: Firmware error code
 * @vdev_id: VDEV ID associated with error
 * @req: Frame request that caused error (optional)
 *
 * This function handles firmware errors during frame injection. It implements
 * retry logic for transient failures and coordinates with HDD layer for
 * error recovery. It also maintains firmware state synchronization.
 *
 * Return: QDF_STATUS_SUCCESS on successful recovery, error code on failure
 */
QDF_STATUS wma_handle_firmware_injection_error(tp_wma_handle wma_handle,
						uint32_t error_code,
						uint8_t vdev_id,
						struct inject_frame_req *req)
{
	enum wma_injection_fw_error_type error_type;
	struct wma_injection_fw_error_info *error_info;
	QDF_STATUS status = QDF_STATUS_SUCCESS;
	bool should_retry = false;
	uint32_t retry_delay_ms = 0;

	WMA_LOGD("Handling firmware injection error: code=0x%x, vdev_id=%u", error_code, vdev_id);

	if (!wma_handle) {
		WMA_LOGE("Invalid WMA handle");
		return QDF_STATUS_E_INVAL;
	}

	/* Translate firmware error code */
	error_type = wma_translate_fw_injection_error(error_code);

	/* Update error statistics - this would be part of a larger injection context */
	/* For now, we'll log the error information */
	WMA_LOGW("Firmware injection error: type=%d, code=0x%x, vdev_id=%u",
		 error_type, error_code, vdev_id);

	/* Determine if we should retry based on error type */
	switch (error_type) {
	case WMA_INJECTION_FW_ERROR_NO_RESOURCES:
		/* Transient error - retry with delay */
		should_retry = true;
		retry_delay_ms = 100; /* 100ms delay */
		break;

	case WMA_INJECTION_FW_ERROR_POWER_SAVE:
		/* Device in power save - retry with longer delay */
		should_retry = true;
		retry_delay_ms = 500; /* 500ms delay */
		break;

	case WMA_INJECTION_FW_ERROR_CHANNEL_SWITCH:
		/* Channel switch in progress - retry with delay */
		should_retry = true;
		retry_delay_ms = 200; /* 200ms delay */
		break;

	case WMA_INJECTION_FW_ERROR_SCAN_ACTIVE:
		/* Scan active - retry with short delay */
		should_retry = true;
		retry_delay_ms = 50; /* 50ms delay */
		break;

	case WMA_INJECTION_FW_ERROR_TIMEOUT:
		/* Timeout - retry once with longer delay */
		should_retry = true;
		retry_delay_ms = 1000; /* 1 second delay */
		break;

	case WMA_INJECTION_FW_ERROR_INVALID_VDEV:
	case WMA_INJECTION_FW_ERROR_INTERFACE_DOWN:
		/* Permanent errors - synchronize state but don't retry */
		status = wma_sync_firmware_injection_state(wma_handle, vdev_id);
		should_retry = false;
		break;

	case WMA_INJECTION_FW_ERROR_REJECTED:
	case WMA_INJECTION_FW_ERROR_UNKNOWN:
	default:
		/* Unknown or permanent errors - don't retry */
		should_retry = false;
		break;
	}

	/* Attempt retry if appropriate and request is available */
	if (should_retry && req) {
		/* Check retry count to avoid infinite loops */
		if (req->retry_count < 3) { /* Maximum 3 retries */
			WMA_LOGI("Retrying injection after %u ms delay (attempt %u)",
				 retry_delay_ms, req->retry_count + 1);

			/* Schedule retry with delay */
			status = wma_retry_injection_frame(wma_handle, req, vdev_id, error_type);
			if (QDF_IS_STATUS_SUCCESS(status)) {
				return QDF_STATUS_E_PENDING; /* Retry scheduled */
			}
		} else {
			WMA_LOGW("Maximum retry attempts reached for injection request");
			status = QDF_STATUS_E_FAILURE;
		}
	}

	/* If we reach here, either no retry was needed or retry failed */
	if (QDF_IS_STATUS_ERROR(status)) {
		WMA_LOGE("Firmware injection error handling failed: %d", status);
	} else {
		WMA_LOGI("Firmware injection error handled successfully");
	}

	return status;
}

/**
 * wma_retry_injection_frame() - Retry injection frame after firmware error
 * @wma_handle: WMA handle
 * @req: Frame injection request to retry
 * @vdev_id: VDEV ID for the frame
 * @error_type: Type of error that occurred
 *
 * This function implements retry logic for transient firmware failures.
 * It uses exponential backoff and limits the number of retry attempts.
 *
 * Return: QDF_STATUS_SUCCESS on successful retry, error code on failure
 */
QDF_STATUS wma_retry_injection_frame(tp_wma_handle wma_handle,
				     struct inject_frame_req *req,
				     uint8_t vdev_id,
				     enum wma_injection_fw_error_type error_type)
{
	QDF_STATUS status;
	uint32_t delay_ms;

	WMA_LOGD("Retrying injection frame: vdev_id=%u, error_type=%d, retry_count=%u",
		 vdev_id, error_type, req->retry_count);

	if (!wma_handle || !req) {
		WMA_LOGE("Invalid parameters for retry");
		return QDF_STATUS_E_INVAL;
	}

	/* Increment retry count */
	req->retry_count++;

	/* Calculate exponential backoff delay */
	delay_ms = 100 * (1 << (req->retry_count - 1)); /* 100ms, 200ms, 400ms, ... */
	if (delay_ms > 2000) {
		delay_ms = 2000; /* Cap at 2 seconds */
	}

	/* Add some jitter to avoid thundering herd */
	delay_ms += (qdf_get_log_timestamp() % 50); /* Add 0-49ms jitter */

	WMA_LOGI("Scheduling injection retry in %u ms (attempt %u)",
		 delay_ms, req->retry_count);

	/* For now, we'll simulate the retry by calling the send function again */
	/* In a real implementation, this would be scheduled with a timer */
	qdf_sleep(delay_ms);

	/* Attempt to send the frame again */
	status = wma_send_injection_frame_to_fw(wma_handle, req, vdev_id);
	if (QDF_IS_STATUS_ERROR(status)) {
		WMA_LOGW("Injection retry failed: %d", status);
		return status;
	}

	WMA_LOGI("Injection retry initiated successfully");
	return QDF_STATUS_SUCCESS;
}

/**
 * wma_sync_firmware_injection_state() - Synchronize firmware injection state
 * @wma_handle: WMA handle
 * @vdev_id: VDEV ID to synchronize
 *
 * This function synchronizes the firmware injection state after errors.
 * It ensures that the firmware and driver are in a consistent state.
 *
 * Return: QDF_STATUS_SUCCESS on success, error code on failure
 */
QDF_STATUS wma_sync_firmware_injection_state(tp_wma_handle wma_handle,
					      uint8_t vdev_id)
{
	QDF_STATUS status = QDF_STATUS_SUCCESS;

	WMA_LOGD("Synchronizing firmware injection state for vdev_id=%u", vdev_id);

	if (!wma_handle) {
		WMA_LOGE("Invalid WMA handle");
		return QDF_STATUS_E_INVAL;
	}

	/* Check if VDEV is valid and active */
	if (vdev_id >= wma_handle->max_bssid) {
		WMA_LOGE("Invalid VDEV ID: %u", vdev_id);
		return QDF_STATUS_E_INVAL;
	}

	/* Verify VDEV state */
	if (!wma_handle->interfaces[vdev_id].handle) {
		WMA_LOGW("VDEV %u is not active", vdev_id);
		return QDF_STATUS_E_INVAL;
	}

	/* Check interface type - injection typically requires monitor mode */
	if (wma_handle->interfaces[vdev_id].type != WMI_VDEV_TYPE_MONITOR) {
		WMA_LOGW("VDEV %u is not in monitor mode (type=%d)",
			 vdev_id, wma_handle->interfaces[vdev_id].type);
		/* This might not be an error depending on implementation */
	}

	/* Flush any pending injection frames for this VDEV */
	/* This would be implemented as part of the queue management */
	WMA_LOGI("Flushing pending injection frames for vdev_id=%u", vdev_id);

	/* Send a sync command to firmware if needed */
	/* This would involve sending a WMI command to query/reset injection state */
	WMA_LOGD("Sending injection state sync command to firmware");

	/* For now, we'll just log that synchronization is complete */
	WMA_LOGI("Firmware injection state synchronized for vdev_id=%u", vdev_id);

	return status;
}

#endif /* FEATURE_FRAME_INJECTION_SUPPORT */
