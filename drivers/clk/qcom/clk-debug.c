// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2016, 2019-2021 The Linux Foundation. All rights reserved. */
/* Copyright (c) 2023, Qualcomm Innovation Center, Inc. All rights reserved. */

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/lockdep.h>
#include <linux/overflow.h>
#include <linux/clk.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/clk-provider.h>
#include <linux/of.h>
#include <linux/bitops.h>
#include <linux/mfd/syscon.h>
#include <linux/msm-bus.h>
#include <linux/interconnect.h>
#include <trace/events/power.h>

#define CREATE_TRACE_POINTS
#include "trace.h"

#include "clk-regmap.h"
#include "clk-debug.h"
#include "common.h"
#include "gdsc-debug.h"

static struct clk_hw *measure;

static DEFINE_MUTEX(clk_debug_lock);
static bool clk_debug_measure_faulted;

#define TCXO_DIV_4_HZ		4800000
#define SAMPLE_TICKS_1_MS	0x1000
#define SAMPLE_TICKS_14_MS	0x10000

#define XO_DIV4_CNT_DONE	BIT(25)
#define CNT_EN			BIT(20)
#define MEASURE_CNT		GENMASK(24, 0)
#define CBCR_ENA		BIT(0)

/* Sample clock for 'ticks' reference clock ticks. */
static int clk_debug_wait_done(struct regmap *regmap,
			       unsigned int status_reg,
			       bool done,
			       unsigned int timeout_us,
			       unsigned int *regval)
{
	ktime_t deadline;
	int ret;

	lockdep_assert_held(&clk_debug_lock);

	deadline = ktime_add_us(ktime_get(), timeout_us);

	for (;;) {
		ret = regmap_read(regmap, status_reg, regval);
		if (ret)
			return ret;

		if (!!(*regval & XO_DIV4_CNT_DONE) == done)
			return 0;

		if (ktime_compare(ktime_get(), deadline) >= 0)
			return -ETIMEDOUT;

		usleep_range(50, 100);
	}
}

static int run_measurement(unsigned int ticks,
			   struct regmap *regmap,
			   unsigned int ctl_reg,
			   unsigned int status_reg,
			   u64 *count)
{
	unsigned int regval;
	int ret, stop_ret;

	ret = regmap_write(regmap, ctl_reg, ticks);
	if (ret)
		goto stop;

	ret = clk_debug_wait_done(regmap, status_reg,
				  false, 2000, &regval);
	if (ret)
		goto stop;

	ret = regmap_write(regmap, ctl_reg, CNT_EN | ticks);
	if (ret)
		goto stop;

	ret = clk_debug_wait_done(regmap, status_reg,
				  true, 30000, &regval);
	if (ret)
		goto stop;

	*count = regval & MEASURE_CNT;

stop:
	stop_ret = regmap_write(regmap, ctl_reg, ticks);
	return ret ? ret : stop_ret;
}

/*
 * Perform a hardware rate measurement for a given clock.
 * FOR DEBUG USE ONLY: Measurements take ~15 ms!
 */
static int clk_debug_mux_measure_rate(struct clk_hw *hw, u64 *rate)
{
	u32 saved_xo4_reg, saved_ctl_reg, readback;
	u64 raw_count_short, raw_count_full;
	u64 measured_rate = 0;
	int err, cleanup_ret;
	struct clk_debug_mux *meas = to_clk_measure(hw);
	struct measure_clk_data *data = meas->priv;

	lockdep_assert_held(&clk_debug_lock);

	*rate = 0;

	if (READ_ONCE(clk_debug_measure_faulted))
		return -EIO;

	if (!data || IS_ERR_OR_NULL(data->cxo) ||
	    IS_ERR_OR_NULL(meas->regmap))
		return -EINVAL;

	err = clk_prepare_enable(data->cxo);
	if (err)
		return err;

	/* Do not disturb an already-active measurement. */
	err = regmap_read(meas->regmap, data->ctl_reg,
			  &saved_ctl_reg);
	if (err)
		goto disable_xo;

	if (saved_ctl_reg & CNT_EN) {
		err = -EBUSY;
		goto disable_xo;
	}

	err = regmap_read(meas->regmap, data->xo_div4_cbcr,
			  &saved_xo4_reg);
	if (err)
		goto disable_xo;

	/* Change only the branch enable bit. */
	err = regmap_update_bits(meas->regmap,
				 data->xo_div4_cbcr,
				 CBCR_ENA, CBCR_ENA);
	if (err)
		goto restore_xo4;

	err = run_measurement(SAMPLE_TICKS_1_MS,
			      meas->regmap,
			      data->ctl_reg,
			      data->status_reg,
			      &raw_count_short);
	if (err)
		goto restore_counter;

	err = run_measurement(SAMPLE_TICKS_14_MS,
			      meas->regmap,
			      data->ctl_reg,
			      data->status_reg,
			      &raw_count_full);
	if (err)
		goto restore_counter;

	if (raw_count_full != raw_count_short) {
		raw_count_full =
			((raw_count_full * 10) + 15) * TCXO_DIV_4_HZ;
		do_div(raw_count_full,
		       ((SAMPLE_TICKS_14_MS * 10) + 35));
		measured_rate = raw_count_full;
	}

restore_counter:
	/*
	 * Stop our measurement before restoring the original
	 * inactive counter configuration.
	 */
	cleanup_ret = regmap_update_bits(meas->regmap,
					 data->ctl_reg, CNT_EN, 0);
	if (!cleanup_ret)
		cleanup_ret = regmap_write(meas->regmap,
					   data->ctl_reg,
					   saved_ctl_reg);

	/*
	 * Verify the documented counter enable bit.
	 * Do not assume undocumented control bits are stable.
	 */
	if (!cleanup_ret) {
		cleanup_ret = regmap_read(meas->regmap,
					  data->ctl_reg, &readback);
		if (!cleanup_ret && (readback & CNT_EN))
			cleanup_ret = -EIO;
	}

	if (cleanup_ret) {
		WRITE_ONCE(clk_debug_measure_faulted, true);
		pr_err_ratelimited(
			"debugcc: counter restore failed: %d\n",
			cleanup_ret);
		if (!err)
			err = cleanup_ret;
	}

restore_xo4:
	cleanup_ret = regmap_update_bits(meas->regmap,
					 data->xo_div4_cbcr,
					 CBCR_ENA,
					 saved_xo4_reg & CBCR_ENA);

	if (!cleanup_ret) {
		cleanup_ret = regmap_read(meas->regmap,
					  data->xo_div4_cbcr,
					  &readback);
		if (!cleanup_ret &&
		    (readback & CBCR_ENA) !=
		    (saved_xo4_reg & CBCR_ENA))
			cleanup_ret = -EIO;
	}

	if (cleanup_ret) {
		WRITE_ONCE(clk_debug_measure_faulted, true);
		pr_err_ratelimited(
			"debugcc: XO/4 restore failed: %d\n",
			cleanup_ret);
		if (!err)
			err = cleanup_ret;
	}

disable_xo:
	clk_disable_unprepare(data->cxo);

	if (err) {
		pr_err_ratelimited(
			"debugcc: measurement failed: %d\n", err);
		return err;
	}

	*rate = measured_rate;
	return 0;
}


static struct icc_path *
clk_debug_get_icc_path(struct clk_debug_mux *meas, u32 i)
{
	return meas->num_icc_paths ?
		meas->icc_paths[i] : meas->icc_path;
}

static int clk_debug_release_votes(struct clk_debug_mux *meas)
{
	int i, ret, first_error = 0;

	for (i = CLK_DEBUG_MAX_ICC_PATHS - 1; i >= 0; i--) {
		if (!(meas->icc_active_mask & BIT(i)))
			continue;

		ret = icc_set_bw(clk_debug_get_icc_path(meas, i),
				 0, 0);
		if (!ret)
			meas->icc_active_mask &= ~BIT(i);
		else if (!first_error)
			first_error = ret;
	}

	if (meas->legacy_vote_active) {
		ret = msm_bus_scale_client_update_request(
				meas->bus_cl_id, 0);
		if (!ret)
			meas->legacy_vote_active = false;
		else if (!first_error)
			first_error = ret;
	}

	if (first_error)
		pr_err_ratelimited("debugcc: vote release failed: %d\n",
				   first_error);

	return first_error;
}

/* Caller serializes the complete debug measurement transaction. */
static int clk_debug_icc_bus_vote(struct clk_debug_mux *meas, bool enable)
{
	u32 i, count;
	int ret, cleanup_ret;

	if (!enable)
		return clk_debug_release_votes(meas);

	count = meas->num_icc_paths;
	if (count > CLK_DEBUG_MAX_ICC_PATHS)
		return -EINVAL;

	if (!count && meas->icc_path)
		count = 1;

	/* Retry cleanup from a previous failed transaction. */
	ret = clk_debug_release_votes(meas);
	if (ret)
		return ret;

	for (i = 0; i < count; i++) {
		struct icc_path *path =
			clk_debug_get_icc_path(meas, i);

		if (IS_ERR_OR_NULL(path)) {
			ret = -EINVAL;
			goto rollback;
		}

		ret = icc_set_bw(path, 0, 1);
		if (ret)
			goto rollback;

		meas->icc_active_mask |= BIT(i);
	}

	if (count)
		return 0;

	goto legacy;

rollback:
	cleanup_ret = clk_debug_release_votes(meas);
	if (cleanup_ret)
		return cleanup_ret;

	if (ret != -EAGAIN || !meas->bus_cl_id)
		return ret;

legacy:
	if (!meas->bus_cl_id)
		return 0;

	ret = msm_bus_scale_client_update_request(
			meas->bus_cl_id, 1);
	if (!ret)
		meas->legacy_vote_active = true;

	return ret;
}

#define CLK_DEBUG_MAX_ROUTE_DEPTH 16

struct clk_debug_route_entry {
	struct clk_hw *mux;
	struct clk_hw *parent;
	struct clk_hw *saved_parent;
	unsigned int parent_index;
	u32 saved_src;
	u32 saved_div;
	bool saved;
	bool attempted;
};

struct clk_debug_route {
	unsigned int count;
	struct clk_debug_route_entry
		entries[CLK_DEBUG_MAX_ROUTE_DEPTH];
};

/*
 * Discover a valid route without programming hardware.
 * Entries are stored from the root mux towards the target.
 */
static int clk_debug_find_route(struct clk_hw *mux,
				struct clk_hw *target,
				struct clk_debug_route *route,
				unsigned int depth)
{
	struct clk_hw *parent;
	unsigned int i, j, num_parents;
	int ret;

	if (!mux || !target || !mux->init ||
	    !(mux->init->flags & CLK_IS_MEASURE))
		return -ENOENT;

	if (depth >= CLK_DEBUG_MAX_ROUTE_DEPTH)
		return -ELOOP;

	for (j = 0; j < depth; j++) {
		if (route->entries[j].mux == mux)
			return -ELOOP;
	}

	num_parents = clk_hw_get_num_parents(mux);

	/* Prefer a direct connection to the target. */
	for (i = 0; i < num_parents; i++) {
		parent = clk_hw_get_parent_by_index(mux, i);
		if (parent != target)
			continue;

		route->entries[depth].mux = mux;
		route->entries[depth].parent = parent;
		route->entries[depth].parent_index = i;
		route->count = depth + 1;
		return 0;
	}

	/* Search upstream measurement muxes. */
	for (i = 0; i < num_parents; i++) {
		parent = clk_hw_get_parent_by_index(mux, i);
		if (!parent)
			continue;

		route->entries[depth].mux = mux;
		route->entries[depth].parent = parent;
		route->entries[depth].parent_index = i;

		ret = clk_debug_find_route(parent, target,
					   route, depth + 1);
		if (!ret)
			return 0;
	}

	return -ENOENT;
}

/*
 * Apply only the selected route, starting with the
 * deepest upstream mux.
 */

/*
 * Caller must hold the DebugCC transaction mutex and
 * protect the CCF topology for the entire transaction.
 *
 * Snapshot every route entry before programming any mux.
 */
static int
clk_debug_snapshot_route(struct clk_debug_route *route)
{
	struct clk_debug_route_entry *entry;
	struct clk_debug_mux *mux;
	unsigned int i;
	int ret;

	lockdep_assert_held(&clk_debug_lock);

	if (!route->count ||
	    route->count > CLK_DEBUG_MAX_ROUTE_DEPTH)
		return -EINVAL;

	for (i = 0; i < route->count; i++) {
		entry = &route->entries[i];
		entry->saved = false;
		entry->attempted = false;

		if (!entry->mux || !entry->parent)
			return -EINVAL;

		entry->saved_parent =
			clk_hw_get_parent(entry->mux);

		/*
		 * Do not modify orphaned muxes until their
		 * original topology can be restored safely.
		 */
		if (!entry->saved_parent)
			return -ENODEV;

		mux = to_clk_measure(entry->mux);

		if (!mux->mux_sels) {
			entry->saved = true;
			continue;
		}

		if (IS_ERR_OR_NULL(mux->regmap))
			return -EINVAL;

		ret = regmap_read(mux->regmap,
				  mux->debug_offset,
				  &entry->saved_src);
		if (ret)
			return ret;

		if (mux->debug_offset == mux->post_div_offset) {
			entry->saved_div = entry->saved_src;
		} else {
			ret = regmap_read(mux->regmap,
					  mux->post_div_offset,
					  &entry->saved_div);
			if (ret)
				return ret;
		}

		entry->saved_src &= mux->src_sel_mask;
		entry->saved_div &= mux->post_div_mask;
		entry->saved = true;
	}

	return 0;
}

/*
 * Restore exact hardware fields after restoring the
 * corresponding CCF parent.
 */
static int clk_debug_restore_route_entry(
			struct clk_debug_route_entry *entry)
{
	struct clk_debug_mux *mux;
	u32 mask, readback;
	int ret, err = 0;

	if (!entry->saved || !entry->attempted)
		return 0;

	mux = to_clk_measure(entry->mux);

	if (clk_hw_get_parent(entry->mux) !=
	    entry->saved_parent) {
		ret = clk_set_parent(entry->mux->clk,
				     entry->saved_parent->clk);
		if (ret)
			err = ret < 0 ? ret : -EIO;
	}

	if (clk_hw_get_parent(entry->mux) !=
	    entry->saved_parent && !err)
		err = -EIO;

	if (!mux->mux_sels)
		return err;

	if (mux->debug_offset == mux->post_div_offset) {
		mask = mux->src_sel_mask | mux->post_div_mask;

		ret = regmap_update_bits(mux->regmap,
					 mux->debug_offset,
					 mask,
					 entry->saved_src |
					 entry->saved_div);
		if (ret && !err)
			err = ret;
	} else {
		ret = regmap_update_bits(mux->regmap,
					 mux->post_div_offset,
					 mux->post_div_mask,
					 entry->saved_div);
		if (ret && !err)
			err = ret;

		ret = regmap_update_bits(mux->regmap,
					 mux->debug_offset,
					 mux->src_sel_mask,
					 entry->saved_src);
		if (ret && !err)
			err = ret;
	}

	/*
	 * Verify the restored hardware fields rather
	 * than relying solely on successful writes.
	 */
	ret = regmap_read(mux->regmap,
			  mux->debug_offset,
			  &readback);
	if (ret) {
		if (!err)
			err = ret;
	} else if ((readback & mux->src_sel_mask) !=
		   entry->saved_src && !err) {
		err = -EIO;
	}

	ret = regmap_read(mux->regmap,
			  mux->post_div_offset,
			  &readback);
	if (ret) {
		if (!err)
			err = ret;
	} else if ((readback & mux->post_div_mask) !=
		   entry->saved_div && !err) {
		err = -EIO;
	}

	if (err)
		pr_err_ratelimited(
			"debugcc: mux %s restoration verification failed: %d\n",
			clk_hw_get_name(entry->mux), err);

	return err;
}

/*
 * Application runs leaf-to-root.
 * Restoration runs root-to-leaf.
 *
 * Attempt all restorations even if one fails.
 */
static int
clk_debug_restore_route(struct clk_debug_route *route)
{
	unsigned int i;
	int ret, err = 0;

	lockdep_assert_held(&clk_debug_lock);

	for (i = 0; i < route->count; i++) {
		ret = clk_debug_restore_route_entry(
					&route->entries[i]);
		if (ret && !err)
			err = ret;
	}

	if (err) {
		WRITE_ONCE(clk_debug_measure_faulted, true);
		pr_err_ratelimited(
			"debugcc: route restoration failed: %d\n",
			err);
	}

	return err;
}

static int clk_debug_apply_route(struct clk_debug_route *route)
{
	struct clk_debug_route_entry *entry;
	unsigned int i;
	int ret;

	lockdep_assert_held(&clk_debug_lock);

	for (i = route->count; i > 0; i--) {
		entry = &route->entries[i - 1];

		if (READ_ONCE(clk_debug_measure_faulted))
			return -EIO;

		entry->attempted = true;
		ret = clk_set_parent(entry->mux->clk,
				     entry->parent->clk);
		if (ret)
			return ret < 0 ? ret : -EIO;
	}

	return 0;
}

static int clk_debug_begin_route(struct clk_hw *mux,
				 struct clk_hw *target,
				 struct clk_debug_route *route)
{
	int ret;

	lockdep_assert_held(&clk_debug_lock);

	ret = clk_debug_find_route(mux, target, route, 0);
	if (ret)
		return ret;

	ret = clk_debug_snapshot_route(route);
	if (ret)
		return ret;

	return clk_debug_apply_route(route);
}

static u8 clk_debug_mux_get_parent(struct clk_hw *hw)
{
	int i, num_parents = clk_hw_get_num_parents(hw);
	struct clk_hw *hw_clk = clk_hw_get_parent(hw);

	if (!hw_clk)
		return 0;
	for (i = 0; i < num_parents; i++) {
		if (!strcmp(hw->init->parent_names[i],
					clk_hw_get_name(hw_clk))) {
			pr_debug("%s: clock parent - %s, index %d\n", __func__,
					hw->init->parent_names[i], i);
			return i;
		}
	}
	return 0;
}

static int clk_debug_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct clk_debug_mux *mux = to_clk_measure(hw);
	u32 saved_src, saved_div;
	u32 src_val, div_val;
	u32 mask;
	int ret, restore_ret;
	int rollback_error = 0;

	if (index >= clk_hw_get_num_parents(hw))
		return -EINVAL;

	/* Period-based measurement muxes have no selector. */
	if (!mux->mux_sels)
		return 0;

	if (IS_ERR_OR_NULL(mux->regmap))
		return -EINVAL;

	if (!mux->post_div_val ||
	    mux->src_sel_shift >= 32 ||
	    mux->post_div_shift >= 32)
		return -EINVAL;

	if (mux->mux_sels[index] < 0 ||
	    (u32)mux->mux_sels[index] >
		(mux->src_sel_mask >> mux->src_sel_shift) ||
	    mux->post_div_val - 1 >
		(mux->post_div_mask >> mux->post_div_shift))
		return -EINVAL;

	src_val = (u32)mux->mux_sels[index] <<
		  mux->src_sel_shift;
	div_val = (mux->post_div_val - 1) <<
		  mux->post_div_shift;

	if ((src_val & ~mux->src_sel_mask) ||
	    (div_val & ~mux->post_div_mask))
		return -EINVAL;

	/*
	 * Some controllers place both fields in the same
	 * register. Update them together.
	 */
	if (mux->debug_offset == mux->post_div_offset) {
		if (mux->src_sel_mask & mux->post_div_mask)
			return -EINVAL;

		mask = mux->src_sel_mask | mux->post_div_mask;

		ret = regmap_read(mux->regmap,
				  mux->debug_offset,
				  &saved_src);
		if (ret)
			return ret;

		ret = regmap_update_bits(mux->regmap,
					 mux->debug_offset,
					 mask,
					 src_val | div_val);
		if (!ret)
			return 0;

		rollback_error = regmap_update_bits(
					mux->regmap,
					mux->debug_offset,
					mask,
					saved_src & mask);
		goto rollback_done;
	}

	/* Snapshot both fields before modifying either. */
	ret = regmap_read(mux->regmap,
			  mux->debug_offset,
			  &saved_src);
	if (ret)
		return ret;

	ret = regmap_read(mux->regmap,
			  mux->post_div_offset,
			  &saved_div);
	if (ret)
		return ret;

	ret = regmap_update_bits(mux->regmap,
				 mux->debug_offset,
				 mux->src_sel_mask,
				 src_val);
	if (ret)
		goto restore_source;

	ret = regmap_update_bits(mux->regmap,
				 mux->post_div_offset,
				 mux->post_div_mask,
				 div_val);
	if (!ret)
		return 0;

	/* Restore divider first, then source selector. */
	restore_ret = regmap_update_bits(
				mux->regmap,
				mux->post_div_offset,
				mux->post_div_mask,
				saved_div & mux->post_div_mask);
	if (restore_ret)
		rollback_error = restore_ret;

restore_source:
	restore_ret = regmap_update_bits(
				mux->regmap,
				mux->debug_offset,
				mux->src_sel_mask,
				saved_src & mux->src_sel_mask);
	if (restore_ret && !rollback_error)
		rollback_error = restore_ret;

rollback_done:
	if (rollback_error) {
		WRITE_ONCE(clk_debug_measure_faulted, true);
		pr_err_ratelimited(
			"debugcc: mux %s rollback failed: %d\n",
			clk_hw_get_name(hw), rollback_error);
	}

	return ret;
}

const struct clk_ops clk_debug_mux_ops = {
	.get_parent = clk_debug_mux_get_parent,
	.set_parent = clk_debug_mux_set_parent,
};
EXPORT_SYMBOL(clk_debug_mux_ops);

#define CLK_DEBUG_MAX_BRANCH_DEPTH 16

struct clk_debug_branch_state {
	struct regmap *regmap;
	u32 offset;
	u32 mask;
	u32 saved;
};

struct clk_debug_branch_txn {
	unsigned int count;
	struct clk_debug_branch_state
		states[CLK_DEBUG_MAX_BRANCH_DEPTH];
};

static int enable_debug_clks(struct clk_hw *mux,
			     struct clk_debug_branch_txn *txn,
			     unsigned int depth)
{
	struct clk_debug_mux *meas;
	struct clk_debug_branch_state *state;
	struct clk_hw *parent;
	u32 mask, saved;
	int ret;

	lockdep_assert_held(&clk_debug_lock);

	if (!mux)
		return 0;

	if (!mux->init)
		return -EINVAL;

	if (!(mux->init->flags & CLK_IS_MEASURE))
		return 0;

	if (depth >= CLK_DEBUG_MAX_BRANCH_DEPTH)
		return -ELOOP;

	meas = to_clk_measure(mux);
	parent = clk_hw_get_parent(mux);

	/* Enable the selected upstream branch first. */
	ret = enable_debug_clks(parent, txn, depth + 1);
	if (ret)
		return ret;

	/* Not all debug muxes have a branch register. */
	if (meas->cbcr_offset == U32_MAX)
		return 0;

	if (IS_ERR_OR_NULL(meas->regmap))
		return -EINVAL;

	mask = meas->en_mask ? meas->en_mask : CBCR_ENA;

	ret = regmap_read(meas->regmap,
			  meas->cbcr_offset, &saved);
	if (ret)
		return ret;

	/* Do not take ownership of an already-enabled branch. */
	if ((saved & mask) == mask)
		return 0;

	if (txn->count >= CLK_DEBUG_MAX_BRANCH_DEPTH)
		return -EOVERFLOW;

	/*
	 * Record state before attempting the write so that
	 * partial failures can also be rolled back.
	 */
	state = &txn->states[txn->count++];
	state->regmap = meas->regmap;
	state->offset = meas->cbcr_offset;
	state->mask = mask;
	state->saved = saved & mask;

	return regmap_update_bits(state->regmap,
				  state->offset,
				  state->mask,
				  state->mask);
}

static int restore_debug_clks(struct clk_debug_branch_txn *txn)
{
	struct clk_debug_branch_state *state;
	u32 readback;
	int ret, first_error = 0;

	lockdep_assert_held(&clk_debug_lock);

	while (txn->count) {
		state = &txn->states[--txn->count];

		ret = regmap_update_bits(state->regmap,
					 state->offset,
					 state->mask,
					 state->saved);

		if (!ret) {
			ret = regmap_read(state->regmap,
					  state->offset, &readback);
			if (!ret &&
			    (readback & state->mask) != state->saved)
				ret = -EIO;
		}

		if (ret) {
			if (!first_error)
				first_error = ret;

			WRITE_ONCE(clk_debug_measure_faulted, true);

			pr_err_ratelimited(
				"debugcc: branch restore failed at %#x: %d\n",
				state->offset, ret);
		}
	}

	return first_error;
}

static int clk_debug_get_route_divs(
			struct clk_debug_route *route, u64 *result)
{
	struct clk_debug_route_entry *entry;
	struct clk_debug_mux *mux;
	u64 div = 1;
	unsigned int i, index;
	int pre_div;

	lockdep_assert_held(&clk_debug_lock);

	if (!route->count ||
	    route->count > CLK_DEBUG_MAX_ROUTE_DEPTH)
		return -EINVAL;

	for (i = 0; i < route->count; i++) {
		entry = &route->entries[i];
		mux = to_clk_measure(entry->mux);
		index = entry->parent_index;

		if (clk_hw_get_parent(entry->mux) != entry->parent)
			return -EIO;

		if (index >= clk_hw_get_num_parents(entry->mux))
			return -EINVAL;

		if (!mux->post_div_val)
			return -EINVAL;

		if (check_mul_overflow(div,
				(u64)mux->post_div_val, &div))
			return -EOVERFLOW;

		if (mux->pre_div_vals) {
			pre_div = mux->pre_div_vals[index];

			if (pre_div <= 0)
				return -EINVAL;

			if (check_mul_overflow(div,
					(u64)pre_div, &div))
				return -EOVERFLOW;
		}
	}

	*result = div;
	return 0;
}

static int clk_debug_measure_get(void *data, u64 *val)
{
	struct clk_hw *hw = data;
	struct clk_debug_mux *meas;
	struct clk_debug_branch_txn txn = { 0 };
	struct clk_debug_route route = { 0 };
	int ret = 0, restore_ret;
	u64 divisor;

	*val = 0;

	mutex_lock(&clk_debug_lock);

	if (READ_ONCE(clk_debug_measure_faulted)) {
		mutex_unlock(&clk_debug_lock);
		return -EIO;
	}

	if (IS_ERR_OR_NULL(measure)) {
		mutex_unlock(&clk_debug_lock);
		return -ENODEV;
	}

	meas = to_clk_measure(measure);

	/*
	 * Vote for bandwidth to re-connect config ports
	 * to multimedia clock controllers.
	 */
	ret = clk_debug_icc_bus_vote(meas, true);
	if (ret) {
		mutex_unlock(&clk_debug_lock);
		return ret;
	}

	clk_hw_debug_transaction_begin();

	ret = clk_debug_begin_route(measure, hw, &route);
	if (ret) {
		pr_err("Failed to set the debug mux's parent.\n");
		goto exit;
	}

	ret = enable_debug_clks(measure, &txn, 0);
	if (ret)
		goto restore_branches;

	ret = clk_debug_mux_measure_rate(measure, val);
	if (ret)
		goto restore_branches;

	ret = clk_debug_get_route_divs(&route, &divisor);
	if (ret)
		goto restore_branches;

	if (check_mul_overflow(*val, divisor, val)) {
		*val = 0;
		ret = -EOVERFLOW;
	}

restore_branches:
	restore_ret = restore_debug_clks(&txn);
	if (!ret)
		ret = restore_ret;

exit:
	restore_ret = clk_debug_restore_route(&route);
	if (!ret)
		ret = restore_ret;

	if (!ret)
		trace_clk_measure(clk_hw_get_name(hw), *val);

	clk_hw_debug_transaction_end();
	{
		int release_ret;

		/*
		 * Keep configuration ports accessible if hardware
		 * restoration failed. Further measurements are blocked.
		 */
		if (READ_ONCE(clk_debug_measure_faulted)) {
			pr_err_ratelimited(
				"debugcc: retaining votes after hardware fault\n");
			if (!ret)
				ret = -EIO;
		} else {
			release_ret = clk_debug_icc_bus_vote(meas, false);
			if (!ret)
				ret = release_ret;
		}
	}
	mutex_unlock(&clk_debug_lock);
	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(clk_measure_fops, clk_debug_measure_get,
							NULL, "%lld\n");

static int clk_debug_read_period(void *data, u64 *val)
{
	struct clk_hw *hw = data;
	struct clk_hw *parent;
	struct clk_debug_mux *meas;
	struct clk_debug_mux *mux;
	struct clk_debug_route route = { 0 };
	int ret, release_ret, restore_ret;
	u32 regval;

	*val = 0;

	mutex_lock(&clk_debug_lock);

	if (READ_ONCE(clk_debug_measure_faulted)) {
		mutex_unlock(&clk_debug_lock);
		return -EIO;
	}

	if (IS_ERR_OR_NULL(measure)) {
		ret = -ENODEV;
		goto unlock;
	}

	meas = to_clk_measure(measure);

	ret = clk_debug_icc_bus_vote(meas, true);
	if (ret)
		goto unlock;

	clk_hw_debug_transaction_begin();

	ret = clk_debug_begin_route(measure, hw, &route);
	if (ret)
		goto release;

	parent = clk_hw_get_parent(measure);
	if (!parent || !parent->init ||
	    !(parent->init->flags & CLK_IS_MEASURE)) {
		ret = -EINVAL;
		goto release;
	}

	mux = to_clk_measure(parent);

	if (IS_ERR_OR_NULL(mux->regmap) || mux->mux_sels) {
		ret = -EINVAL;
		goto release;
	}

	ret = regmap_read(mux->regmap, mux->period_offset,
			  &regval);
	if (ret)
		goto release;

	if (!regval) {
		ret = -EIO;
		goto release;
	}

	*val = 1000000000000ULL;
	do_div(*val, regval);

release:
	restore_ret = clk_debug_restore_route(&route);
	if (!ret)
		ret = restore_ret;

	clk_hw_debug_transaction_end();

	/*
	 * Match the counter measurement failure policy.
	 * Preserve configuration access after failed
	 * hardware restoration and reject further reads.
	 */
	if (READ_ONCE(clk_debug_measure_faulted)) {
		pr_err_ratelimited(
			"debugcc: retaining period votes after hardware fault\n");
		if (!ret)
			ret = -EIO;
	} else {
		release_ret = clk_debug_icc_bus_vote(meas, false);
		if (!ret)
			ret = release_ret;
	}

unlock:
	mutex_unlock(&clk_debug_lock);
	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(clk_read_period_fops, clk_debug_read_period,
							NULL, "%lld\n");

void clk_debug_measure_add(struct clk_hw *hw, struct dentry *dentry)
{
	struct clk_debug_route route = { 0 };
	struct clk_hw *parent;
	struct clk_debug_mux *meas_parent = NULL;
	int ret;

	mutex_lock(&clk_debug_lock);
	clk_hw_debug_transaction_begin();

	if (READ_ONCE(clk_debug_measure_faulted))
		goto unlock;

	if (IS_ERR_OR_NULL(measure))
		goto unlock;

	ret = clk_debug_find_route(measure, hw, &route, 0);
	if (ret || !route.count)
		goto unlock;

	parent = route.entries[0].parent;

	if (!parent || !parent->init)
		goto unlock;

	if (parent->init->flags & CLK_IS_MEASURE)
		meas_parent = to_clk_measure(parent);

	if (meas_parent && !meas_parent->mux_sels)
		debugfs_create_file("clk_measure", 0444,
				    dentry, hw,
				    &clk_read_period_fops);
	else
		debugfs_create_file("clk_measure", 0444,
				    dentry, hw,
				    &clk_measure_fops);

unlock:
	clk_hw_debug_transaction_end();
	mutex_unlock(&clk_debug_lock);
}
EXPORT_SYMBOL(clk_debug_measure_add);

int clk_debug_measure_register(struct clk_hw *hw)
{
	int ret = 0;

	if (!hw || !hw->init ||
	    !(hw->init->flags & CLK_IS_MEASURE))
		return -EINVAL;

	mutex_lock(&clk_debug_lock);

	if (measure && measure != hw)
		ret = -EBUSY;
	else
		measure = hw;

	mutex_unlock(&clk_debug_lock);

	return ret;
}
EXPORT_SYMBOL(clk_debug_measure_register);

void clk_debug_measure_unregister(struct clk_hw *hw)
{
	mutex_lock(&clk_debug_lock);

	if (measure == hw) {
		struct clk_debug_mux *meas = to_clk_measure(hw);

		measure = NULL;
		clk_debug_release_votes(meas);
	}

	mutex_unlock(&clk_debug_lock);
}
EXPORT_SYMBOL(clk_debug_measure_unregister);


static int clk_debug_shared_release(
		struct clk_debug_vote_state *state)
{
	int ret;

	switch (state->backend) {
	case CLK_DEBUG_VOTE_NONE:
		return 0;

	case CLK_DEBUG_VOTE_ICC:
		ret = icc_set_bw(state->path, 0, 0);
		break;

	case CLK_DEBUG_VOTE_LEGACY:
		ret = msm_bus_scale_client_update_request(
				state->bus_cl_id, 0);
		break;

	default:
		return -EINVAL;
	}

	if (!ret)
		state->backend = CLK_DEBUG_VOTE_NONE;

	return ret;
}

static int clk_debug_shared_vote(
		struct clk_debug_vote_state *state, bool enable)
{
	int ret = 0;

	mutex_lock(&state->lock);

	if (!enable) {
		if (!state->users) {
			ret = -EINVAL;
			goto unlock;
		}

		state->users--;

		if (!state->users)
			ret = clk_debug_shared_release(state);

		goto unlock;
	}

	if (state->shutting_down) {
		ret = -ENODEV;
		goto unlock;
	}

	if (state->users) {
		if (state->users == (unsigned int)-1) {
			ret = -EOVERFLOW;
			goto unlock;
		}

		state->users++;
		goto unlock;
	}

	/*
	 * A failed final release leaves the backend recorded.
	 * Resolve that outstanding vote before acquiring again.
	 */
	if (state->backend != CLK_DEBUG_VOTE_NONE) {
		ret = clk_debug_shared_release(state);
		if (ret)
			goto unlock;
	}

	/*
	 * Clock controllers may register before the ICC provider.
	 * Acquire the missing path on demand, under the vote lock.
	 * Late paths are explicitly owned by this vote state.
	 */
	if (!state->path && state->icc_name &&
	    IS_ENABLED(CONFIG_INTERCONNECT) &&
	    state->dev->of_node &&
	    of_find_property(state->dev->of_node,
			     "interconnects", NULL)) {
		struct icc_path *path;

		path = of_icc_get(state->dev, state->icc_name);
		if (IS_ERR(path)) {
			ret = PTR_ERR(path);
			if (ret != -EPROBE_DEFER ||
			    !state->bus_cl_id)
				goto unlock;
		} else if (path) {
			state->path = path;
			state->late_path_owned = true;
			dev_dbg(state->dev,
				"Late ICC path acquired: %s\n",
				state->icc_name);
		}
	}

	if (IS_ERR(state->path)) {
		ret = PTR_ERR(state->path);
		goto unlock;
	}

	if (state->path) {
		ret = icc_set_bw(state->path, 0, 1);
		if (!ret) {
			state->backend = CLK_DEBUG_VOTE_ICC;
			goto acquired;
		}

		if (ret != -EAGAIN || !state->bus_cl_id)
			goto unlock;
	}

	if (!state->bus_cl_id) {
		ret = -ENODEV;
		goto unlock;
	}

	ret = msm_bus_scale_client_update_request(
			state->bus_cl_id, 1);

	if (ret)
		goto unlock;

	state->backend = CLK_DEBUG_VOTE_LEGACY;

acquired:
	state->users = 1;

unlock:
	mutex_unlock(&state->lock);
	return ret;
}


static void clk_debug_vote_state_cleanup(void *data)
{
	struct clk_debug_vote_state *state = data;
	int ret;

	mutex_lock(&state->lock);

	state->shutting_down = true;

	WARN_ON(state->users);

	ret = clk_debug_shared_release(state);
	if (ret)
		pr_err("debugcc: final bandwidth cleanup failed: %d\n",
		       ret);

	state->users = 0;

	if (state->late_path_owned) {
		icc_put(state->path);
		state->path = NULL;
		state->late_path_owned = false;
	}

	if (state->bus_cl_id) {
		msm_bus_scale_unregister_client(state->bus_cl_id);
		state->bus_cl_id = 0;
	}

	mutex_unlock(&state->lock);
}

struct clk_debug_vote_state *
clk_debug_vote_state_init(struct device *dev,
			  struct icc_path *path,
			  u32 bus_cl_id, const char *icc_name)
{
	struct clk_debug_vote_state *state;
	int ret;

	if (IS_ERR(path)) {
		ret = PTR_ERR(path);
		goto unregister;
	}

	/*
	 * Clock registration must not depend on the availability
	 * of an optional debug bandwidth backend.
	 * Shared vote acquisition still rejects missing backends.
	 */
	if (!path && !bus_cl_id)
		dev_warn(dev, "Clock debug bandwidth backend unavailable\n");

	state = devm_kzalloc(dev, sizeof(*state), GFP_KERNEL);
	if (!state) {
		ret = -ENOMEM;
		goto unregister;
	}

	mutex_init(&state->lock);

	state->dev = dev;
	state->icc_name = icc_name;
	state->path = path;
	state->bus_cl_id = bus_cl_id;
	state->backend = CLK_DEBUG_VOTE_NONE;

	ret = devm_add_action_or_reset(dev,
			clk_debug_vote_state_cleanup, state);
	if (ret)
		return ERR_PTR(ret);

	return state;

unregister:
	if (bus_cl_id)
		msm_bus_scale_unregister_client(bus_cl_id);

	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(clk_debug_vote_state_init);


static void clk_debug_vote_detach(void *data)
{
	struct clk_hw *hw = data;

	hw->debug_vote = NULL;
}

int clk_debug_vote_attach(struct device *dev,
			  struct clk_hw *hw,
			  struct clk_debug_vote_state *state)
{
	if (!hw || IS_ERR_OR_NULL(state))
		return -EINVAL;

	if (hw->debug_vote)
		return -EBUSY;

	hw->debug_vote = state;

	return devm_add_action_or_reset(dev,
			clk_debug_vote_detach, hw);
}
EXPORT_SYMBOL_GPL(clk_debug_vote_attach);

int clk_debug_bus_vote(struct clk_hw *hw, bool enable)
{
	struct icc_path *path;
	u32 bus_cl_id;
	int ret;

	if (!hw || !hw->init)
		return -EINVAL;

	if (hw->debug_vote)
		return clk_debug_shared_vote(
				hw->debug_vote, enable);

	path = hw->init->icc_path;
	bus_cl_id = hw->init->bus_cl_id;

	if (IS_ERR(path))
		return PTR_ERR(path);

	if (path) {
		ret = icc_set_bw(path, 0, enable ? 1 : 0);
		if (!ret)
			return 0;

		if (ret != -EAGAIN || !bus_cl_id)
			return ret;
	}

	if (bus_cl_id)
		return msm_bus_scale_client_update_request(
				bus_cl_id, enable);

	return 0;
}
EXPORT_SYMBOL_GPL(clk_debug_bus_vote);

/**
 * map_debug_bases - maps each debug mux based on phandle
 * @pdev: the platform device used to find phandles
 * @base: regmap base name used to look up phandle
 * @mux: debug mux that requires a regmap
 *
 * This function attempts to look up and map a regmap for a debug mux
 * using syscon_regmap_lookup_by_phandle if the base name property exists
 * and assigns an appropriate regmap.
 *
 * Returns 0 on success, -EBADR when it can't find base name, -EERROR otherwise.
 */
int map_debug_bases(struct platform_device *pdev, const char *base,
		    struct clk_debug_mux *mux)
{
	if (!of_get_property(pdev->dev.of_node, base, NULL))
		return -EBADR;

	mux->regmap = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						     base);
	if (IS_ERR(mux->regmap)) {
		pr_err("Failed to map %s (ret=%ld)\n", base,
				PTR_ERR(mux->regmap));
		return PTR_ERR(mux->regmap);
	}
	return 0;
}
EXPORT_SYMBOL(map_debug_bases);

/**
 * qcom_clk_dump - dump the HW specific registers associated with this clock
 * and regulator
 * @clk: clock source
 * @regulator: regulator
 * @calltrace: indicates whether calltrace is required
 *
 * This function attempts to print all the registers associated with the
 * clock, it's parents and regulator.
 */
void qcom_clk_dump(struct clk *clk, struct regulator *regulator,
			bool calltrace)
{
	struct clk_hw *hw;

	if (!IS_ERR_OR_NULL(regulator))
		gdsc_debug_print_regs(regulator);

	if (IS_ERR_OR_NULL(clk))
		return;

	hw = __clk_get_hw(clk);
	if (IS_ERR_OR_NULL(hw))
		return;

	pr_info("Dumping %s Registers:\n", clk_hw_get_name(hw));
	WARN_CLK(hw->core, clk_hw_get_name(hw), calltrace, "");
}
EXPORT_SYMBOL(qcom_clk_dump);

/**
 * qcom_clk_bulk_dump - dump the HW specific registers associated with clocks
 * and regulator
 * @num_clks: the number of clk_bulk_data
 * @clks: the clk_bulk_data table of consumer
 * @regulator: regulator source
 * @calltrace: indicates whether calltrace is required
 *
 * This function attempts to print all the registers associated with the
 * clocks in the list and regulator.
 */
void qcom_clk_bulk_dump(int num_clks, struct clk_bulk_data *clks,
			struct regulator *regulator, bool calltrace)
{
	int i;

	if (!IS_ERR_OR_NULL(regulator))
		gdsc_debug_print_regs(regulator);

	if (IS_ERR_OR_NULL(clks))
		return;

	for (i = 0; i < num_clks; i++)
		qcom_clk_dump(clks[i].clk, NULL, calltrace);
}
EXPORT_SYMBOL(qcom_clk_bulk_dump);
