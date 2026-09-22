// SPDX-License-Identifier: GPL-2.0
/*
 * Multi-Gen LRU aging and reclaim machinery for the 4.19 backport.
 */

#include <linux/mm.h>
#include <linux/mm_inline.h>
#include <linux/mmzone.h>
#include <linux/jump_label.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/string.h>
#include <linux/swap.h>

#ifdef CONFIG_LRU_GEN
static DEFINE_STATIC_KEY_FALSE(lru_gen_caps);
static bool __read_mostly lru_gen_boot_enabled =
	IS_ENABLED(CONFIG_LRU_GEN_ENABLED);
static unsigned int __read_mostly lru_gen_min_ttl_ms;
static unsigned int __read_mostly lru_gen_age_period_ms = 1000;

static int __init setup_lru_gen(char *str)
{
	if (!str)
		return -EINVAL;

	if (!strcmp(str, "1") || !strcmp(str, "on") || !strcmp(str, "y"))
		lru_gen_boot_enabled = true;
	else if (!strcmp(str, "0") || !strcmp(str, "off") || !strcmp(str, "n"))
		lru_gen_boot_enabled = false;
	else
		return -EINVAL;

	return 0;
}
early_param("lru_gen", setup_lru_gen);

static int __init setup_lru_gen_min_ttl(char *str)
{
	unsigned int val;

	if (!str || kstrtouint(str, 0, &val))
		return -EINVAL;

	lru_gen_min_ttl_ms = val;

	return 0;
}
early_param("lru_gen_min_ttl_ms", setup_lru_gen_min_ttl);

static int __init setup_lru_gen_age_period(char *str)
{
	unsigned int val;

	if (!str || kstrtouint(str, 0, &val))
		return -EINVAL;

	lru_gen_age_period_ms = clamp_t(unsigned int, val, 100, 60000);

	return 0;
}
early_param("lru_gen_age_period_ms", setup_lru_gen_age_period);

static int __init init_lru_gen(void)
{
	if (lru_gen_boot_enabled)
		static_branch_enable(&lru_gen_caps);

	return 0;
}
late_initcall(init_lru_gen);

static void __maybe_unused lru_gen_advance_seq(struct lruvec *lruvec)
{
	/*
	 * Diagnostic mode: keep MGLRU accounting active but prevent all
	 * generation transitions. This isolates sequence advancement from
	 * page-generation ownership and reclaim accounting.
	 */
	(void)lruvec;
}


static int lru_gen_lru_type(enum lru_list lru)
{
	return is_file_lru(lru) ? LRU_GEN_FILE : LRU_GEN_ANON;
}

void lru_gen_update_size(struct lruvec *lruvec, struct page *page,
			 enum lru_list lru, enum zone_type zid, long delta)
{
	/*
	 * Diagnostic mode:
	 * disable all per-page MGLRU generation accounting while preserving
	 * the higher-level reclaim and scan hooks.
	 */
	(void)lruvec;
	(void)page;
	(void)lru;
	(void)zid;
	(void)delta;
}

static void __maybe_unused lru_gen_decay_pressure(struct lru_gen_struct *lrugen)
{
	unsigned int type;

	for (type = 0; type < ANON_AND_FILE; type++)
		lrugen->pressure[type] -= lrugen->pressure[type] >> 3;
}



bool lru_gen_enabled(void)
{
	return static_branch_likely(&lru_gen_caps);
}

int lru_gen_get_state(void)
{
	return READ_ONCE(lru_gen_boot_enabled);
}

int lru_gen_set_state(bool enable)
{
	bool old = READ_ONCE(lru_gen_boot_enabled);

	if (old == enable)
		return 0;

	/*
	 * Runtime transitions are unsafe in this backport because existing
	 * pages are not walked and assigned generation ownership. MGLRU must
	 * therefore remain in the state selected during boot.
	 */
	return -EBUSY;
}

int lru_gen_set_min_ttl(unsigned int ttl_ms)
{
	WRITE_ONCE(lru_gen_min_ttl_ms, ttl_ms);
	return 0;
}

unsigned int lru_gen_get_min_ttl(void)
{
	return READ_ONCE(lru_gen_min_ttl_ms);
}

int lru_gen_set_age_period(unsigned int period_ms)
{
	WRITE_ONCE(lru_gen_age_period_ms, clamp_t(unsigned int, period_ms, 100, 60000));
	return 0;
}

unsigned int lru_gen_get_age_period(void)
{
	return READ_ONCE(lru_gen_age_period_ms);
}

void lru_gen_init_lruvec(struct lruvec *lruvec)
{
	struct lru_gen_struct *lrugen = &lruvec->lrugen;
	unsigned int gen, type, zone;

	for (gen = 0; gen < MAX_NR_GENS; gen++) {
		lrugen->timestamps[gen] = jiffies;

		for (type = 0; type < ANON_AND_FILE; type++) {
			for (zone = 0; zone < MAX_NR_ZONES; zone++) {
				INIT_LIST_HEAD(&lrugen->lists[gen][type][zone]);
				lrugen->nr_pages[gen][type][zone] = 0;
			}
		}
	}

	lrugen->max_seq = MIN_NR_GENS + 1;
	lrugen->min_seq[LRU_GEN_ANON] = lrugen->max_seq - MIN_NR_GENS;
	lrugen->min_seq[LRU_GEN_FILE] = lrugen->max_seq - MIN_NR_GENS;
	lrugen->pressure[LRU_GEN_ANON] = 1;
	lrugen->pressure[LRU_GEN_FILE] = 1;
	lrugen->tiers[LRU_GEN_ANON] = 0;
	lrugen->tiers[LRU_GEN_FILE] = 0;
	lrugen->scanned[LRU_GEN_ANON] = 0;
	lrugen->scanned[LRU_GEN_FILE] = 0;
	lrugen->reclaimed[LRU_GEN_ANON] = 0;
	lrugen->reclaimed[LRU_GEN_FILE] = 0;
	lrugen->accessed[LRU_GEN_ANON] = 0;
	lrugen->accessed[LRU_GEN_FILE] = 0;
	lrugen->evicted[LRU_GEN_ANON] = 0;
	lrugen->evicted[LRU_GEN_FILE] = 0;
	lrugen->last_reclaim = jiffies;
}

void lru_gen_track_page_scan(struct lruvec *lruvec, enum lru_list lru,
			     unsigned long nr_scanned, unsigned long nr_taken,
			     unsigned long nr_reclaimed)
{
	struct lru_gen_struct *lrugen = &lruvec->lrugen;
	unsigned long gen;
	unsigned long delta;
	int type;

	if (!lru_gen_enabled() || !lru_gen_get_state() || !nr_scanned)
		return;

	type = lru_gen_lru_type(lru);

	/*
	 * MGLRU classification stage:
	 * - treat active LRU pages as "young" (max_seq)
	 * - treat inactive LRU pages as "old" (min_seq[type])
	 * - bias pressure according to isolation and reclaim efficiency
	 */
	gen = (is_active_lru(lru) ? lrugen->max_seq : lrugen->min_seq[type]) %
	      MAX_NR_GENS;

	spin_lock_irq(&lruvec_pgdat(lruvec)->lru_lock);
	lrugen->timestamps[gen] = jiffies;
	delta = nr_taken + nr_scanned - min(nr_reclaimed, nr_taken);
	lrugen->pressure[type] += delta;
	lrugen->scanned[type] += nr_scanned;
	lrugen->reclaimed[type] += nr_reclaimed;
	if (is_active_lru(lru) && lrugen->pressure[type] > nr_reclaimed)
		lrugen->pressure[type] -= nr_reclaimed;
	if (!is_active_lru(lru) && nr_reclaimed)
		lrugen->evicted[type] += nr_reclaimed;
	spin_unlock_irq(&lruvec_pgdat(lruvec)->lru_lock);
}

void lru_gen_note_access(struct lruvec *lruvec, bool file)
{
	struct lru_gen_struct *lrugen = &lruvec->lrugen;
	unsigned int type = file ? LRU_GEN_FILE : LRU_GEN_ANON;

	if (!lru_gen_enabled() || !lru_gen_get_state())
		return;

	spin_lock_irq(&lruvec_pgdat(lruvec)->lru_lock);
	lrugen->accessed[type]++;
	/*
	 * Age generations when enough new accesses arrive, so the reclaim
	 * side can keep selecting truly older generations.
	 */
	if (lrugen->accessed[type] >= SWAP_CLUSTER_MAX * 4)
		lrugen->accessed[type] = 0;
	spin_unlock_irq(&lruvec_pgdat(lruvec)->lru_lock);
}

void lru_gen_note_page_referenced(struct lruvec *lruvec, struct page *page,
				  bool from_reclaim)
{
	struct lru_gen_struct *lrugen = &lruvec->lrugen;
	unsigned int type;
	unsigned long nr_pages;
	unsigned long threshold;

	if (!lru_gen_enabled() || !lru_gen_get_state())
		return;

	type = page_is_file_cache(page) ? LRU_GEN_FILE : LRU_GEN_ANON;
	nr_pages = hpage_nr_pages(page);
	threshold = SWAP_CLUSTER_MAX * (from_reclaim ? 2 : 4);

	spin_lock_irq(&lruvec_pgdat(lruvec)->lru_lock);
	lrugen->accessed[type] += nr_pages;
	/*
	 * Reclaim-driven references are collected from page table walks
	 * (via page_referenced()), so use them as a direct aging signal.
	 */
	if (from_reclaim)
		lrugen->pressure[type] += min_t(unsigned long, nr_pages, SWAP_CLUSTER_MAX);

	if (lrugen->accessed[type] >= threshold)
		lrugen->accessed[type] = 0;
	spin_unlock_irq(&lruvec_pgdat(lruvec)->lru_lock);
}

void lru_gen_note_lru_move(struct lruvec *lruvec, enum lru_list old_lru,
			   enum lru_list new_lru, unsigned long nr_pages)
{
	struct lru_gen_struct *lrugen = &lruvec->lrugen;
	struct pglist_data *pgdat = lruvec_pgdat(lruvec);
	unsigned int old_type, new_type;
	unsigned long old_seq;

	if (!lru_gen_enabled() || !lru_gen_get_state() || !nr_pages)
		return;

	if (old_lru == LRU_UNEVICTABLE || new_lru == LRU_UNEVICTABLE)
		return;

	lockdep_assert_held(&pgdat->lru_lock);

	old_type = lru_gen_lru_type(old_lru);
	new_type = lru_gen_lru_type(new_lru);

	old_seq = is_active_lru(old_lru) ? lrugen->max_seq : lrugen->min_seq[old_type];
	if (!is_active_lru(new_lru) && old_type == new_type &&
	    old_seq == lrugen->min_seq[new_type])
		lrugen->evicted[new_type] += nr_pages;
}

void lru_gen_enter_reclaim(struct lruvec *lruvec, struct scan_control *sc)
{
	struct lru_gen_struct *lrugen = &lruvec->lrugen;
	struct pglist_data *pgdat = lruvec_pgdat(lruvec);
	unsigned long flags;
	unsigned int gen;

	(void)sc;

	if (!lru_gen_enabled() || !lru_gen_get_state())
		return;

	/*
	 * Diagnostic mode:
	 * reclaim may call this for the node and for every memcg. Never
	 * spin waiting for the shared pgdat LRU lock here; skip the hint
	 * if normal reclaim currently owns it.
	 */
	local_irq_save(flags);

	if (!spin_trylock(&pgdat->lru_lock)) {
		local_irq_restore(flags);
		return;
	}

	gen = lrugen->max_seq % MAX_NR_GENS;
	if (time_before(lrugen->timestamps[gen] + HZ / 4, jiffies))
		lrugen->timestamps[gen] = jiffies;

	spin_unlock(&pgdat->lru_lock);
	local_irq_restore(flags);
}

void lru_gen_adjust_scan(struct lruvec *lruvec, bool may_swap,
			 unsigned long *nr)
{
	/*
	 * Diagnostic mode:
	 * leave legacy reclaim scan targets completely untouched.
	 */
	(void)lruvec;
	(void)may_swap;
	(void)nr;
}

void lru_gen_tune_memcg(struct lruvec *lruvec, struct scan_control *sc,
                        unsigned long reclaimed, unsigned long scanned)
{
	/*
	 * Diagnostic mode:
	 * disable reclaim-feedback aging/tiering while isolating the
	 * MGLRU hard-freeze.
	 */
	(void)lruvec;
	(void)sc;
	(void)reclaimed;
	(void)scanned;
}

/*
 * Reclaim integration hook.
 *
 * This is intentionally conservative for the first stage of this backport:
 * keep classic reclaim behavior unless full MGLRU scan/evict wiring is
 * available. Returning false hands control back to legacy shrink_node().
 */
#endif
