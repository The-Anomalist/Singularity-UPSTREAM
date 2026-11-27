// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <asm/cacheflush.h>
#include <linux/atomic.h>
#include <linux/highmem.h>
#include <linux/mm.h>
#include <linux/jiffies.h>
#include <linux/llist.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/scatterlist.h>
#include <linux/swap.h>
#include <linux/workqueue.h>

#include "kgsl_device.h"
#include "kgsl_pool.h"
#include "kgsl_sharedmem.h"

#define KGSL_MAX_POOLS 6
#define KGSL_MAX_POOL_ORDER 8
/*
 * Keep an upper bound for per-pool reservation requests from DT.
 * This allows high-memory targets to opt into larger reservations
 * (up to 2.5GB) while retaining a hard sanity limit.
 */
#define KGSL_MAX_RESERVED_PAGES	((SZ_2G + SZ_512M) >> PAGE_SHIFT)

/**
 * struct kgsl_page_pool - Structure to hold information for the pool
 * @pool_order: Page order describing the size of the page
 * @page_count: Number of pages currently present in the pool
 * @reserved_pages: Number of pages reserved at init for the pool
 * @allocation_allowed: Tells if reserved pool gets exhausted, can we allocate
 * from system memory
 * @max_pages: Limit on number of pages this pool can hold
 * @list_lock: Serializes page list removals
 * @page_list: List of pages held/reserved in this pool
 */
struct kgsl_page_pool {
	unsigned int pool_order;
	atomic_t page_count;
	unsigned int reserved_pages;
	bool allocation_allowed;
	unsigned int max_pages;
	spinlock_t list_lock;
	struct llist_head page_list;
};

static struct kgsl_page_pool kgsl_pools[KGSL_MAX_POOLS];
static int kgsl_num_pools;
static unsigned int kgsl_pool_max_pages;

/*
 * Serialize pool admission and removal across all page orders.
 * Aggregate accounting uses base pages, not compound entries.
 */
static DEFINE_SPINLOCK(kgsl_pool_account_lock);
static unsigned long kgsl_pool_cached_pages;

/* Singularity AGMM: GPU pool observability */
static atomic64_t kgsl_pool_hits = ATOMIC64_INIT(0);
static atomic64_t kgsl_pool_misses = ATOMIC64_INIT(0);
static atomic64_t kgsl_pool_shrink_pages = ATOMIC64_INIT(0);

/* AGMM: foreground physical allocation diagnostics. */
static atomic64_t kgsl_pool_alloc_attempts[KGSL_MAX_POOL_ORDER + 1];
static atomic64_t kgsl_pool_alloc_successes[KGSL_MAX_POOL_ORDER + 1];
static atomic64_t kgsl_pool_alloc_failures[KGSL_MAX_POOL_ORDER + 1];

static struct page *kgsl_pool_alloc_system(gfp_t mask,
                                         unsigned int order)
{
    struct page *page;

    if (order > KGSL_MAX_POOL_ORDER)
        return alloc_pages(mask, order);

    atomic64_inc(&kgsl_pool_alloc_attempts[order]);

    page = alloc_pages(mask, order);

    if (page)
        atomic64_inc(&kgsl_pool_alloc_successes[order]);
    else
        atomic64_inc(&kgsl_pool_alloc_failures[order]);

    return page;
}
/*
 * Allow high-order pool misses to spill to system allocation before
 * immediately degrading to lower-order pages.
 *
 * Some heavy GPU benchmarks can transiently exhaust reserved pools, and
 * forcing smaller fallback pages increases mapping pressure and can cause
 * workloads to timeout before completion.
 */
static bool kgsl_pool_sysmem_fallback = true;

static bool kgsl_pool_background_refill = true;
module_param_named(pool_background_refill, kgsl_pool_background_refill,
		bool, 0644);
MODULE_PARM_DESC(pool_background_refill,
		"Refill KGSL page pools in the background after allocation bursts");

#define KGSL_POOL_REFILL_BATCH 16
#define KGSL_POOL_RECLAIM_COOLDOWN_MS 500

static unsigned long kgsl_pool_reclaim_until;
static bool kgsl_pool_reclaim_cooldown;
static atomic64_t kgsl_pool_refill_reclaim_skips =
	ATOMIC64_INIT(0);

static bool kgsl_pool_recent_reclaim(void)
{
	return READ_ONCE(kgsl_pool_reclaim_cooldown) &&
		time_before(jiffies,
			READ_ONCE(kgsl_pool_reclaim_until));
}

/*
 * Singularity AGMM pressure-aware refill.
 *
 * Background caching is opportunistic. Never restrict demand
 * allocations based on this policy.
 */
static atomic64_t kgsl_pool_refill_throttled = ATOMIC64_INIT(0);
static atomic64_t kgsl_pool_refill_skipped = ATOMIC64_INIT(0);

static unsigned int kgsl_pool_adaptive_batch(void)
{
	long available = si_mem_available();
	unsigned long total = totalram_pages;
	unsigned long floor = SZ_256M >> PAGE_SHIFT;

	if (available <= 0)
		return 0;

	if ((unsigned long)available < max(floor, total / 10))
		return 0;

	if ((unsigned long)available < total / 5)
		return 4;

	return KGSL_POOL_REFILL_BATCH;
}


static void kgsl_pool_refill_worker(struct work_struct *work);
static DECLARE_WORK(kgsl_pool_refill_work, kgsl_pool_refill_worker);


/* Returns KGSL pool corresponding to input page order*/
static struct kgsl_page_pool *
_kgsl_get_pool_from_order(unsigned int order)
{
	int i;

	for (i = 0; i < kgsl_num_pools; i++) {
		if (kgsl_pools[i].pool_order == order)
			return &kgsl_pools[i];
	}

	return NULL;
}

/*
 * AGMM: caller must hold kgsl_pool_account_lock.
 * All aggregate accounting is expressed in base pages.
 */
static bool kgsl_pool_can_cache(struct kgsl_page_pool *pool,
                               unsigned long pages)
{
    if ((unsigned int)atomic_read(&pool->page_count) >=
            pool->max_pages)
        return false;

    if (kgsl_pool_max_pages &&
        (kgsl_pool_cached_pages >= kgsl_pool_max_pages ||
         pages > kgsl_pool_max_pages - kgsl_pool_cached_pages))
        return false;

    return true;
}

/* Add a page to specified pool */
static void
_kgsl_pool_add_page(struct kgsl_page_pool *pool, struct page *p)
{
	unsigned long pages = 1UL << pool->pool_order;

	/*
	 * Sanity check to make sure we don't re-pool a page that
	 * somebody else has a reference to.
	 */
	if (WARN_ON_ONCE(unlikely(page_count(p) > 1))) {
		__free_pages(p, pool->pool_order);
		return;
	}

	/*
	 * Avoid expensive zeroing and cache maintenance when
	 * the page cannot currently be admitted.
	 */
	spin_lock(&kgsl_pool_account_lock);
	if (!kgsl_pool_can_cache(pool, pages)) {
		spin_unlock(&kgsl_pool_account_lock);
		__free_pages(p, pool->pool_order);
		return;
	}
	spin_unlock(&kgsl_pool_account_lock);

	/*
	 * Keep zeroing outside the lock. Every cached page
	 * must be sanitized before publication.
	 */
	kgsl_zero_page(p, pool->pool_order);

	spin_lock(&kgsl_pool_account_lock);

	/* Capacity may have changed while zeroing. */
	if (!kgsl_pool_can_cache(pool, pages)) {
		spin_unlock(&kgsl_pool_account_lock);
		__free_pages(p, pool->pool_order);
		return;
	}

	atomic_inc(&pool->page_count);
	kgsl_pool_cached_pages += pages;

	mod_node_page_state(page_pgdat(p), NR_KERNEL_MISC_RECLAIMABLE,
			    pages);

	llist_add((struct llist_node *)&p->lru, &pool->page_list);

	spin_unlock(&kgsl_pool_account_lock);
}

/* Returns a page from specified pool */
static struct page *
_kgsl_pool_get_page(struct kgsl_page_pool *pool)
{
	struct llist_node *node;
	struct page *p = NULL;

	spin_lock(&kgsl_pool_account_lock);
	node = llist_del_first(&pool->page_list);

	if (node) {
		atomic_dec(&pool->page_count);
		kgsl_pool_cached_pages -= 1UL << pool->pool_order;
		p = container_of((struct list_head *)node, struct page, lru);
		mod_node_page_state(page_pgdat(p),
				    NR_KERNEL_MISC_RECLAIMABLE,
				    -(1 << pool->pool_order));
	}

	spin_unlock(&kgsl_pool_account_lock);

	return p;
}

/* Returns the number of pages in specified pool */
static int
kgsl_pool_size(struct kgsl_page_pool *kgsl_pool)
{
	return atomic_read(&kgsl_pool->page_count) *
		(1 << kgsl_pool->pool_order);
}

static int kgsl_pool_page_count(struct kgsl_page_pool *kgsl_pool)
{
	return atomic_read(&kgsl_pool->page_count);
}

static void kgsl_pool_schedule_refill(struct kgsl_page_pool *pool)
{
	if (!kgsl_pool_background_refill || !pool || !pool->reserved_pages)
		return;

	if (kgsl_pool_recent_reclaim()) {
		atomic64_inc(&kgsl_pool_refill_reclaim_skips);
		return;
	}

	if (kgsl_pool_page_count(pool) * 2 <= pool->reserved_pages)
		schedule_work(&kgsl_pool_refill_work);
}

/* Returns the number of pages in all kgsl page pools */
static int kgsl_pool_size_total(void)
{
	int i;
	int total = 0;

	for (i = 0; i < kgsl_num_pools; i++)
		total += kgsl_pool_size(&kgsl_pools[i]);
	return total;
}


/* Read-only GPU pool telemetry. All sizes are reported in bytes. */
static int kgsl_pool_cached_get(char *buf,
        const struct kernel_param *kp)
{
    return scnprintf(buf, PAGE_SIZE, "%llu\n",
        (unsigned long long)kgsl_pool_size_total() << PAGE_SHIFT);
}

static int kgsl_pool_reclaimable_get(char *buf,
        const struct kernel_param *kp)
{
    unsigned long long pages = 0;
    int i;

    for (i = 0; i < kgsl_num_pools; i++)
        pages += kgsl_pool_size(&kgsl_pools[i]);

    return scnprintf(buf, PAGE_SIZE, "%llu\n",
        pages << PAGE_SHIFT);
}

static int kgsl_pool_protected_get(char *buf,
        const struct kernel_param *kp)
{
    /* All idle pool pages are now shrinker eligible. */
    return scnprintf(buf, PAGE_SIZE, "0\n");
}

static int kgsl_pool_stats_get(char *buf,
        const struct kernel_param *kp)
{
    int i, len = 0;

    len += scnprintf(buf + len, PAGE_SIZE - len,
        "size_bytes entries cached_bytes reserved max_entries reclaimable allocation_allowed\n");

    for (i = 0; i < kgsl_num_pools; i++) {
        struct kgsl_page_pool *pool = &kgsl_pools[i];
        unsigned int count = kgsl_pool_page_count(pool);
        unsigned long long size =
            (unsigned long long)PAGE_SIZE << pool->pool_order;

        len += scnprintf(buf + len, PAGE_SIZE - len,
            "%llu %u %llu %u %u %u %u\n",
            size, count, size * count,
            pool->reserved_pages, pool->max_pages,
            1U, (unsigned int)pool->allocation_allowed);
    }

    return len;
}

static int kgsl_pool_hits_get(char *buf,
        const struct kernel_param *kp)
{
    return scnprintf(buf, PAGE_SIZE, "%lld\n",
        (long long)atomic64_read(&kgsl_pool_hits));
}

static int kgsl_pool_misses_get(char *buf,
        const struct kernel_param *kp)
{
    return scnprintf(buf, PAGE_SIZE, "%lld\n",
        (long long)atomic64_read(&kgsl_pool_misses));
}

static int kgsl_pool_shrink_get(char *buf,
        const struct kernel_param *kp)
{
    return scnprintf(buf, PAGE_SIZE, "%llu\n",
        (unsigned long long)
        atomic64_read(&kgsl_pool_shrink_pages) << PAGE_SHIFT);
}

#define KGSL_POOL_RO_PARAM(name, getter) \
    static const struct kernel_param_ops name##_ops = { \
        .get = getter, \
    }; \
    module_param_cb(name, &name##_ops, NULL, 0444)

static int kgsl_pool_limit_get(char *buf,
        const struct kernel_param *kp)
{
    return scnprintf(buf, PAGE_SIZE, "%llu\n",
        (unsigned long long)kgsl_pool_max_pages << PAGE_SHIFT);
}

KGSL_POOL_RO_PARAM(pool_cache_limit_bytes, kgsl_pool_limit_get);

static int kgsl_pool_refill_throttled_get(char *buf,
		const struct kernel_param *kp)
{
	return scnprintf(buf, PAGE_SIZE, "%lld\n",
		(long long)atomic64_read(&kgsl_pool_refill_throttled));
}

static int kgsl_pool_refill_skipped_get(char *buf,
		const struct kernel_param *kp)
{
	return scnprintf(buf, PAGE_SIZE, "%lld\n",
		(long long)atomic64_read(&kgsl_pool_refill_skipped));
}

static int kgsl_pool_refill_batch_get(char *buf,
		const struct kernel_param *kp)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n",
		kgsl_pool_adaptive_batch());
}

KGSL_POOL_RO_PARAM(pool_refill_throttled,
		kgsl_pool_refill_throttled_get);
KGSL_POOL_RO_PARAM(pool_refill_skipped,
		kgsl_pool_refill_skipped_get);
KGSL_POOL_RO_PARAM(pool_refill_batch,
		kgsl_pool_refill_batch_get);

static int kgsl_pool_refill_reclaim_skips_get(char *buf,
		const struct kernel_param *kp)
{
	return scnprintf(buf, PAGE_SIZE, "%lld\n",
		(long long)atomic64_read(
			&kgsl_pool_refill_reclaim_skips));
}

static int kgsl_pool_reclaim_active_get(char *buf,
		const struct kernel_param *kp)
{
	return scnprintf(buf, PAGE_SIZE, "%u\n",
		(unsigned int)kgsl_pool_recent_reclaim());
}

KGSL_POOL_RO_PARAM(pool_refill_reclaim_skips,
		kgsl_pool_refill_reclaim_skips_get);

KGSL_POOL_RO_PARAM(pool_reclaim_active,
		kgsl_pool_reclaim_active_get);

static int kgsl_pool_order_stats_get(char *buf,
        const struct kernel_param *kp)
{
    int order, len = 0;

    len += scnprintf(buf + len, PAGE_SIZE - len,
        "order size_bytes attempts successes failures\n");

    for (order = 0; order <= KGSL_MAX_POOL_ORDER; order++) {
        len += scnprintf(buf + len, PAGE_SIZE - len,
            "%d %lu %lld %lld %lld\n",
            order, PAGE_SIZE << order,
            (long long)atomic64_read(
                &kgsl_pool_alloc_attempts[order]),
            (long long)atomic64_read(
                &kgsl_pool_alloc_successes[order]),
            (long long)atomic64_read(
                &kgsl_pool_alloc_failures[order]));
    }

    return len;
}

KGSL_POOL_RO_PARAM(pool_order_stats,
        kgsl_pool_order_stats_get);

KGSL_POOL_RO_PARAM(pool_cached_bytes, kgsl_pool_cached_get);
KGSL_POOL_RO_PARAM(pool_reclaimable_bytes, kgsl_pool_reclaimable_get);
KGSL_POOL_RO_PARAM(pool_protected_bytes, kgsl_pool_protected_get);
KGSL_POOL_RO_PARAM(pool_stats, kgsl_pool_stats_get);
KGSL_POOL_RO_PARAM(pool_hits, kgsl_pool_hits_get);
KGSL_POOL_RO_PARAM(pool_misses, kgsl_pool_misses_get);
KGSL_POOL_RO_PARAM(pool_shrink_bytes, kgsl_pool_shrink_get);

/*
 * This will shrink the specified pool by num_pages or its pool_size,
 * whichever is smaller.
 */
static unsigned int
_kgsl_pool_shrink(struct kgsl_page_pool *pool, int num_pages)
{
	int j;
	unsigned int pcount = 0;

	if (pool == NULL || num_pages <= 0)
		return pcount;

	for (j = 0; j < num_pages >> pool->pool_order; j++) {
		struct page *page = _kgsl_pool_get_page(pool);

		if (page != NULL) {
			__free_pages(page, pool->pool_order);
			pcount += (1 << pool->pool_order);
		} else {
			/* Break as this pool is empty */
			break;
		}
	}

	return pcount;
}

/*
 * This function reduces the total pool size
 * to number of pages specified by target_pages.
 *
 * If target_pages are greater than current pool size
 * nothing needs to be done otherwise remove
 * (current_pool_size - target_pages) pages from pool
 * starting from higher order pool.
 */
static unsigned long
kgsl_pool_reduce(unsigned int target_pages, bool exit)
{
	int total_pages = 0;
	int i;
	int nr_removed;
	struct kgsl_page_pool *pool;
	unsigned long pcount = 0;

	total_pages = kgsl_pool_size_total();

	for (i = (kgsl_num_pools - 1); i >= 0; i--) {
		pool = &kgsl_pools[i];

		/*
		 * Only reduce the pool sizes for pools which are allowed to
		 * allocate memory unless we are at close, in which case the
		 * reserved memory for all pools needs to be freed
		 */
		if (!pool->allocation_allowed && !exit)
			continue;

		nr_removed = total_pages - target_pages - pcount;
		if (nr_removed <= 0)
			return pcount;

		/* Round up to integral number of pages in this pool */
		nr_removed = ALIGN(nr_removed, 1 << pool->pool_order);

		/* Remove nr_removed pages from this pool*/
		pcount += _kgsl_pool_shrink(pool, nr_removed);
	}

	return pcount;
}

/**
 * kgsl_pool_free_pages() - Free pages in the pages array
 * @pages: pointer of the pages array
 *
 * Free the pages by collapsing any physical adjacent pages.
 * Pages are added back to the pool, if pool has sufficient space
 * otherwise they are given back to system.
 */
void kgsl_pool_free_pages(struct page **pages, unsigned int pcount)
{
	int i;

	if (pages == NULL || pcount == 0)
		return;

	if (WARN(!kern_addr_valid((unsigned long)pages),
		"Address of pages=%pK is not valid\n", pages))
		return;

	for (i = 0; i < pcount;) {
		/*
		 * Free each page or compound page group individually.
		 */
		struct page *p = pages[i];

		if (WARN(!kern_addr_valid((unsigned long)p),
			"Address of page=%pK is not valid\n", p))
			return;

		i += 1 << compound_order(p);
		kgsl_pool_free_page(p);
	}
}
static int kgsl_pool_idx_lookup(unsigned int order)
{
	int i;

	for (i = 0; i < kgsl_num_pools; i++)
		if (order == kgsl_pools[i].pool_order)
			return i;

	return -ENOMEM;
}

static int kgsl_pool_get_retry_order(unsigned int order)
{
	int i;

	for (i = kgsl_num_pools-1; i > 0; i--)
		if (order >= kgsl_pools[i].pool_order)
			return kgsl_pools[i].pool_order;

	return 0;
}

/**
 * kgsl_pool_alloc_page() - Allocate a page of requested size
 * @page_size: Size of the page to be allocated
 * @pages: pointer to hold list of pages, should be big enough to hold
 * requested page
 * @len: Length of array pages.
 *
 * Return total page count on success and negative value on failure
 */
int kgsl_pool_alloc_page(int *page_size, struct page **pages,
			unsigned int pages_len, unsigned int *align,
			struct kgsl_memdesc *memdesc)
{
	int j;
	int pcount = 0;
	struct kgsl_page_pool *pool;
	struct page *page = NULL;
	struct page *p = NULL;
	int order = get_order(*page_size);
	int pool_idx;
	size_t size = 0;

	if ((pages == NULL) || pages_len < (*page_size >> PAGE_SHIFT))
		return -EINVAL;

	/* If the pool is not configured get pages from the system */
	if (!kgsl_num_pools) {
		gfp_t gfp_mask = kgsl_gfp_mask(order);

		page = kgsl_pool_alloc_system(gfp_mask, order);
		if (page == NULL) {
			/* Retry with lower order pages */
			if (order > 0) {
				size = PAGE_SIZE << --order;
				goto eagain;

			} else
				return -ENOMEM;
		}
		kgsl_zero_page(page, order);
		goto done;
	}

	pool = _kgsl_get_pool_from_order(order);
	if (pool == NULL) {
		/* Retry with lower order pages */
		if (order > 0) {
			size = PAGE_SIZE << kgsl_pool_get_retry_order(order);
			goto eagain;
		} else {
			/*
			 * Fall back to direct allocation in case
			 * pool with zero order is not present
			 */
			gfp_t gfp_mask = kgsl_gfp_mask(order);

			page = kgsl_pool_alloc_system(gfp_mask, order);
			if (page == NULL)
				return -ENOMEM;
			kgsl_zero_page(page, order);
			goto done;
		}
	}

	pool_idx = kgsl_pool_idx_lookup(order);
	page = _kgsl_pool_get_page(pool);
	if (page) {
		atomic64_inc(&kgsl_pool_hits);
		kgsl_pool_schedule_refill(pool);
	} else {
		atomic64_inc(&kgsl_pool_misses);
	}

	/* Allocate a new page if not allocated from pool */
	if (page == NULL) {
		gfp_t gfp_mask = kgsl_gfp_mask(order);
		bool tried_sysmem_fallback = false;

		/* Only allocate non-reserved memory for certain pools */
		if (!pool->allocation_allowed && pool_idx > 0) {
			/*
			 * For reserved-only pools, try a direct system
			 * allocation first to preserve larger page mappings
			 * under transient memory pressure. If this fails, fall
			 * back to the legacy lower-order retry behavior.
			 */
			if (kgsl_pool_sysmem_fallback) {
				page = kgsl_pool_alloc_system(gfp_mask, order);
				tried_sysmem_fallback = true;
				if (page)
					kgsl_zero_page(page, order);
			}

			if (!page) {
				size = PAGE_SIZE <<
					kgsl_pools[pool_idx-1].pool_order;
				goto eagain;
			}
		}

		if (!page)
			page = kgsl_pool_alloc_system(gfp_mask, order);

		if (!page) {
			if (pool_idx > 0) {
				/* Retry with lower order pages */
				size = PAGE_SIZE <<
					kgsl_pools[pool_idx-1].pool_order;
				goto eagain;
			} else
				return -ENOMEM;
		}

		if (!tried_sysmem_fallback)
			kgsl_zero_page(page, order);
	}

done:
	for (j = 0; j < (*page_size >> PAGE_SHIFT); j++) {
		p = nth_page(page, j);
		pages[pcount] = p;
		pcount++;
	}

	mod_node_page_state(page_pgdat(page), NR_UNRECLAIMABLE_PAGES,
					(1 << order));
	return pcount;

eagain:
	*page_size = kgsl_get_page_size(size,
			ilog2(size), memdesc);
	*align = ilog2(*page_size);
	return -EAGAIN;
}

void kgsl_pool_free_page(struct page *page)
{
	struct kgsl_page_pool *pool;
	int page_order;

	if (page == NULL)
		return;

	page_order = compound_order(page);

	mod_node_page_state(page_pgdat(page), NR_UNRECLAIMABLE_PAGES,
					-(1 << page_order));

	pool = _kgsl_get_pool_from_order(page_order);
	if (pool) {
		_kgsl_pool_add_page(pool, page);
		return;
	}

	/* Give back to system as not added to pool */
	__free_pages(page, page_order);
}

/*
 * Return true if the pool of specified page size is supported
 * or no pools are supported otherwise return false.
 */
bool kgsl_pool_avaialable(int page_size)
{
	int i;

	if (!kgsl_num_pools)
		return true;

	for (i = 0; i < kgsl_num_pools; i++)
		if (ilog2(page_size >> PAGE_SHIFT) == kgsl_pools[i].pool_order)
			return true;

	return false;
}

static void kgsl_pool_reserve_pages(void)
{
	int i, j;
	unsigned int page_count;

	for (i = 0; i < kgsl_num_pools; i++) {
		struct page *page;

		page_count = min_t(unsigned int, kgsl_pools[i].max_pages,
				kgsl_pools[i].reserved_pages);
		for (j = 0; j < page_count; j++) {
			int order = kgsl_pools[i].pool_order;
			gfp_t gfp_mask = kgsl_gfp_mask(order);

			page = alloc_pages(gfp_mask, order);
			if (page != NULL)
				_kgsl_pool_add_page(&kgsl_pools[i], page);
		}
	}
}

static void kgsl_pool_refill_worker(struct work_struct *work)
{
	unsigned int adaptive_batch;

	int i;


	if (!READ_ONCE(kgsl_pool_background_refill))
		return;

	if (kgsl_pool_recent_reclaim()) {
		atomic64_inc(&kgsl_pool_refill_reclaim_skips);
		return;
	}

	adaptive_batch = kgsl_pool_adaptive_batch();

	if (!adaptive_batch) {
		atomic64_inc(&kgsl_pool_refill_skipped);
		return;
	}

	if (adaptive_batch < KGSL_POOL_REFILL_BATCH)
		atomic64_inc(&kgsl_pool_refill_throttled);

	for (i = 0; i < kgsl_num_pools; i++) {
		struct kgsl_page_pool *pool = &kgsl_pools[i];
		int target = min_t(unsigned int, pool->reserved_pages,
				pool->max_pages);
		int refill = target - kgsl_pool_page_count(pool);
		int j;

		if (refill <= 0)
			continue;

		adaptive_batch = kgsl_pool_adaptive_batch();
		if (!adaptive_batch) {
			atomic64_inc(&kgsl_pool_refill_skipped);
			return;
		}

		refill = min_t(int, refill, adaptive_batch);

		for (j = 0; j < refill; j++) {
			gfp_t gfp_mask = kgsl_gfp_mask(pool->pool_order);
			struct page *page;
			bool can_cache;

			if (kgsl_pool_recent_reclaim()) {
				atomic64_inc(&kgsl_pool_refill_reclaim_skips);
				return;
			}

			spin_lock(&kgsl_pool_account_lock);
			can_cache = kgsl_pool_can_cache(pool,
					1UL << pool->pool_order);
			spin_unlock(&kgsl_pool_account_lock);

			if (!can_cache)
				break;

			/*
			 * Background refill is opportunistic.
			 * Never initiate reclaim just to populate a cache.
			 */
			gfp_mask &= ~__GFP_RECLAIM;
			gfp_mask |= __GFP_NOWARN | __GFP_NORETRY;

			page = alloc_pages(gfp_mask, pool->pool_order);
			if (!page)
				break;

			_kgsl_pool_add_page(pool, page);
		}
	}
}

/* Functions for the shrinker */

static unsigned long
kgsl_pool_shrink_scan_objects(struct shrinker *shrinker,
					struct shrink_control *sc)
{
	unsigned long freed = 0;
	int i;

	/* Reclaim against the remaining budget as pool sizes change. */
	for (i = kgsl_num_pools - 1; i >= 0 && freed < sc->nr_to_scan; i--) {
		struct kgsl_page_pool *pool = &kgsl_pools[i];
		unsigned long nr;

		/*
		 * AGMM: reserved-only pools contain idle cached pages.
		 * They must remain reclaimable under system pressure.
		 */
		nr = min_t(unsigned long, sc->nr_to_scan - freed,
			   kgsl_pool_size(pool));
		/* Free whole compound pages. */
		nr = ALIGN(nr, 1UL << pool->pool_order);
		freed += _kgsl_pool_shrink(pool, nr);
	}

	/*
	 * Concurrent allocations may empty the pools after
	 * count_objects(). Stop rather than report zero progress.
	 */
	if (!freed) {
		sc->nr_scanned = 0;
		return SHRINK_STOP;
	}

	/*
	 * Compound-page rounding can free more base pages than
	 * requested. Never over-consume the VM scan budget.
	 */
	sc->nr_scanned = min(freed, sc->nr_to_scan);

	spin_lock(&kgsl_pool_account_lock);
	WRITE_ONCE(kgsl_pool_reclaim_until,
		jiffies + msecs_to_jiffies(
			KGSL_POOL_RECLAIM_COOLDOWN_MS));
	WRITE_ONCE(kgsl_pool_reclaim_cooldown, true);
	spin_unlock(&kgsl_pool_account_lock);

	atomic64_add(freed, &kgsl_pool_shrink_pages);
	return freed;
}

static unsigned long
kgsl_pool_shrink_count_objects(struct shrinker *shrinker,
					struct shrink_control *sc)
{
	unsigned long count = 0;
	int i;

	/* Trigger mem_workqueue flush to free memory */
	kgsl_schedule_work(&kgsl_driver.mem_work);

	for (i = 0; i < kgsl_num_pools; i++)
		count += kgsl_pool_size(&kgsl_pools[i]);

	return count ? count : SHRINK_EMPTY;
}

/* Shrinker callback data*/
static struct shrinker kgsl_pool_shrinker = {
	.count_objects = kgsl_pool_shrink_count_objects,
	.scan_objects = kgsl_pool_shrink_scan_objects,
	.seeks = DEFAULT_SEEKS,
	.batch = 0,
};

static void kgsl_pool_config(unsigned int order, unsigned int reserved_pages,
		bool allocation_allowed, unsigned int max_pages)
{
#ifdef CONFIG_ALLOC_BUFFERS_IN_4K_CHUNKS
	if (order > 0) {
		pr_err("kgsl: pool order:%d not supprted\n", order);
		return;
	}
#endif
	if (order > KGSL_MAX_POOL_ORDER ||
	    kgsl_num_pools >= KGSL_MAX_POOLS)
		return;

	/* DT reservations count compound entries, not base pages. */
	if (reserved_pages > (KGSL_MAX_RESERVED_PAGES >> order))
		return;

	kgsl_pools[kgsl_num_pools].pool_order = order;
	kgsl_pools[kgsl_num_pools].reserved_pages = reserved_pages;
	kgsl_pools[kgsl_num_pools].allocation_allowed = allocation_allowed;
	kgsl_pools[kgsl_num_pools].max_pages = max_pages;
	spin_lock_init(&kgsl_pools[kgsl_num_pools].list_lock);
	atomic_set(&kgsl_pools[kgsl_num_pools].page_count, 0);
	init_llist_head(&kgsl_pools[kgsl_num_pools].page_list);
	kgsl_num_pools++;
}

static void kgsl_of_parse_mempools(struct device_node *node)
{
	struct device_node *child;
	unsigned int page_size, reserved_pages = 0, max_pages = UINT_MAX;
	bool allocation_allowed;

	for_each_child_of_node(node, child) {
		unsigned int index;

		/* Optional DT properties apply to this pool only. */
		reserved_pages = 0;
		max_pages = UINT_MAX;

		if (of_property_read_u32(child, "reg", &index))
			return;

		if (index >= KGSL_MAX_POOLS)
			continue;

		if (of_property_read_u32(child, "qcom,mempool-page-size",
					&page_size))
			return;

		of_property_read_u32(child, "qcom,mempool-reserved",
				&reserved_pages);

		allocation_allowed = of_property_read_bool(child,
				"qcom,mempool-allocate");

		of_property_read_u32(child, "qcom,mempool-max-pages",
				&max_pages);

		kgsl_pool_config(ilog2(page_size >> PAGE_SHIFT), reserved_pages,
				allocation_allowed, max_pages);
	}
}

static void kgsl_of_get_mempools(struct device_node *parent)
{
	struct device_node *node = NULL;

	/*
	 * If available memory is less than 2GB first
	 * check for low memory pool configuration. If
	 * low memory configuration is not specified
	 * then fallback to default pool configuration.
	 */
	if (totalram_pages < (SZ_2G >> PAGE_SHIFT))
		node = of_find_compatible_node(parent, NULL,
				"qcom,gpu-mempools-lowmem");

	if (node == NULL)
		node = of_find_compatible_node(parent, NULL,
				"qcom,gpu-mempools");

	if (node != NULL) {
		/* Get Max pages limit for mempool */
		of_property_read_u32(node, "qcom,mempool-max-pages",
				&kgsl_pool_max_pages);
		kgsl_of_parse_mempools(node);
	}
}

/*
 * Singularity AGMM: RAM-aware GPU cache policy.
 *
 * Limit idle cached pages without restricting active GPU allocations.
 * Use managed RAM thresholds to account for firmware reservations.
 * Preserve stricter platform limits supplied by the device tree.
 */
static void kgsl_pool_apply_ram_policy(void)
{
	unsigned int cache_mib;
	unsigned int large_mib;
	unsigned int cache_pages;
	unsigned long ram_mib;
	int i;

	ram_mib = ((unsigned long)totalram_pages << PAGE_SHIFT) >> 20;

	if (ram_mib >= 10240) {
		cache_mib = 384;
		large_mib = 96;
	} else if (ram_mib >= 6656) {
		cache_mib = 256;
		large_mib = 64;
	} else if (ram_mib >= 4096) {
		cache_mib = 192;
		large_mib = 48;
	} else {
		cache_mib = 64;
		large_mib = 16;
	}

	cache_pages = (cache_mib * SZ_1M) >> PAGE_SHIFT;

	if (!kgsl_pool_max_pages)
		kgsl_pool_max_pages = cache_pages;
	else
		kgsl_pool_max_pages = min(kgsl_pool_max_pages,
					  cache_pages);

	for (i = 0; i < kgsl_num_pools; i++) {
		struct kgsl_page_pool *pool = &kgsl_pools[i];
		unsigned long entry_size = PAGE_SIZE << pool->pool_order;
		unsigned int limit;

		if (entry_size < SZ_1M)
			continue;

		limit = (large_mib * SZ_1M) / entry_size;
		pool->max_pages = min(pool->max_pages, limit);
	}

	pr_info("kgsl: AGMM RAM=%lu MiB cache_limit=%lu MiB large_pool_limit=%u MiB\n",
		ram_mib,
		((unsigned long)kgsl_pool_max_pages << PAGE_SHIFT) >> 20,
		large_mib);
}

void kgsl_init_page_pools(struct kgsl_device *device)
{
	if (device->flags & KGSL_FLAG_USE_SHMEM)
		return;

	/* Get GPU mempools data and configure pools */
	kgsl_of_get_mempools(device->pdev->dev.of_node);

	/* Establish cache limits before admitting reserved pages. */
	kgsl_pool_apply_ram_policy();

	/* Reserve the appropriate number of pages for each pool */
	kgsl_pool_reserve_pages();

	/* Initialize shrinker */
	register_shrinker(&kgsl_pool_shrinker);
}

void kgsl_exit_page_pools(void)
{
	cancel_work_sync(&kgsl_pool_refill_work);

	/* Release all pages in pools, if any.*/
	kgsl_pool_reduce(0, true);

	/* Unregister shrinker */
	unregister_shrinker(&kgsl_pool_shrinker);
}
