/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Hugo Guiroux <hugo.guiroux at gmail dot com>
 *               UPMC, 2010-2011, Jean-Pierre Lozi <jean-pierre.lozi@lip6.fr>
 *                                Gaël Thomas <gael.thomas@lip6.fr>
 *                                Florian David <florian.david@lip6.fr>
 *                                Julia Lawall <julia.lawall@lip6.fr>
 *                                Gilles Muller <gilles.muller@lip6.fr>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of his software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <aqssclwonode.h>
#include <assert.h>
#include <errno.h>
#include <papi.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

#include "interpose.h"
#include "utils.h"
#include "waiting_policy.h"

#include <assert.h>

#ifndef CYCLE_PER_US
#error Must define CYCLE_PER_US for the current machine in Makefile or elsewhere
#endif

#ifndef FAIRLOCK_GRANULARITY
#error Must define FAIRLOCK_GRANULARITY for the current machine in Makefile or elsewhere
#endif

#define CYCLE_PER_MS (CYCLE_PER_US * 1000L)
#define CYCLE_PER_S (CYCLE_PER_MS * 1000L)

static const int prio_to_weight[40] = {
    /* -20 */ 88761, 71755, 56483, 46273, 36291,
    /* -15 */ 29154, 23254, 18705, 14949, 11916,
    /* -10 */ 9548,  7620,  6100,  4904,  3906,
    /*  -5 */ 3121,  2501,  1991,  1586,  1277,
    /*   0 */ 1024,  820,   655,   526,   423,
    /*   5 */ 335,   272,   215,   172,   137,
    /*  10 */ 110,   87,    70,    56,    45,
    /*  15 */ 36,    29,    23,    18,    15,
};

/* debugging */
#ifdef WAITER_DEBUG
typedef enum {
    RED,
    GREEN,
    BLUE,
    MAGENTA,
    YELLOW,
    CYAN,
    END,
} color_num;

static char colors[END][8] = {
    "\x1B[31m", "\x1B[32m", "\x1B[34m", "\x1B[35m", "\x1b[33m", "\x1b[36m",
};
static unsigned long counter = 0;

#define dprintf(__fmt, ...)                                                    \
    do {                                                                       \
        smp_faa(&counter, 1);                                                  \
        fprintf(stderr, "%s [DBG:%010lu: %d (%s: %d)]: " __fmt,                \
                colors[__my_cpu_id % END], counter, __my_cpu_id, __func__,     \
                __LINE__, ##__VA_ARGS__);                                      \
    } while (0);
#define dassert(v) assert((v))
#else
#define dprintf(__fmt, ...)                                                    \
    do {                                                                       \
    } while (0)
#define dassert(v)                                                             \
    do {                                                                       \
    } while (0)
#endif

#define GET_TIME() ((long long)PAPI_get_real_cyc() / (CPU_FREQ * 1000))

#define AQSSCL_MIN_WAIT_TIME 50LL
#define AQSSCL_MAX_WAIT_TIME 10000000LL
extern __thread unsigned int cur_thread_id;
extern __thread struct t_info tinfo;
extern unsigned int last_thread_id;

typedef enum {
    RED,
    GREEN,
    BLUE,
    MAGENTA,
    YELLOW,
    CYAN,
    END,
} color_num;

static char colors[END][8] = {
    "\x1B[31m", "\x1B[32m", "\x1B[34m", "\x1B[35m", "\x1b[33m", "\x1b[36m",
};

#define eprintf(__fmt, ...)                                                    \
    do {                                                                       \
        fprintf(stderr, "%s [%d (%s: %d)]: " __fmt,                            \
                colors[cur_thread_id % END], cur_thread_id, __func__,          \
                __LINE__, ##__VA_ARGS__);                                      \
    } while (0);

/* debugging */
/* #ifdef WAITER_DEBUG */
static inline void traverse_nodes(aqsscl_mutex_t *lock, aqsscl_node_t *node) {
    aqsscl_node_t *curr = node;
    int count           = 0;
    eprintf("prev shuffler: ");
    if (node->last_visited) {
        eprintf("%d\n", node->last_visited->cid);
    } else {
        eprintf("no one!\n");
    }

    printf("#coreid[lock-status:shuffle-leader:waiter-count]\n");
    for (;;) {
        if (++count > 200)
            assert(0);
        if (!curr)
            break;

        printf("%d[%d:%d:%d]->", curr->cid, curr->lstatus, curr->sleader,
               curr->wcount);
        curr = READ_ONCE(curr->next);
    }
    printf("\n");
}
/* #endif */

#define THRESHOLD (0xffff)
#ifndef UNLOCK_COUNT_THRESHOLD
#define UNLOCK_COUNT_THRESHOLD 1024
#endif

static inline uint32_t xor_random() {
    static __thread uint32_t rv = 0;

    if (rv == 0)
        rv = cur_thread_id + 1;

    uint32_t v = rv;
    v ^= v << 6;
    v ^= (uint32_t)(v) >> 21;
    v ^= v << 7;
    rv = v;

    return v & (UNLOCK_COUNT_THRESHOLD - 1);
    /* return v; */
}

static inline int keep_lock_local(void) {
    return xor_random() & THRESHOLD;
}

static inline int current_numa_node() {
    unsigned long a, d, c;
    int core;
    __asm__ volatile("rdtscp" : "=a"(a), "=d"(d), "=c"(c));
    core = c & 0xFFF;
    return core / (CPU_NUMBER / NUMA_NODES);
}

#define false 0
#define true 1

#define atomic_andnot(val, ptr) __sync_fetch_and_and((ptr), ~(val));
#define atomic_fetch_or_acquire(val, ptr) __sync_fetch_and_or((ptr), (val));

#define _AQSSCL_NOSTEAL_VAL                                                    \
    (1U << (_AQSSCL_LOCKED_OFFSET + _AQSSCL_LOCKED_BITS))
#define AQSSCL_MAX_PATIENCE_COUNT 2
#define MAX_CONT_SHFLD_COUNT 2

static inline void smp_wmb(void) {
    __asm __volatile("sfence" ::: "memory");
}

static inline void enable_stealing(aqsscl_mutex_t *lock) {
    WRITE_ONCE(lock->no_stealing, 1);
}

static inline void disable_stealing(aqsscl_mutex_t *lock) {
    WRITE_ONCE(lock->no_stealing, 0);
}

static inline uint8_t is_stealing_disabled(aqsscl_mutex_t *lock) {
    return READ_ONCE(lock->no_stealing);
}

static inline void set_sleader(struct aqsscl_node *node,
                               struct aqsscl_node *qend) {
    WRITE_ONCE(node->sleader, 1);
    if (qend != node)
        WRITE_ONCE(node->last_visited, qend);
}

static inline void clear_sleader(struct aqsscl_node *node) {
    node->sleader = 0;
}

static inline void set_waitcount(struct aqsscl_node *node, int count) {
    WRITE_ONCE(node->wcount, count);
}

/* #define USE_COUNTER */
static void shuffle_waiters(aqsscl_mutex_t *lock, struct aqsscl_node *node,
                            int is_next_waiter) {
    aqsscl_node_t *curr, *prev, *next, *last, *sleader, *qend;
    int nid               = node->nid;
    int curr_locked_count = node->wcount;
    int one_shuffle       = 0;
    uint32_t lock_ready;

    prev = READ_ONCE(node->last_visited);
    if (!prev)
        prev = node;
    sleader = NULL;
    last    = node;
    curr    = NULL;
    next    = NULL;
    qend    = NULL;

    dprintf("node (%d) with sleader (%d), wcount (%d) and lock->slocked: %d\n",
            node->cid, node->sleader, node->wcount, READ_ONCE(lock->slocked));

    if (curr_locked_count == 0)
        set_waitcount(node, ++curr_locked_count);

    clear_sleader(node);

#ifdef USE_COUNTER
    if (curr_locked_count >= AQSSCL_MAX_LOCK_COUNT) {
        sleader = READ_ONCE(node->next);
        dprintf("1. selecting new shuffler %d\n", sleader->cid);
        goto out;
    }
#else
    if (!keep_lock_local()) {
        sleader = READ_ONCE(node->next);
        goto out;
    }
#endif

    for (;;) {
        curr = READ_ONCE(prev->next);

        barrier();

        if (!curr) {
            sleader = last;
            qend    = prev;
            break;
        }

        if (curr == READ_ONCE(lock->tail)) {
            sleader = last;
            qend    = prev;
            break;
        }

        /* got the current for sure */

        /* Check if curr->nid is same as nid */
        if ((lock->slice_valid && lock->slice == curr->slice) ||
            (curr->nid == nid)) {
            if ((curr->slice < prev->slice) || (prev->nid == nid)) {
#ifdef USE_COUNTER
                set_waitcount(curr, ++curr_locked_count);
#else
                set_waitcount(curr, curr_locked_count);
#endif

                last        = curr;
                prev        = curr;
                one_shuffle = 1;
            } else {
                next = READ_ONCE(curr->next);
                if (!next) {
                    sleader = last;
                    qend    = prev;
                    goto out;
                }

#ifdef USE_COUNTER
                set_waitcount(curr, ++curr_locked_count);
#else
                set_waitcount(curr, curr_locked_count);
#endif

                prev->next  = next;
                curr->next  = last->next;
                last->next  = curr;
                last        = curr;
                one_shuffle = 1;
            }
        } else
            prev = curr;

        lock_ready = !READ_ONCE(lock->locked);
        if (one_shuffle && ((is_next_waiter && lock_ready) ||
                            (!is_next_waiter && READ_ONCE(node->lstatus)))) {
            sleader = last;
            qend    = prev;
            break;
        }
    }

out:
#ifdef WAITER_CORRECTNESS
    smp_swap(&lock->slocked, 0);
    WRITE_ONCE(lock->shuffler, NULL);
#endif

    dprintf("time to go out for me (%d)!\n", node->cid);
    if (sleader) {
        /* WRITE_ONCE(sleader->sleader, 1); */
        set_sleader(sleader, qend);
    }
}

static aqsscl_thread_info_t *thread_info_create(aqsscl_mutex_t *lock,
                                                int weight) {
    aqsscl_thread_info_t *info =
        alloc_cache_align(sizeof(aqsscl_thread_info_t));
    if (!info) {
        return NULL;
    }

    info->banned_until = rdtsc();
    if (weight == 0) {
        // identifies the nice value of the new thread
        int prio = getpriority(PRIO_PROCESS, 0);
        // convert to weights
        weight = prio_to_weight[prio + 20];
    }
    info->weight = weight;
    __sync_add_and_fetch(&lock->total_weight, weight);
    info->banned      = 0;
    info->slice       = 0;
    info->start_ticks = 0;

    return info;
}

static inline void aqsscl_mutex_unlock_stats(aqsscl_mutex_t *lock) {
    aqsscl_thread_info_t *thread_info =
        (aqsscl_thread_info_t *)pthread_getspecific(lock->thread_info_key);
    uint64_t now = rdtsc(), cs = now - thread_info->start_ticks;
    thread_info->banned_until +=
        cs * (__atomic_load_n(&lock->total_weight, __ATOMIC_RELAXED) /
              thread_info->weight);
}

static void
aqsscl_mutex_lock_update_slice_info(aqsscl_mutex_t *lock,
                                    aqsscl_thread_info_t *thread_info,
                                    aqsscl_node_t *node) {
    const uint64_t now       = rdtsc();
    thread_info->start_ticks = now;
    thread_info->slice       = now + FAIRLOCK_GRANULARITY;
    lock->slice              = thread_info->slice;
    node->slice              = thread_info->slice;
    lock->slice_valid        = 1;
}

static aqsscl_thread_info_t *aqsscl_thread_info(aqsscl_mutex_t *lock) {
    // Retrieve per-thread tracking information
    aqsscl_thread_info_t *info =
        (aqsscl_thread_info_t *)pthread_getspecific(lock->thread_info_key);
    if (!info) {
        info = thread_info_create(lock, 0);
        pthread_setspecific(lock->thread_info_key, info);
    }
    return info;
}

/* Interpose */
aqsscl_mutex_t *aqsscl_mutex_create(const pthread_mutexattr_t *attr) {
    aqsscl_mutex_t *impl =
        (aqsscl_mutex_t *)alloc_cache_align(sizeof(aqsscl_mutex_t));
    impl->tail = NULL;
    impl->val  = 0;
#ifdef WAITER_CORRECTNESS
    impl->slocked = 0;
#endif
#if COND_VAR
    REAL(pthread_mutex_init)(&impl->posix_lock, attr);
    DEBUG("Mutex init lock=%p posix_lock=%p\n", impl, &impl->posix_lock);
#endif

    impl->slice        = 0;
    impl->total_weight = 0;
    impl->slice_valid  = 0;

    if (pthread_key_create(&impl->thread_info_key, NULL)) {
        return NULL;
    }

    barrier();
    return impl;
}

static int __aqsscl_mutex_lock(aqsscl_mutex_t *lock, aqsscl_node_t *node) {
    aqsscl_thread_info_t *thread_info = aqsscl_thread_info(lock);

    while (rdtsc() < thread_info->banned_until)
        ;

    if (smp_cas(&lock->locked_no_stealing, 0, 1) == 0) {
        goto release;
    }

    node->slice        = thread_info->slice;
    node->cid          = cur_thread_id;
    node->next         = NULL;
    node->locked       = AQSSCL_STATUS_WAIT;
    node->nid          = current_numa_node();
    node->last_visited = NULL;

    /*
     * Publish the updated tail.
     */
    aqsscl_node_t *prev = smp_swap(&lock->tail, node);

    if (prev) {

        WRITE_ONCE(prev->next, node);

        /*
         * Now, we are waiting for the lock holder to
         * allow us to become the very next lock waiter.
         * In the meantime, we also check whether the node
         * is the shuffle leader, if that's the case,
         * then it goes on shuffling waiters in its socket
         */
        for (;;) {

            if (READ_ONCE(node->lstatus) == AQSSCL_STATUS_LOCKED)
                break;

            if (READ_ONCE(node->sleader)) {
                shuffle_waiters(lock, node, 0);
            }

            CPU_PAUSE();
        }
    } else
        disable_stealing(lock);

    /*
     * we are now the very next waiters, all we have to do is
     * to wait for the @lock->locked to become 0, i.e. unlocked.
     * In the meantime, we will try to be shuffle leader if possible
     * and at least find someone in my socket.
     */
    for (;;) {
        int wcount;

        // wait until the current slice expires
        if (rdtsc() >= lock->slice) {
            if (!READ_ONCE(lock->locked)) {
                break;
            }
        }

        /*
         * There are two ways to become a shuffle leader:
         * 1) my @node->wcount is 0
         * 2) someone or myself (earlier) appointed me
         * (@node->sleader = 1)
         */
        wcount = node->wcount;
        if (!wcount || (wcount && node->sleader)) {
            shuffle_waiters(lock, node, 1);
        }

        CPU_PAUSE();
    }

    /*
     * The biggest catch with our algorithm is that it allows
     * stealing in the fast path.
     * Thus, even if the @lock->locked was 0 above, it doesn't
     * mean that we have the lock. So, we acquire the lock
     * in two ways:
     * 1) Either someone disabled the lock stealing before us
     * that allows us to directly set the lock->locked value 1
     * 2) Or, I will explicitly try to do a cmpxchg operation
     * on the @lock->locked variable. If I am unsuccessful for
     * @impatient_cap times, then I explicitly lock stealing,
     * this is to ensure starvation freedom, and will wait
     * for the lock->locked status to change to 0.
     */
    for (;;) {
        /*
         * If someone has already disable stealing,
         * change locked and proceed forward
         */
        if (smp_cas(&lock->locked, 0, 1) == 0)
            break;

        while (READ_ONCE(lock->locked))
            CPU_PAUSE();
    }

    if (!READ_ONCE(node->next)) {
        if (smp_cas(&lock->tail, node, NULL) == node) {
            enable_stealing(lock);
            goto release;
        }

        while (!READ_ONCE(node->next))
            CPU_PAUSE();
    }

    WRITE_ONCE(node->next->lstatus, 1);

release:
    aqsscl_mutex_lock_update_slice_info(lock, thread_info, node);

    return 0;
}

int aqsscl_mutex_lock(aqsscl_mutex_t *impl, aqsscl_node_t *UNUSED(me)) {
    aqsscl_node_t node = {0};
    int ret            = __aqsscl_mutex_lock(impl, &node);
    assert(ret == 0);
#if COND_VAR
    if (ret == 0) {
        DEBUG_PTHREAD("[%d] Lock posix=%p\n", cur_thread_id, &impl->posix_lock);
        assert(REAL(pthread_mutex_lock)(&impl->posix_lock) == 0);
    }
#endif
    DEBUG("[%d] Lock acquired posix=%p\n", cur_thread_id, &impl->posix_lock);
    return ret;
}

int aqsscl_mutex_trylock(aqsscl_mutex_t *impl, aqsscl_node_t *UNUSED(me)) {

    if ((smp_cas(&impl->locked, 0, 1) == 0)) {
#if COND_VAR
        DEBUG_PTHREAD("[%d] Lock posix=%p\n", cur_thread_id, &impl->posix_lock);
        int ret = 0;
        while ((ret = REAL(pthread_mutex_trylock)(&impl->posix_lock)) == EBUSY)
            ;
        assert(ret == 0);
#endif
        return 0;
    }
    return EBUSY;
}

static inline void __aqsscl_mutex_unlock(aqsscl_mutex_t *lock) {
    WRITE_ONCE(lock->locked, 0);
    aqsscl_mutex_unlock_stats(lock);
}

void aqsscl_mutex_unlock(aqsscl_mutex_t *impl, aqsscl_node_t *UNUSED(me)) {
#if COND_VAR
    DEBUG_PTHREAD("[%d] Unlock posix=%p\n", cur_thread_id, &impl->posix_lock);
    assert(REAL(pthread_mutex_unlock)(&impl->posix_lock) == 0);
#endif
    __aqsscl_mutex_unlock(impl);
}

int aqsscl_mutex_destroy(aqsscl_mutex_t *lock) {
#if COND_VAR
    REAL(pthread_mutex_destroy)(&lock->posix_lock);
#endif
    free(lock);
    lock = NULL;

    return 0;
}

int aqsscl_cond_init(aqsscl_cond_t *cond, const pthread_condattr_t *attr) {
#if COND_VAR
    return REAL(pthread_cond_init)(cond, attr);
#else
    fprintf(stderr, "Error cond_var not supported.");
    assert(0);
#endif
}

int aqsscl_cond_timedwait(aqsscl_cond_t *cond, aqsscl_mutex_t *lock,
                          aqsscl_node_t *UNUSED(me),
                          const struct timespec *ts) {
#if COND_VAR
    int res;
    aqsscl_node_t node;

    __aqsscl_mutex_unlock(lock);
    DEBUG("[%d] Sleep cond=%p lock=%p posix_lock=%p\n", cur_thread_id, cond,
          lock, &(lock->posix_lock));
    DEBUG_PTHREAD("[%d] Cond posix = %p lock = %p\n", cur_thread_id, cond,
                  &lock->posix_lock);

    if (ts)
        res = REAL(pthread_cond_timedwait)(cond, &lock->posix_lock, ts);
    else
        res = REAL(pthread_cond_wait)(cond, &lock->posix_lock);

    if (res != 0 && res != ETIMEDOUT) {
        fprintf(stderr, "Error on cond_{timed,}wait %d\n", res);
        assert(0);
    }

    int ret = 0;
    if ((ret = REAL(pthread_mutex_unlock)(&lock->posix_lock)) != 0) {
        fprintf(stderr, "Error on mutex_unlock %d\n", ret == EPERM);
        assert(0);
    }

    aqsscl_mutex_lock(lock, &node);

    return res;
#else
    fprintf(stderr, "Error cond_var not supported.");
    assert(0);
#endif
}

int aqsscl_cond_wait(aqsscl_cond_t *cond, aqsscl_mutex_t *lock,
                     aqsscl_node_t *me) {
    return aqsscl_cond_timedwait(cond, lock, me, 0);
}

int aqsscl_cond_signal(aqsscl_cond_t *cond) {
#if COND_VAR
    return REAL(pthread_cond_signal)(cond);
#else
    fprintf(stderr, "Error cond_var not supported.");
    assert(0);
#endif
}

int aqsscl_cond_broadcast(aqsscl_cond_t *cond) {
#if COND_VAR
    DEBUG("[%d] Broadcast cond=%p\n", cur_thread_id, cond);
    return REAL(pthread_cond_broadcast)(cond);
#else
    fprintf(stderr, "Error cond_var not supported.");
    assert(0);
#endif
}

int aqsscl_cond_destroy(aqsscl_cond_t *cond) {
#if COND_VAR
    return REAL(pthread_cond_destroy)(cond);
#else
    fprintf(stderr, "Error cond_var not supported.");
    assert(0);
#endif
}

void aqsscl_thread_start(void) {
}

void aqsscl_thread_exit(void) {
}

void aqsscl_application_init(void) {
}

void aqsscl_application_exit(void) {
}
void aqsscl_init_context(lock_mutex_t *UNUSED(impl),
                         lock_context_t *UNUSED(context), int UNUSED(number)) {
}
