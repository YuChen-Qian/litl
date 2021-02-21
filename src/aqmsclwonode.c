#include <aqmsclwonode.h>
#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/time.h>

#include "interpose.h"
#include "utils.h"
#include "waiting_policy.h"

#include "padding.h"

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

/* debugging */
#ifdef WAITER_DEBUG
static inline void traverse_nodes(aqmscl_mutex_t *lock, aqmscl_node_t *node) {
    aqmscl_node_t *curr = node;
    int count           = 0;
    printf("my prev: ");
    if (node->prev_shuffler)
        printf("%d\n", node->prev_shuffler->cid);
    else
        printf("no one!\n");
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
#endif

static inline void print_node_state(const char *str, struct aqmscl_node *node) {
    return;
    switch (node->lstatus) {
    case _AQ_MCS_STATUS_PWAIT:
        printf("[%s] node(%d)->lstatus: PWAIT\n", str, node->cid);
        break;
    case _AQ_MCS_STATUS_PARKED:
        printf("[%s] node(%d)->lstatus: PARKED\n", str, node->cid);
        break;
    case _AQ_MCS_STATUS_LOCKED:
        printf("[%s] node(%d)->lstatus: LOCKED\n", str, node->cid);
        break;
    case _AQ_MCS_STATUS_UNPWAIT:
        printf("[%s] node(%d)->lstatus: UNPWAIT\n", str, node->cid);
        break;
    }
}

extern __thread unsigned int cur_thread_id;

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
}

static int keep_lock_local(void) {
    return xor_random() & THRESHOLD;
}

static inline int current_numa_node() {
    unsigned long a, d, c;
    int core;
    __asm__ volatile("rdtscp" : "=a"(a), "=d"(d), "=c"(c));
    core = c & 0xFFF;
    return core / (CPU_NUMBER / NUMA_NODES);
}

static inline void enable_stealing(aqmscl_mutex_t *lock) {
    smp_swap(&lock->no_stealing, 1);
}

static inline void disable_stealing(aqmscl_mutex_t *lock) {
    smp_swap(&lock->no_stealing, 0);
}

static inline int is_stealing_disabled(aqmscl_mutex_t *lock) {
    return READ_ONCE(lock->no_stealing);
}

static inline void __waiting_policy_wake(volatile int *var) {
    *var    = 1;
    int ret = sys_futex((int *)var, FUTEX_WAKE_PRIVATE, UNLOCKED, NULL, 0, 0);
    if (ret == -1) {
        perror("Unable to futex wake");
        exit(-1);
    }
}

static inline void __waiting_policy_sleep(volatile int *var) {
    if (*var == 1)
        return;

    int ret = 0;
    while ((ret = sys_futex((int *)var, FUTEX_WAIT_PRIVATE, LOCKED, NULL, 0,
                            0)) != 0) {
        if (ret == -1 && errno != EINTR) {
            /**
             * futex returns EAGAIN if *var is not 0 anymore.
             * This can happen when the value of *var is changed by another
             * thread after the spinning loop.
             * Note: FUTEX_WAIT_PRIVATE acts like an atomic operation.
             **/
            if (errno == EAGAIN) {
                DEBUG("[-1] Race\n");
                break;
            }
            perror("Unable to futex wait");
            exit(-1);
        }
    }

    /**
     * *var is not always 1 immediately when the thread wakes up
     * (but eventually it is).
     * Maybe related to memory reordering?
     **/
    while (*var != 1)
        CPU_PAUSE();
}

static inline void __wakeup_waiter(aqmscl_node_t *node) {
    __waiting_policy_wake((volatile int *)&node->pstate);
}

static inline int force_update_node(aqmscl_node_t *node, uint8_t state) {
    if ((smp_cas(&node->lstatus, _AQ_MCS_STATUS_PARKED, state) ==
         _AQ_MCS_STATUS_PARKED)) { // ||
        /*
         * Ouch, we need to explicitly wake up the guy
         */
        __wakeup_waiter(node);
        return true;
    }
    return false;
}

static inline int park_waiter(struct aqmscl_node *node) {
    if (smp_cas(&node->lstatus, _AQ_MCS_STATUS_PWAIT, _AQ_MCS_STATUS_PARKED) !=
        _AQ_MCS_STATUS_PWAIT)
        goto out_acquired;

    __waiting_policy_sleep((volatile int *)&node->pstate);

out_acquired:
    return 1;
}

static aqmscl_thread_info_t *thread_info_create(aqmscl_mutex_t *lock,
                                                int weight) {
    aqmscl_thread_info_t *info =
        alloc_cache_align(sizeof(aqmscl_thread_info_t));
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
#ifdef AQMSCL_STAT
    memset(&info->stat, 0, sizeof(aqmscl_stats_t));
    info->stat.start = info->banned_until;
#endif
    return info;
}

static aqmscl_thread_info_t *aqmscl_thread_info(aqmscl_mutex_t *lock) {
    // Retrieve per-thread tracking information
    aqmscl_thread_info_t *info =
        (aqmscl_thread_info_t *)pthread_getspecific(lock->thread_info_key);
    if (!info) {
        info = thread_info_create(lock, 0);
        pthread_setspecific(lock->thread_info_key, info);
    }
    return info;
}

static void aqmscl_mutex_unlock_stats(aqmscl_mutex_t *lock) {
    aqmscl_thread_info_t *thread_info =
        (aqmscl_thread_info_t *)pthread_getspecific(lock->thread_info_key);
    uint64_t now = rdtsc(), cs = now - thread_info->start_ticks;
    thread_info->banned_until +=
        cs * (__atomic_load_n(&lock->total_weight, __ATOMIC_RELAXED) /
              thread_info->weight);
    thread_info->banned = now < thread_info->banned_until;
}

void shuffle_waiters(aqmscl_mutex_t *lock, aqmscl_node_t *node,
                     int is_next_waiter) {
    aqmscl_node_t *curr, *prev, *next, *last, *sleader;
    int nid               = node->nid;
    int curr_locked_count = node->wcount;
    int one_shuffle       = 0;
    uint32_t lock_ready;

    sleader = NULL;
    prev    = node;
    last    = node;
    curr    = NULL;
    next    = NULL;

    dprintf("node (%d) with sleader (%d), wcount (%d) and lock->slocked: %d\n",
            node->cid, node->sleader, node->wcount, READ_ONCE(lock->slocked));

#ifdef WAITER_CORRECTNESS
    if (smp_cas(&lock->slocked, 0, 1) != 0) {
        traverse_nodes(lock, node);
        assert(0);
        return;
    } else {
        WRITE_ONCE(lock->shuffler, node);
    }
#endif

    if (curr_locked_count == 0) {
        // lstat_inc(lock_num_shuffles);
        WRITE_ONCE(node->wcount, ++curr_locked_count);
        dprintf("increaing wcount of %d to %d\n", node->cid, curr_locked_count);
    }

    dprintf("clearing node %d sleader value\n", node->cid);
    WRITE_ONCE(node->sleader, 0);

    /* if (curr_locked_count >= _AQ_MAX_LOCK_COUNT) { */
    /*     sleader = READ_ONCE(node->next); */
    /*     dprintf("1. selecting new shuffler %d\n", sleader->cid); */
    /*     goto out; */
    /* } */

    if (!keep_lock_local()) {
        sleader = READ_ONCE(node->next);
        goto out;
    }

    for (;;) { // traverse the linked list
        curr = READ_ONCE(prev->next);

        if (!curr) {
            sleader = last;
            break;
        }

        if (curr == READ_ONCE(lock->tail)) {
            sleader = last;
            break;
        }

        /* got the current for sure */
        if ((lock->slice_valid && lock->slice == curr->slice) ||
            (curr->nid == nid)) {
            if ((curr->slice < prev->slice) || (prev->nid == nid)) {
                print_node_state("before", curr);
                force_update_node(curr, _AQ_MCS_STATUS_UNPWAIT);
                print_node_state("after", curr);
                WRITE_ONCE(curr->wcount, curr_locked_count);
                last        = curr;
                prev        = curr;
                one_shuffle = 1;
            } else { // otherwise waiters exist between curr and last
                next = READ_ONCE(curr->next);
                if (!next) {
                    sleader = last;
                    goto out;
                }

                print_node_state("start marking shuffled", curr);
                print_node_state("before", curr);
                force_update_node(curr, _AQ_MCS_STATUS_UNPWAIT);
                print_node_state("after", curr);
                print_node_state("end marking shuffled", curr);
                WRITE_ONCE(curr->wcount, curr_locked_count);
                prev->next  = next;
                curr->next  = last->next;
                last->next  = curr;
                last        = curr;
                one_shuffle = 1;
            }
        }

        lock_ready = !READ_ONCE(lock->locked);
        if (!next || (one_shuffle && is_next_waiter && lock_ready)) {
            sleader = last;
            break;
        }
    }

out:
#ifdef WAITER_CORRECTNESS
    smp_swap(&lock->slocked, 0);
    WRITE_ONCE(lock->shuffler, NULL);
#endif

    DEBUG("time to go out for me (%d)!\n", node->cid);
    if (sleader) {
        WRITE_ONCE(sleader->sleader, 1);
    }
}

aqmscl_mutex_t *aqmscl_mutex_create(const pthread_mutexattr_t *attr) {
    aqmscl_mutex_t *lock =
        (aqmscl_mutex_t *)alloc_cache_align(sizeof(aqmscl_mutex_t));
    if (!lock) {
        return NULL;
    }

    lock->tail = NULL;
    lock->next = NULL;
    lock->val  = 0;
#ifdef WAITER_CORRECTNESS
    lock->slocked = 0;
#endif

    lock->total_weight = 0;
    lock->slice        = 0;
    lock->slice_valid  = 0;
    if (pthread_key_create(&lock->thread_info_key, NULL)) {
        return NULL;
    }

    barrier();
    return lock;
}

static void
aqmscl_mutex_lock_update_slice_info(aqmscl_mutex_t *lock,
                                    aqmscl_thread_info_t *thread_info,
                                    aqmscl_node_t *node) {
    const uint64_t now       = rdtsc();
    thread_info->start_ticks = now;
    thread_info->slice       = now + FAIRLOCK_GRANULARITY;
    lock->slice              = thread_info->slice;
    node->slice              = thread_info->slice;
    lock->slice_valid        = 1;
}

static int aqmscl_mutex_lock_impl(aqmscl_mutex_t *lock, aqmscl_node_t *node) {
    aqmscl_thread_info_t *thread_info = aqmscl_thread_info(lock);

    while (rdtsc() < thread_info->banned_until)
        ;

    if (smp_cas(&lock->locked_no_stealing, 0, 1) == 0) {
        dprintf("acquired in fastpath\n");
        goto release;
    }

    DEBUG("acquiring in the slowpath\n");

    node->slice        = thread_info->slice;
    node->cid          = cur_thread_id;
    node->next         = NULL;
    node->last_visited = NULL;
    node->locked       = _AQ_MCS_STATUS_PWAIT;
    node->nid          = current_numa_node();
    node->pstate       = 0;

    aqmscl_node_t *pred = smp_swap(&lock->tail, node);
    aqmscl_node_t *succ = NULL;

    if (pred) {
        int i;
        int should_park      = false;
        int very_next_waiter = false;

        DEBUG("my pred is %d\n", pred->cid);

        WRITE_ONCE(pred->next, node);

    retry:
        for (i = 0; i < SPINNING_THRESHOLD; ++i) {
            uint32_t val = READ_ONCE(node->locked);

            if (_AQ_MCS_LOCKED_VAL(val) == _AQ_MCS_STATUS_LOCKED) {
                very_next_waiter = true;
                break;
            }

            if (_AQ_MCS_SLEADER_VAL(val) == 1) {
                DEBUG("shuffle waiter (%d) NOT very next waiter\n", node->cid);
                // lstat_inc(lock_waiter_shuffler);
                should_park = true;
                shuffle_waiters(lock, node, 0);
            }
            CPU_PAUSE();
        }

        if (!should_park) {
            park_waiter(node);
        }

        if (!very_next_waiter)
            goto retry;

        DEBUG("I am the very next lock waiter\n");
        for (;;) {
            uint32_t val = READ_ONCE(node->locked);

            if (!READ_ONCE(lock->locked))
                break;

            if (!_AQ_MCS_WCOUNT_VAL(val) ||
                (_AQ_MCS_WCOUNT_VAL(val) && _AQ_MCS_SLEADER_VAL(val))) {
                DEBUG("shuffle waiter (%d) is the very next lock waiter\n",
                      node->cid);
                shuffle_waiters(lock, node, 1);
            }
        }
    }

    DEBUG("waiting for the lock to be released\n");
    for (;;) {

        if (is_stealing_disabled(lock))
            disable_stealing(lock);

        while (READ_ONCE(lock->locked))
            CPU_PAUSE();

        if (smp_cas(&lock->locked, 0, 1) == 0)
            break;

        CPU_PAUSE();
    }

    DEBUG("lock acquired\n");

    succ = READ_ONCE(node->next);
    if (!succ) {
        if (smp_cas(&lock->tail, node, NULL) == node) {
            enable_stealing(lock);
            goto release;
        }

        for (;;) {
            succ = READ_ONCE(node->next);
            if (succ)
                break;
            CPU_PAUSE();
        }
    }

    DEBUG("notifying the very next waiter (%d) to be ready\n", succ->cid);
    if (smp_swap(&succ->lstatus, _AQ_MCS_STATUS_LOCKED) ==
        _AQ_MCS_STATUS_PARKED) {
        __wakeup_waiter(succ);
    }

release:
    aqmscl_mutex_lock_update_slice_info(lock, thread_info, node);

    return 0;
}

int aqmscl_mutex_lock(aqmscl_mutex_t *lock, aqmscl_node_t *UNUSED(me)) {
    DEBUG("acquiring the lock\n");
    aqmscl_node_t node;
    const int ret = aqmscl_mutex_lock_impl(lock, &node);
    assert(ret == 0);
    DEBUG("[%d] Lock acquired posix=%p\n", cur_thread_id, &lock->posix_lock);
    return ret;
}

int aqmscl_mutex_trylock(aqmscl_mutex_t *lock, aqmscl_node_t *me) {
    if (smp_cas(&lock->val, 0, 1) == 0) {
#if COND_VAR
        DEBUG_PTHREAD("[%d] Lock posix=%p\n", cur_thread_id, &lock->posix_lock);
        int ret = 0;
        while ((ret = REAL(pthread_mutex_trylock)(&lock->posix_lock)) == EBUSY)
            ;
        assert(ret == 0);
#endif
        return 0;
    }
    return EBUSY;
}

static void aqmscl_mutex_unlock_impl(aqmscl_mutex_t *lock) {
    DEBUG("releasing the lock\n");

    WRITE_ONCE(lock->locked, 0);

    aqmscl_mutex_unlock_stats(lock);
}

void aqmscl_mutex_unlock(aqmscl_mutex_t *lock, aqmscl_node_t *UNUSED(me)) {
    return aqmscl_mutex_unlock_impl(lock);
}

int aqmscl_mutex_destroy(aqmscl_mutex_t *lock) {
    pthread_key_delete(lock->thread_info_key);
    free(lock);
    lock = NULL;
    return 0;
}

int aqmscl_cond_init(aqmscl_cond_t *cond, const pthread_condattr_t *attr) {
#if COND_VAR
    return REAL(pthread_cond_init)(cond, attr);
#else
    fprintf(stderr, "Error cond_var not supported.");
    assert(0);
#endif
}

int aqmscl_cond_timedwait(aqmscl_cond_t *cond, aqmscl_mutex_t *lock,
                          aqmscl_node_t *UNUSED(me),
                          const struct timespec *ts) {
#if COND_VAR
    int res;
    aqmscl_node_t node;

    aqmscl_mutex_unlock_impl(lock);
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

    aqmscl_mutex_lock(lock, &node);

    return res;
#else
    fprintf(stderr, "Error cond_var not supported.");
    assert(0);
#endif
}

int aqmscl_cond_wait(aqmscl_cond_t *cond, aqmscl_mutex_t *lock,
                     aqmscl_node_t *me) {
    return aqmscl_cond_timedwait(cond, lock, me, 0);
}

int aqmscl_cond_signal(aqmscl_cond_t *cond) {
#if COND_VAR
    return REAL(pthread_cond_signal)(cond);
#else
    fprintf(stderr, "Error cond_var not supported.");
    assert(0);
#endif
}

int aqmscl_cond_broadcast(aqmscl_cond_t *cond) {
#if COND_VAR
    DEBUG("[%d] Broadcast cond=%p\n", cur_thread_id, cond);
    return REAL(pthread_cond_broadcast)(cond);
#else
    fprintf(stderr, "Error cond_var not supported.");
    assert(0);
#endif
}

int aqmscl_cond_destroy(aqmscl_cond_t *cond) {
#if COND_VAR
    return REAL(pthread_cond_destroy)(cond);
#else
    fprintf(stderr, "Error cond_var not supported.");
    assert(0);
#endif
}

void aqmscl_thread_start(void) {
}

void aqmscl_thread_exit(void) {
}

void aqmscl_application_init(void) {
}

void aqmscl_application_exit(void) {
}

void aqmscl_init_context(lock_mutex_t *UNUSED(lock),
                         lock_context_t *UNUSED(context), int UNUSED(number)) {
}
