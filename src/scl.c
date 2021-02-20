#include "utils.h"
#include "waiting_policy.h"
#include <errno.h>
#include <linux/futex.h>
#include <scl.h>
#include <stdint.h>
#include <sys/syscall.h>
#include <sys/time.h>
#include <unistd.h>

#define spin_then_yield(limit, expr)                                           \
    while (1) {                                                                \
        int val, counter = 0;                                                  \
        while ((val = (expr)) && counter++ < limit)                            \
            ;                                                                  \
        if (!val)                                                              \
            break;                                                             \
        sched_yield();                                                         \
    }

#ifndef CYCLE_PER_US
#error Must define CYCLE_PER_US for the current machine in Makefile or elsewhere
#endif
#define CYCLE_PER_MS (CYCLE_PER_US * 1000L)
#define CYCLE_PER_S (CYCLE_PER_MS * 1000L)
#ifndef FAIRLOCK_GRANULARITY
#error Must define FAIRLOCK_GRANULARITY for the current machine in Makefile or elsewhere
#endif

static const unsigned long long spin_limit        = 20;
static const unsigned long long sleep_granularity = 8;
static const unsigned long long sleep_time = CYCLE_PER_US * sleep_granularity;

// Same as the sched_prio_to_weight in Linux kernel
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

static inline int futex(int *uaddr, int futex_op, int val,
                        const struct timespec *timeout) {
    return syscall(SYS_futex, uaddr, futex_op, val, timeout, NULL, 0);
}

scl_mutex_t *scl_mutex_create(const pthread_mutexattr_t *attr) {
    scl_mutex_t *lock = (scl_mutex_t *)alloc_cache_align(sizeof(scl_mutex_t));
    if (!lock) {
        return NULL;
    }

    lock->tail         = NULL;
    lock->next         = NULL;
    lock->total_weight = 0;
    lock->slice        = 0;
    lock->slice_valid  = 0;
    if (pthread_key_create(&lock->tinfo_key, NULL)) {
        return NULL;
    }

    return lock;
}

static tinfo_t *tinfo_create(scl_mutex_t *lock, int weight) {
    tinfo_t *info = alloc_cache_align(sizeof(tinfo_t));
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
#ifdef DEBUG
    memset(&info->stat, 0, sizeof(stats_t));
    info->stat.start = info->banned_until;
#endif
    return info;
}

static tinfo_t *__thread_info(scl_mutex_t *lock) {
    // Retrieve per-thread tracking information
    tinfo_t *info = (tinfo_t *)pthread_getspecific(lock->tinfo_key);
    if (!info) {
        info = tinfo_create(lock, 0);
        pthread_setspecific(lock->tinfo_key, info);
    }
    return info;
}

static void __scl_mutex_lock_reenter(struct tinfo *info,
                                     unsigned long long now) {
#ifdef DEBUG
    info->stat.reenter++;
#endif
    info->start_ticks = now;
}

static int __scl_mutex_lock(scl_mutex_t *lock, tinfo_t *info) {
    unsigned long long now;

    if (info->banned) {
        if ((now = rdtsc()) < info->banned_until) {
            unsigned long long banned_time = info->banned_until - now;
#ifdef DEBUG
            info->stat.banned_time += banned_time;
#endif
            // sleep with granularity of sleep_granularity us
            while (banned_time > sleep_time) {
                struct timespec req = {
                    .tv_sec  = banned_time / CYCLE_PER_S,
                    .tv_nsec = (banned_time % CYCLE_PER_S / CYCLE_PER_US /
                                sleep_granularity) *
                               sleep_granularity * 1000,
                };
                nanosleep(&req, NULL);
                if ((now = rdtsc()) >= info->banned_until)
                    break;
                banned_time = info->banned_until - now;
            }
            // spin for the remaining (<sleep_granularity us)
            spin_then_yield(spin_limit, (now = rdtsc()) < info->banned_until);
        }
    }

    scl_node_t cur_qnode = {0};
    while (1) {
        scl_node_t *prev_qnode = lock->tail;
        if (!__sync_bool_compare_and_swap(&lock->tail, prev_qnode,
                                          &cur_qnode)) {
            continue; // lock->tail changed... try again
        }
        // cur_qnode entered the lock queue, complete the lock chain
        if (!prev_qnode) { // no more waiting threads
            cur_qnode.state = RUNNABLE;
            lock->next      = &cur_qnode;
        } else if (prev_qnode == (scl_node_t *)lock) { // next runnable
            cur_qnode.state  = NEXT;
            prev_qnode->next = &cur_qnode;
        } else { // waiters
            prev_qnode->next = &cur_qnode;
            // wait until we become the next runnable
#ifdef DEBUG
            now = rdtsc();
#endif
            // park till cur_qnode is ready to run
            do {
                futex((int *)&cur_qnode.state, FUTEX_WAIT_PRIVATE, INIT, NULL);
            } while (INIT == cur_qnode.state);
#ifdef DEBUG
            info->stat.next_runnable_wait += rdtsc() - now;
#endif
        }
        // invariant: cur_qnode.state >= NEXT

        // wait until the current slice expires
        int slice_valid;
        unsigned long long curr_slice;
        while ((slice_valid = lock->slice_valid) &&
               (now = rdtsc()) + sleep_granularity <
                   (curr_slice = lock->slice)) {
            unsigned long long slice_left = curr_slice - now;
            struct timespec timeout       = {
                .tv_sec  = 0, // slice will be less then 1 sec
                .tv_nsec = (slice_left / (CYCLE_PER_US * sleep_granularity)) *
                           sleep_granularity * 1000,
            };
            futex((int *)&lock->slice_valid, FUTEX_WAIT_PRIVATE, 0, &timeout);
#ifdef DEBUG
            info->stat.prev_slice_wait += rdtsc() - now;
#endif
        }

        if (slice_valid) {
            spin_then_yield(spin_limit, (slice_valid = lock->slice_valid) &&
                                            (rdtsc() < lock->slice));
            lock->slice_valid = 0;
        }
        // invariant: rdtsc() >= curr_slice && lock->slice_valid == 0

#ifdef DEBUG
        now = rdtsc();
#endif
        // spin until RUNNABLE and try to grab the lock
        spin_then_yield(spin_limit,
                        RUNNABLE != cur_qnode.state ||
                            0 == __sync_bool_compare_and_swap(
                                     &cur_qnode.state, RUNNABLE, RUNNING));
        // invariant: cur_qnode.state == RUNNING
#ifdef DEBUG
        info->stat.runnable_wait += rdtsc() - now;
#endif

        // record the successor in the lock so we can notify it when we release
        scl_node_t *next_qnode = cur_qnode.next;
        if (!next_qnode) {
            lock->next = NULL;
            if (!__sync_bool_compare_and_swap(&lock->tail, &cur_qnode, lock)) {
                spin_then_yield(spin_limit, !cur_qnode.next);
#ifdef DEBUG
                info->stat.succ_wait += rdtsc() - now;
#endif
                next_qnode = cur_qnode.next;
                lock->next = next_qnode;
            }
        } else {
            lock->next = next_qnode;
        }

        // invariant: NULL == next_qnode <=> lock->qtail == flqnode(lock)
        now               = rdtsc();
        info->start_ticks = now;
        info->slice       = now + FAIRLOCK_GRANULARITY;
        lock->slice       = info->slice;
        lock->slice_valid = 1;

        // wake up successor if necessary
        if (next_qnode) {
            next_qnode->state = NEXT;
            futex((int *)&next_qnode->state, FUTEX_WAKE_PRIVATE, 1, NULL);
        }

        return 0;
    }
}

int scl_mutex_lock(scl_mutex_t *lock, scl_node_t *UNUSED(me)) {
    unsigned long long now;

    tinfo_t *info = __thread_info(lock);

    if (lock->slice_valid) {
        unsigned long long curr_slice = lock->slice;
        // If owner of current slice, try to reenter at the beginning of the
        // queue
        if (curr_slice == info->slice && (now = rdtsc()) < curr_slice) {
            scl_node_t *succ = lock->next;
            if (!succ) { // next waiting thread is empty
                if (__sync_bool_compare_and_swap(&lock->tail, NULL, lock)) {
                    // lock is not held, reenter
                    __scl_mutex_lock_reenter(info, now);
                    return 0;
                }
                spin_then_yield(spin_limit, (now = rdtsc()) < curr_slice &&
                                                NULL == (succ = lock->next));
#ifdef DEBUG
                info->stat.own_slice_wait += rdtsc() - now;
#endif
                // let the succ invalidate the slice, and don't need to wake it
                // up because slice expires naturally
                if (now >= curr_slice) {
                    return __scl_mutex_lock(lock, info);
                }
            }
            // if state < RUNNABLE, it won't become RUNNABLE unless someone
            // releases lock, but as no one is holding the lock, there is no
            // race
            if (succ->state < RUNNABLE ||
                __sync_bool_compare_and_swap(&succ->state, RUNNABLE, NEXT)) {
                __scl_mutex_lock_reenter(info, now);
                return 0;
            }
        }
    }
    return __scl_mutex_lock(lock, info);
}

static void
__scl_mutex_unlock_accounting(scl_mutex_t *lock,
                              unsigned long long release_succ_wait) {
    unsigned long long now, cs;
    tinfo_t *info;

    // invariant: NULL == succ || succ->state = RUNNABLE
    info = (tinfo_t *)pthread_getspecific(lock->tinfo_key);
    now  = rdtsc();
    cs   = now - info->start_ticks;
    info->banned_until +=
        cs *
        (__atomic_load_n(&lock->total_weight, __ATOMIC_RELAXED) / info->weight);
    info->banned = now < info->banned_until;

    if (info->banned) {
        if (__sync_bool_compare_and_swap(&lock->slice_valid, 1, 0)) {
            futex((int *)&lock->slice_valid, FUTEX_WAKE_PRIVATE, 1, NULL);
        }
    }
#ifdef DEBUG
    info->stat.release_succ_wait += release_succ_wait;
#endif
}

void scl_mutex_unlock(scl_mutex_t *lock, scl_node_t *UNUSED(me)) {
#ifdef DEBUG
    unsigned long long succ_start = 0, succ_end = 0;
#endif
    unsigned long long release_succ_wait = 0;

    scl_node_t *next_qnode = lock->next;
    if (!next_qnode) {
        if (__sync_bool_compare_and_swap(&lock->tail, lock, NULL)) {
            return __scl_mutex_unlock_accounting(lock, release_succ_wait);
        }
#ifdef DEBUG
        succ_start = rdtsc();
#endif
        spin_then_yield(spin_limit, !lock->next);
        next_qnode = lock->next;
#ifdef DEBUG
        succ_end          = rdtsc();
        release_succ_wait = succ_end - succ_start;
#endif
    }
    next_qnode->state = RUNNABLE;
    return __scl_mutex_unlock_accounting(lock, release_succ_wait);
}

int scl_mutex_trylock(scl_mutex_t *lock, scl_node_t *me) {
    // The original implementation does not come with a trylock primitive
    // However, one can simply modify the mutex_lock to implement such function
    // if needed...
    assert(0 && "Trylock not implemented for SCL.");

    return EBUSY;
}

int scl_mutex_destroy(scl_mutex_t *lock) {
    return pthread_key_delete(lock->tinfo_key);
    // return 0;
}

void scl_thread_start(void) {
}

void scl_thread_exit(void) {
}

void scl_application_init(void) {
}

void scl_application_exit(void) {
}