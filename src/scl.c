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

static inline scl_node_t *flqnode(scl_mutex_t *lock) {
    return (scl_node_t *)((char *)&lock->next - offsetof(scl_node_t, next));
}

scl_mutex_t *scl_mutex_create(const pthread_mutexattr_t *attr) {
    scl_mutex_t *impl = (scl_mutex_t *)alloc_cache_align(sizeof(scl_mutex_t));
    if (!impl) {
        return NULL;
    }

    impl->tail         = NULL;
    impl->next         = NULL;
    impl->total_weight = 0;
    impl->slice        = 0;
    impl->slice_valid  = 0;
    if (pthread_key_create(&impl->tinfo_key, NULL)) {
        return NULL;
    }

    return impl;
}

static tinfo_t *tinfo_create(scl_mutex_t *impl, int weight) {
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
    __sync_add_and_fetch(&impl->total_weight, weight);
    info->banned      = 0;
    info->slice       = 0;
    info->start_ticks = 0;
#ifdef DEBUG
    memset(&info->stat, 0, sizeof(stats_t));
    info->stat.start = info->banned_until;
#endif
    return info;
}

static tinfo_t *__thread_info(scl_mutex_t *impl) {
    // Retrieve per-thread tracking information
    tinfo_t *info = (tinfo_t *)pthread_getspecific(impl->tinfo_key);
    if (!info) {
        info = tinfo_create(impl, 0);
        pthread_setspecific(impl->tinfo_key, info);
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

static int __scl_mutex_lock_begin(scl_mutex_t *impl, tinfo_t *info,
                                  unsigned long long now) {
    if (info->banned) {
        if ((now = rdtsc()) < info->banned_until) {
            unsigned long long banned_time = info->banned_until - now;
#ifdef DEBUG
            info->stat.banned_time += banned_time;
#endif
            // sleep with granularity of sleep_granularity us
            while (banned_time > sleep_time) {
                struct timespec req = {
                    .tv_sec  = banned_time / cycle_per_s,
                    .tv_nsec = (banned_time % cycle_per_s / cycle_per_us /
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

    scl_node_t n = {0};
    while (1) {
        scl_node_t *prev = impl->tail;
        if (__sync_bool_compare_and_swap(&impl->tail, prev, &n)) {
            // enter the lock queue
            if (!prev) {
                n.state    = RUNNABLE;
                impl->next = &n;
            } else {
                if (prev == flqnode(impl)) {
                    n.state    = NEXT;
                    prev->next = &n;
                } else {
                    prev->next = &n;
                    // wait until we become the next runnable
#ifdef DEBUG
                    now = rdtsc();
#endif
                    do {
                        futex((int *)&n.state, FUTEX_WAIT_PRIVATE, INIT, NULL);
                    } while (INIT == n.state);
#ifdef DEBUG
                    info->stat.next_runnable_wait += rdtsc() - now;
#endif
                }
            }
            // invariant: n.state >= NEXT

            // wait until the current slice expires
            int slice_valid;
            unsigned long long curr_slice;
            while ((slice_valid = impl->slice_valid) &&
                   (now = rdtsc()) + sleep_granularity <
                       (curr_slice = impl->slice)) {
                unsigned long long slice_left = curr_slice - now;
                struct timespec timeout       = {
                    .tv_sec = 0, // slice will be less then 1 sec
                    .tv_nsec =
                        (slice_left / (cycle_per_us * sleep_granularity)) *
                        sleep_granularity * 1000,
                };
                futex((int *)&impl->slice_valid, FUTEX_WAIT_PRIVATE, 0,
                      &timeout);
#ifdef DEBUG
                info->stat.prev_slice_wait += rdtsc() - now;
#endif
            }
            if (slice_valid) {
                spin_then_yield(spin_limit, (slice_valid = impl->slice_valid) &&
                                                rdtsc() < impl->slice);
                if (slice_valid)
                    impl->slice_valid = 0;
            }
            // invariant: rdtsc() >= curr_slice && lock->slice_valid == 0

#ifdef DEBUG
            now = rdtsc();
#endif
            // spin until RUNNABLE and try to grab the lock
            spin_then_yield(spin_limit,
                            RUNNABLE != n.state ||
                                0 == __sync_bool_compare_and_swap(
                                         &n.state, RUNNABLE, RUNNING));
            // invariant: n.state == RUNNING
#ifdef DEBUG
            info->stat.runnable_wait += rdtsc() - now;
#endif

            // record the successor in the lock so we can notify it when we
            // release
            scl_node_t *succ = n.next;
            if (NULL == succ) {
                impl->next = NULL;
                if (0 == __sync_bool_compare_and_swap(&impl->tail, &n,
                                                      flqnode(impl))) {
                    spin_then_yield(spin_limit, NULL == (succ = n.next));
#ifdef DEBUG
                    info->stat.succ_wait += rdtsc() - now;
#endif
                    impl->next = succ;
                }
            } else {
                impl->next = succ;
            }
            // invariant: NULL == succ <=> lock->qtail == flqnode(lock)

            now               = rdtsc();
            info->start_ticks = now;
            info->slice       = now + fairlock_granularity;
            impl->slice       = info->slice;
            impl->slice_valid = 1;
            // wake up successor if necessary
            if (succ) {
                succ->state = NEXT;
                futex((int *)&succ->state, FUTEX_WAKE_PRIVATE, 1, NULL);
            }
            return 0;
        }
    }
}

int scl_mutex_lock(scl_mutex_t *impl, scl_node_t *UNUSED(me)) {
    unsigned long long now;

    tinfo_t *info = __thread_info(impl);

    if (impl->slice_valid) {
        unsigned long long curr_slice = impl->slice;
        // If owner of current slice, try to reenter at the beginning of the
        // queue
        if (curr_slice == info->slice && (now = rdtsc()) < curr_slice) {
            scl_node_t *succ = impl->next;
            if (!succ) { // next waiting thread is empty
                if (__sync_bool_compare_and_swap(&impl->tail, NULL,
                                                 flqnode(impl))) {
                    __scl_mutex_lock_reenter(info, now);
                    return 0;
                }
                spin_then_yield(spin_limit, (now = rdtsc()) < curr_slice &&
                                                NULL == (succ = impl->next));
#ifdef DEBUG
                info->stat.own_slice_wait += rdtsc() - now;
#endif
                // let the succ invalidate the slice, and don't need to wake it
                // up because slice expires naturally
                if (now >= curr_slice) {
                    return __scl_mutex_lock_begin(impl, info, now);
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
    return __scl_mutex_lock_begin(impl, info, now);
}

void scl_mutex_unlock(scl_mutex_t *impl, scl_node_t *UNUSED(me)) {
    unsigned long long now, cs;
#ifdef DEBUG
    unsigned long long succ_start = 0, succ_end = 0;
#endif
    tinfo_t *info;

    scl_node_t *succ = impl->next;
    if (!succ) {
        if (__sync_bool_compare_and_swap(&impl->tail, flqnode(impl), NULL))
            goto accounting;
#ifdef DEBUG
        succ_start = rdtsc();
#endif
        spin_then_yield(spin_limit, NULL == (succ = impl->next));
#ifdef DEBUG
        succ_end = rdtsc();
#endif
    }
    succ->state = RUNNABLE;

accounting:
    // invariant: NULL == succ || succ->state = RUNNABLE
    info = (tinfo_t *)pthread_getspecific(impl->tinfo_key);
    now  = rdtsc();
    cs   = now - info->start_ticks;
    info->banned_until +=
        cs *
        (__atomic_load_n(&impl->total_weight, __ATOMIC_RELAXED) / info->weight);
    info->banned = now < info->banned_until;

    if (info->banned) {
        if (__sync_bool_compare_and_swap(&impl->slice_valid, 1, 0)) {
            futex((int *)&impl->slice_valid, FUTEX_WAKE_PRIVATE, 1, NULL);
        }
    }
#ifdef DEBUG
    info->stat.release_succ_wait += succ_end - succ_start;
#endif
}

int scl_mutex_trylock(scl_mutex_t *impl, scl_node_t *me) {
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