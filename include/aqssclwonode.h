/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Hugo Guiroux <hugo.guiroux at gmail dot com>
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
#ifndef __AQSSCL_H__
#define __AQSSCL_H__

#include <pthread.h>
#include <stdint.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>

#include "padding.h"
#define LOCK_ALGORITHM "AQSSCLWONODE"
#define NEED_CONTEXT 0
#define SUPPORT_WAITING 1

/*
 * Bit manipulation (not used currently)
 * Will use just one variable of 4 byts to enclose the following:
 * 0-7:   locked or unlocked
 * 8-15:  shuffle leader or not
 * 16-31: shuffle count
 */
#define _AQSSCL_SET_MASK(type)                                                 \
    (((1U << _AQSSCL_##type##_BITS) - 1) << _AQSSCL_##type##_OFFSET)

/* This is directly used by the tail bytes (2 bytes) */
#define _AQSSCL_TAIL_IDX_OFFSET (0)
#define _AQSSCL_TAIL_IDX_BITS 2
#define _AQSSCL_TAIL_IDX_MASK _AQSSCL_SET_MASK(TAIL_IDX)

#define _AQSSCL_TAIL_CPU_OFFSET                                                \
    (_AQSSCL_TAIL_IDX_OFFSET + _AQSSCL_TAIL_IDX_BITS)
#define _AQSSCL_TAIL_CPU_BITS (16 - _AQSSCL_TAIL_CPU_OFFSET)
#define _AQSSCL_TAIL_CPU_MASK _AQSSCL_SET_MASK(TAIL_CPU)

#define _AQSSCL_TAIL_OFFSET _AQSSCL_TAIL_IDX_OFFSET
#define _AQSSCL_TAIL_MASK (_AQSSCL_TAIL_IDX_MASK | _AQSSCL_TAIL_CPU_MASK)

/* Use 1 bit for the NOSTEAL part */
#define _AQSSCL_NOSTEAL_OFFSET 0
#define _AQSSCL_NOSTEAL_BITS 1
#define _AQSSCL_NOSTEAL_MASK _AQSSCL_SET_MASK(NOSTEAL)

/* We can support up to 127 sockets for NUMA-aware fastpath stealing */
#define _AQSSCL_NUMA_ID_OFFSET (_AQSSCL_NOSTEAL_OFFSET + _AQSSCL_NOSTEAL_BITS)
#define _AQSSCL_NUMA_ID_BITS 7
#define _AQSSCL_NUMA_ID_MASK _AQSSCL_SET_MASK(NUMA_ID)
#define _AQSSCL_NUMA_ID_VAL(v)                                                 \
    ((v)&_AQSSCL_NUMA_ID_MASK) >> _AQSSCL_NUMA_ID_OFFSET

#define _AQSSCL_LOCKED_OFFSET 0
#define _AQSSCL_LOCKED_BITS 8
#define _AQSSCL_LOCKED_NOSTEAL_OFFSET                                          \
    (_AQSSCL_LOCKED_OFFSET + _AQSSCL_LOCKED_BITS)

#define AQSSCL_NOSTEAL_VAL 1
#define AQSSCL_STATUS_WAIT 0
#define AQSSCL_STATUS_LOCKED 1
#define AQSSCL_MAX_LOCK_COUNT 256
#define AQSSCL_SERVE_COUNT (255) /* max of 8 bits */

/* Arch utility */
static inline void smp_rmb(void) {
    __asm __volatile("lfence" ::: "memory");
}

static inline void smp_cmb(void) {
    __asm __volatile("" ::: "memory");
}

#define barrier() smp_cmb()

static inline void __write_once_size(volatile void *p, void *res, int size) {
    switch (size) {
    case 1:
        *(volatile uint8_t *)p = *(uint8_t *)res;
        break;
    case 2:
        *(volatile uint16_t *)p = *(uint16_t *)res;
        break;
    case 4:
        *(volatile uint32_t *)p = *(uint32_t *)res;
        break;
    case 8:
        *(volatile uint64_t *)p = *(uint64_t *)res;
        break;
    default:
        barrier();
        memcpy((void *)p, (const void *)res, size);
        barrier();
    }
}

static inline void __read_once_size(volatile void *p, void *res, int size) {
    switch (size) {
    case 1:
        *(uint8_t *)res = *(volatile uint8_t *)p;
        break;
    case 2:
        *(uint16_t *)res = *(volatile uint16_t *)p;
        break;
    case 4:
        *(uint32_t *)res = *(volatile uint32_t *)p;
        break;
    case 8:
        *(uint64_t *)res = *(volatile uint64_t *)p;
        break;
    default:
        barrier();
        memcpy((void *)res, (const void *)p, size);
        barrier();
    }
}

#define WRITE_ONCE(x, val)                                                     \
    ({                                                                         \
        union {                                                                \
            typeof(x) __val;                                                   \
            char __c[1];                                                       \
        } __u = {.__val = (typeof(x))(val)};                                   \
        __write_once_size(&(x), __u.__c, sizeof(x));                           \
        __u.__val;                                                             \
    })

#define READ_ONCE(x)                                                           \
    ({                                                                         \
        union {                                                                \
            typeof(x) __val;                                                   \
            char __c[1];                                                       \
        } __u;                                                                 \
        __read_once_size(&(x), __u.__c, sizeof(x));                            \
        __u.__val;                                                             \
    })

#define smp_cas(__ptr, __old_val, __new_val)                                   \
    __sync_val_compare_and_swap(__ptr, __old_val, __new_val)
#define smp_swap(__ptr, __val) __sync_lock_test_and_set(__ptr, __val)
#define smp_faa(__ptr, __val) __sync_fetch_and_add(__ptr, __val)

// Per-thread tracking information
typedef struct aqsscl_thread_info {
    uint64_t banned_until;
    uint64_t weight;
    uint64_t slice;
    uint64_t start_ticks;
    int banned;
#ifdef AQMSCL_STAT
    aqsscl_stats_t stat;
#endif
} aqsscl_thread_info_t;

typedef struct aqsscl_node {
    struct aqsscl_node *next;
    union {
        uint32_t locked;
        struct {
            uint8_t lstatus;
            uint8_t sleader;
            uint16_t wcount;
        };
    };
    int nid;
    int cid;
    struct aqsscl_node *last_visited;

    int lock_status;
    int type;
    char __pad2[pad_to_cache_line(sizeof(uint32_t))];

    uint64_t slice;
    char __pad3[pad_to_cache_line(sizeof(uint64_t))];
} aqsscl_node_t __attribute__((aligned(L_CACHE_LINE_SIZE)));

typedef struct aqsscl_mutex {
    struct aqsscl_node *tail;
    union {
        uint32_t val;
        struct {
            uint8_t locked;
            uint8_t no_stealing;
        };
        struct {
            uint16_t locked_no_stealing;
            uint8_t __pad[2];
        };
    };
    char __pad2[pad_to_cache_line(sizeof(uint32_t))];
#if COND_VAR
    pthread_mutex_t posix_lock;
    char __pad3[pad_to_cache_line(sizeof(pthread_mutex_t))];
#endif

    volatile uint64_t slice;
    volatile uint64_t total_weight;
    volatile int slice_valid;
    pthread_key_t thread_info_key;
} aqsscl_mutex_t __attribute__((aligned(L_CACHE_LINE_SIZE)));

typedef pthread_cond_t aqsscl_cond_t;
aqsscl_mutex_t *aqsscl_mutex_create(const pthread_mutexattr_t *attr);
int aqsscl_mutex_lock(aqsscl_mutex_t *impl, aqsscl_node_t *me);
int aqsscl_mutex_trylock(aqsscl_mutex_t *impl, aqsscl_node_t *me);
void aqsscl_mutex_unlock(aqsscl_mutex_t *impl, aqsscl_node_t *me);
int aqsscl_mutex_destroy(aqsscl_mutex_t *lock);
int aqsscl_cond_init(aqsscl_cond_t *cond, const pthread_condattr_t *attr);
int aqsscl_cond_timedwait(aqsscl_cond_t *cond, aqsscl_mutex_t *lock,
                          aqsscl_node_t *me, const struct timespec *ts);
int aqsscl_cond_wait(aqsscl_cond_t *cond, aqsscl_mutex_t *lock,
                     aqsscl_node_t *me);
int aqsscl_cond_signal(aqsscl_cond_t *cond);
int aqsscl_cond_broadcast(aqsscl_cond_t *cond);
int aqsscl_cond_destroy(aqsscl_cond_t *cond);
void aqsscl_thread_start(void);
void aqsscl_thread_exit(void);
void aqsscl_application_init(void);
void aqsscl_application_exit(void);
void aqsscl_init_context(aqsscl_mutex_t *impl, aqsscl_node_t *context,
                         int number);

typedef aqsscl_mutex_t lock_mutex_t;
typedef aqsscl_node_t lock_context_t;
typedef aqsscl_cond_t lock_cond_t;

#define lock_mutex_create aqsscl_mutex_create
#define lock_mutex_lock aqsscl_mutex_lock
#define lock_mutex_trylock aqsscl_mutex_trylock
#define lock_mutex_unlock aqsscl_mutex_unlock
#define lock_mutex_destroy aqsscl_mutex_destroy
#define lock_cond_init aqsscl_cond_init
#define lock_cond_timedwait aqsscl_cond_timedwait
#define lock_cond_wait aqsscl_cond_wait
#define lock_cond_signal aqsscl_cond_signal
#define lock_cond_broadcast aqsscl_cond_broadcast
#define lock_cond_destroy aqsscl_cond_destroy
#define lock_thread_start aqsscl_thread_start
#define lock_thread_exit aqsscl_thread_exit
#define lock_application_init aqsscl_application_init
#define lock_application_exit aqsscl_application_exit
#define lock_init_context aqsscl_init_context

#endif // __AQSSCL_H__
