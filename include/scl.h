#ifndef __SCL_H__
#define __SCL_H__

#include "padding.h"
#include <assert.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <time.h>
#define LOCK_ALGORITHM "SCL"

enum scl_node_state {
    INIT = 0, // not waiting or after next runnable node
    NEXT,
    RUNNABLE,
    RUNNING
};

#ifdef DEBUG
typedef struct stats {
    unsigned long long reenter;
    unsigned long long banned_time;
    unsigned long long start;
    unsigned long long next_runnable_wait;
    unsigned long long prev_slice_wait;
    unsigned long long own_slice_wait;
    unsigned long long runnable_wait;
    unsigned long long succ_wait;
    unsigned long long release_succ_wait;
} stats_t;
#endif

// Per-thread tracking information
typedef struct tinfo {
    unsigned long long banned_until;
    unsigned long long weight;
    unsigned long long slice;
    unsigned long long start_ticks;
    int banned;
#ifdef DEBUG
    stats_t stat;
#endif
} tinfo_t;

typedef struct scl_node {
    volatile int state __attribute__((aligned(L_CACHE_LINE_SIZE)));
    struct scl_node *volatile next __attribute__((aligned(L_CACHE_LINE_SIZE)));
} scl_node_t __attribute__((aligned(L_CACHE_LINE_SIZE)));

typedef struct scl_mutex {
    struct scl_node *volatile tail __attribute__((aligned(L_CACHE_LINE_SIZE)));
    struct scl_node *volatile next __attribute__((aligned(L_CACHE_LINE_SIZE)));
    volatile unsigned long long slice
        __attribute__((aligned(L_CACHE_LINE_SIZE)));
    volatile int slice_valid __attribute__((aligned(L_CACHE_LINE_SIZE)));
    pthread_key_t tinfo_key;
    volatile unsigned long long total_weight;
} scl_mutex_t __attribute__((aligned(L_CACHE_LINE_SIZE)));

typedef pthread_cond_t scl_cond_t;
scl_mutex_t *scl_mutex_create(const pthread_mutexattr_t *attr);
int scl_mutex_lock(scl_mutex_t *impl, scl_node_t *me);
int scl_mutex_trylock(scl_mutex_t *impl, scl_node_t *me);
void scl_mutex_unlock(scl_mutex_t *impl, scl_node_t *me);
int scl_mutex_destroy(scl_mutex_t *lock);
int scl_cond_init(scl_cond_t *cond, const pthread_condattr_t *attr);
int scl_cond_timedwait(scl_cond_t *cond, scl_mutex_t *lock, scl_node_t *me,
                       const struct timespec *ts);
int scl_cond_wait(scl_cond_t *cond, scl_mutex_t *lock, scl_node_t *me);
int scl_cond_signal(scl_cond_t *cond);
int scl_cond_broadcast(scl_cond_t *cond);
int scl_cond_destroy(scl_cond_t *cond);
void scl_thread_start(void);
void scl_thread_exit(void);
void scl_application_init(void);
void scl_application_exit(void);

typedef scl_mutex_t lock_mutex_t;
typedef scl_node_t lock_context_t;
typedef scl_cond_t lock_cond_t;

#define lock_mutex_create scl_mutex_create
#define lock_mutex_lock scl_mutex_lock
#define lock_mutex_trylock scl_mutex_trylock
#define lock_mutex_unlock scl_mutex_unlock
#define lock_mutex_destroy scl_mutex_destroy
#define lock_cond_init scl_cond_init
#define lock_cond_timedwait scl_cond_timedwait
#define lock_cond_wait scl_cond_wait
#define lock_cond_signal scl_cond_signal
#define lock_cond_broadcast scl_cond_broadcast
#define lock_cond_destroy scl_cond_destroy
#define lock_thread_start scl_thread_start
#define lock_thread_exit scl_thread_exit
#define lock_application_init scl_application_init
#define lock_application_exit scl_application_exit

#endif // __SCL_H__
