#ifndef RMD_UTILS_H
#define RMD_UTILS_H

#include <pthread.h>
#include <stdio.h>
#include <limits.h>

#define RT_MS       2

int generate_nrt_thread(pthread_t &thread_nrt, void* (*thread_func)(void *), const char* name, int cpu_no, void *arg);
int generate_rt_thread(pthread_t &thread_rt, void* (*thread_func)(void *), const char* name, int cpu_no, int priority, void *arg);
int generate_rt_thread_hard(pthread_t &thread_rt, void* (*thread_func)(void *), const char* name, int cpu_no, int priority, void *arg);

void timespec_add_us(struct timespec *t, long us);
int timespec_cmp(struct timespec *a, struct timespec *b);
int timediff_us(struct timespec *before, struct timespec *after);

#endif // RMD_UTILS_H
