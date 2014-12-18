#ifndef __OSX_SEM_TIMEDWAIT_H_
#define __OSX_SEM_TIMEDWAIT_H_

#ifdef __APPLE__

// Mac OS X does not have a working implementation of sem_init, sem_timedwait, ...
// use Mach semaphores instead
#include <mach/mach.h>
#include <mach/semaphore.h>
#include <mach/task.h>
#include <sys/semaphore.h>

// Mac OS X timedwait wrapper
int sem_timedwait_mach(sem_t* sem, long timeout_ms);

#endif
#endif