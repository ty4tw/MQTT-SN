#ifdef __APPLE__

#include "sem_timedwait.h"
#include <errno.h>
#include <time.h>

// Mac OS X timedwait wrapper
int sem_timedwait_mach(sem_t* sem, long timeout_ms) {
    int retval = 0;
    mach_timespec_t mts;
    if (timeout_ms >= 0) {
        mts.tv_sec = timeout_ms / 1000;
        mts.tv_nsec = (timeout_ms % 1000) * 1000000;
    } else {
        // FIX: If we really wait forever, we cannot shut down VERMONT
        // this is mac os x specific and does not happen on linux
        // hence, we just add a small timeout instead of blocking
        // indefinately
        mts.tv_sec = 1;
        mts.tv_nsec = 0;
    }
    retval = semaphore_timedwait(*sem, mts);
    switch (retval) {
        case KERN_SUCCESS:
            return 0;
        case KERN_OPERATION_TIMED_OUT:
            errno = ETIMEDOUT;
            break;
        case KERN_ABORTED:
            errno = EINTR;
            break;
        default:
            errno =  EINVAL;
            break;
    }
    return -1;
}

#endif