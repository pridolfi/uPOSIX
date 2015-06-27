/* Copyright 2014, Pablo Ridolfi
 *
 * This file is part of uPOSIX.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PTHREAD_H_
#define PTHREAD_H_

/** @brief RTOS thread management header file.
 **
 **/

/** \addtogroup kernel uPOSIX RTOS
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 * PR           Pablo Ridolfi
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20140929 v0.0.1   PR first version
 */

/*==================[inclusions]=============================================*/

#include "stdint.h"

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

/** @brief Default thread stack size. */
#define PTHREAD_DEFAULT_STACK_SIZE		1024

/** @brief Maximum number of threads in READY state, per prio. level. */
#define PTHREAD_STATIC_LIST_SIZE		5

/** @brief Mutex types (only this one for now...) */
#define PTHREAD_MUTEX_NORMAL			0

/*==================[typedef]================================================*/

/** @brief Thread states enumeration. */
typedef enum
{
	PTHREAD_STATE_ERROR,	/**< The thread has been suspended because of an error.
					 	 	 	 It cannot be resumed. */
	PTHREAD_STATE_DETACHED,	/**< The thread is terminated and joined, waiting for the
								 scheduler to release its resources. */
	PTHREAD_STATE_READY,	/**< The thread is waiting to be selected by the scheduler
					 	 	 	 and go RUNNING. */
	PTHREAD_STATE_RUNNING,	/**< This thread is currently running. */

	PTHREAD_STATE_BLOCKED,	/**< This thread is waiting for an event. */

	PTHREAD_STATE_ZOMBIE	/**< The thread has finished its execution. Parent thread can
					 	 	 	 retreive its return value before releasing resources. */
}threadState_t;

/** @brief Thread priorities enumeration. */
typedef enum
{
	PTHREAD_PRIORITY_ERROR	= -1,
	PTHREAD_PRIORITY_IDLE	=  0,
	PTHREAD_PRIORITY_LOW	=  1,
	PTHREAD_PRIORITY_MEDIUM	=  2,
	PTHREAD_PRIORITY_HIGH	=  3,
	PTHREAD_PRIORITY_MAX	=  4
}threadPriority_t;

/** @brief Thread attach state enumeration. */
typedef enum
{
	PTHREAD_CREATE_DETACHED,
	PTHREAD_CREATE_JOINABLE
}threadDetachState_t;

/** @brief Thread scheduling policy.
 * 	@see sched_setscheduler man page.
 */
typedef enum
{
	SCHED_MIN 	= -1,
	SCHED_FIFO	= 0,
	SCHED_RR	= 1,
	SCHED_MAX	= 2
}threadSchedPolicy_t;

typedef enum
{
	PTHREAD_CANCEL_ENABLE,
	PTHREAD_CANCEL_DISABLE
}threadCancelState_t;

typedef enum
{
	PTHREAD_CANCEL_DEFERRED,
	PTHREAD_CANCEL_ASYNCHRONOUS
}threadCancelType_t;

/** @brief Thread sched_param structure (priorities). */
typedef struct sched_param
{
	threadPriority_t  sched_priority;		/**< static priority */
	threadPriority_t  sched_curpriority; 	/**< dynamic priority (ceiling protocol) */
}sched_param_t;

/** @brief Thread attribute structure. */
typedef struct
{
    void * sp;							/**< current stack pointer */
    threadState_t state;				/**< thread execution state */
    threadDetachState_t detachstate;	/**< thread attach state */
    sched_param_t schedparam;			/**< thread priorities */
    threadSchedPolicy_t schedpolicy;	/**< scheduling policy */
    threadCancelState_t cancelstate;	/**< cancel state (enabled by default) */
    threadCancelType_t canceltype;		/**< cancel type (DEFERRED not supported) */
    void * retval;						/**< return value if terminated */
    void * stackaddr;					/**< start address for stack */
    unsigned long int stacksize;		/**< stack size in bytes */
} pthread_attr_t;

/** @brief pthread_t definition */
typedef pthread_attr_t * pthread_t;

/** @brief static thread lists */
typedef struct
{
	pthread_t list[PTHREAD_STATIC_LIST_SIZE]; 	/**< static list */
	uint8_t index;								/**< item count */
}pthread_static_list;

/** @brief pthread mutex lock structure definition */
struct _pthread_fastlock
{
    long int status;
    int spinlock;
};

/** @brief pthread mutex attribute definition
  * @warning NOT SUPPORTED.
  */
typedef void pthread_mutexattr_t;

/** @brief pthread mutex definition */
typedef pthread_static_list * pthread_mutex_t;

/*==================[external data declaration]==============================*/

extern const pthread_attr_t pthread_default_attr;
extern pthread_static_list thread_list;
extern pthread_static_list thread_ready_list[PTHREAD_PRIORITY_MAX+1];

/*==================[external functions declaration]=========================*/

int pthread_create(pthread_t *thread, const pthread_attr_t *attr,
		void *(*start_routine)(void*), void *arg);

int pthread_static_list_add(pthread_static_list * list, pthread_t t);

int pthread_static_list_remove(pthread_static_list * list, pthread_t t);

int pthread_add_ready(pthread_t t);

int pthread_remove_ready(pthread_t t);

int pthread_join(pthread_t thread, void **retval);

int pthread_setschedparam(pthread_t thread, threadSchedPolicy_t policy,
		const sched_param_t *param);

int pthread_getschedparam(pthread_t thread, threadSchedPolicy_t *policy,
		sched_param_t *param);

int pthread_mutex_init(pthread_mutex_t *mutex, const pthread_mutexattr_t *attr);

int pthread_mutex_destroy(pthread_mutex_t *mutex);

int pthread_mutex_lock(pthread_mutex_t *mutex);

int pthread_mutex_trylock(pthread_mutex_t *mutex);

int pthread_mutex_unlock(pthread_mutex_t *mutex);

int pthread_setcancelstate(threadCancelState_t state, threadCancelState_t *oldstate);

int pthread_setcanceltype(threadCancelType_t type, threadCancelType_t *oldtype);

void pthread_exit(void * retval);

int usleep(unsigned int usec);

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* PTHREAD_H_ */
