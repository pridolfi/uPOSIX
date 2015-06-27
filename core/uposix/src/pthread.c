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

/** @brief RTOS thread management source file.
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

#include "pthread.h"
#include "heap.h"
#include "kernel.h"
#include "arch.h"
#include "string.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/** @brief default pthread_attr_t structure initializer. */
const pthread_attr_t pthread_default_attr =
{
		0,
		PTHREAD_STATE_READY,
		PTHREAD_CREATE_JOINABLE,
		{ PTHREAD_PRIORITY_LOW, PTHREAD_PRIORITY_LOW },
		SCHED_RR,
		PTHREAD_CANCEL_ENABLE,
		PTHREAD_CANCEL_ASYNCHRONOUS,
		0,
		0,
		PTHREAD_DEFAULT_STACK_SIZE
};

/** @brief list of all created threads */
pthread_static_list thread_list;

/** @brief list of all threads in ready state, ordered by priority */
pthread_static_list thread_ready_list[PTHREAD_PRIORITY_MAX+1];

/*==================[internal functions definition]==========================*/

/** @brief 	Internal function to get memory space for the thread structure.
 * 			It can be done dynamically (like now) or statically.
 * 	@return	pthread_t (pointer to thread structure).
 */
static pthread_t pthread_alloc_context(void)
{
	return (pthread_t)kmalloc(sizeof(pthread_attr_t));
}

/** @brief 				Internal function to get memory space for the thread stack.
 * 						It can be done dynamically (like now) or statically.
 * 	@param stacksize	Stack size in bytes.
 * 	@return				Pointer to stack space.
 */
static void * pthread_alloc_stack(int stacksize)
{
	return kmalloc(stacksize);
}

/*==================[external functions definition]==========================*/

/** @brief add item to static pthread list
 * 	@param list	pointer to the list we want to work with.
 * 	@param t	thread to be added to list.
 * 	@return 0 on success, -1 on error (list full).
 */
int pthread_static_list_add(pthread_static_list * list, pthread_t t)
{
	int rv = -1;
	if(list->index < PTHREAD_STATIC_LIST_SIZE)
	{
		list->list[list->index] = t;
		list->index++;
		rv = 0;
	}
	return rv;
}

/** @brief remove item from static pthread list
 * 	@param list	pointer to the list we want to work with.
 * 	@param t	thread to be removed.
 * 	@return 0 on success, -1 on error (list empty).
 */
int pthread_static_list_remove(pthread_static_list * list, pthread_t t)
{
	int rv = -1;
	if(list->index > 0)
	{
		uint8_t i;
		for(i=0; i<list->index; i++)
		{
			if(list->list[i] == t)
			{
				break;
			}
		}
		if(i < list->index)
		{
			uint8_t j;
			for(j = i; j < (list->index-1); j++)
			{
				list->list[j] = list->list[j+1];
			}
			list->index--;
			rv = 0;
		}
	}
	return rv;
}

/** @brief add thread to ready list (by priority)
 * 	@param t	thread to be added.
 * 	@return 0 on success, -1 on error (list full or out-of-range priority).
 */
int pthread_add_ready(pthread_t t)
{
	int rv = -1;

	threadPriority_t tprio = t->schedparam.sched_curpriority;

	if( (tprio <= PTHREAD_PRIORITY_MAX) &&
		(tprio >= PTHREAD_PRIORITY_IDLE) )
	{
		rv = pthread_static_list_add(&thread_ready_list[tprio], t);
	}

	return rv;
}

/** @brief remove thread from ready list (by priority)
 * 	@param t	thread to be removed.
 * 	@return 0 on success, -1 on error (list empty or out-of-range priority).
 */
int pthread_remove_ready(pthread_t t)
{
	int rv = -1;

	threadPriority_t tprio = t->schedparam.sched_curpriority;

	if( (tprio <= PTHREAD_PRIORITY_MAX) &&
		(tprio >= PTHREAD_PRIORITY_IDLE) )
	{
		rv = pthread_static_list_remove(&thread_ready_list[tprio], t);
	}

	return rv;
}

/** @brief 	Create a new thread and call the scheduler (if the new thread has
 * 			higher priority, it will preempt the calling thread).
 * 	@param thread			Pointer to a pthread_t variable to store thread ID (can be NULL if
 * 							the ID is not required).
 * 	@param attr				Thread attributes, if NULL, pthread_default_attr will be used.
 * 	@param start_routine	Thread entry point.
 * 	@param arg				Argument to be passed to the thread.
 * 	@return 0 on success, -1 on error.
 */
int pthread_create(pthread_t *thread, const pthread_attr_t *attr,
		void *(*start_routine)(void*), void *arg)
{
	int rv = -1;

	pthread_t t = 0;

	t = pthread_alloc_context();

	if(t != 0)
	{
		*t = (attr == 0 ? pthread_default_attr : *attr);

		t->stackaddr = pthread_alloc_stack(t->stacksize);

		if(t->stackaddr != 0)
		{
			memset(t->stackaddr, 0, t->stacksize);

			rv = arch_init_context(t, start_routine, arg);

			if(rv == 0)
			{
				rv = pthread_static_list_add(&thread_list, t);
				if(rv == 0)
				{
					rv = pthread_add_ready(t);
					if(rv == 0)
					{
						if(thread != 0)
						{
							*thread = t;
						}
						rv = schedule();
					}
				}
			}
		}
	}

	return rv;
}

/** @brief 	Wait for a thread to terminate and take its return value.
 * 	@param thread			Thread to be waited.
 * 	@param retval			Pointer to store thread return value.
 * 	@return 0 on success, EINVAL if the thread is not JOINABLE or not found.
 */
int pthread_join(pthread_t thread, void **retval)
{
	int rv = -1;
	uint8_t i;

	for(i=0; i < thread_list.index; i++)
	{
		if(thread_list.list[i] == thread)
		{
			if(thread->detachstate == PTHREAD_CREATE_JOINABLE)
			{
				while(thread->state != PTHREAD_STATE_ZOMBIE)
				{
					schedule();
				}
				if(retval != 0)
				{
					*retval = thread->retval;
				}
				thread->state = PTHREAD_STATE_DETACHED;
				rv = 0;
			}
			else
			{
				rv = EINVAL;
				break;
			}
		}
	}
	if(i == thread_list.index)
	{
		rv = EINVAL;
	}
	return rv;
}

/** @brief Set scheduling policy and priority.
 * 	@param thread	Thread to be configured.
 * 	@param policy	SCHED_FIFO and SCHED_RR are supported.
 * 	@param param	Only sched_priority member will be used.
 * 	@return 0 on success, -1 on error.
 */
int pthread_setschedparam(pthread_t thread, threadSchedPolicy_t policy,
		const sched_param_t *param)
{
	int rv = -1;
	uint8_t i;

	if(	(policy > SCHED_MIN) && (policy < SCHED_MAX) &&
		(param->sched_priority > PTHREAD_PRIORITY_ERROR) &&
		(param->sched_priority <= PTHREAD_PRIORITY_MAX) )
	{
		for(i=0; i<thread_list.index; i++)
		{
			if(thread == thread_list.list[i])
			{
				if(thread->state == PTHREAD_STATE_READY)
				{
					pthread_remove_ready(thread);
				}
				thread->schedpolicy = policy;
				thread->schedparam.sched_priority = param->sched_priority;
				thread->schedparam.sched_curpriority = param->sched_priority;
				if(thread->state == PTHREAD_STATE_READY)
				{
					pthread_add_ready(thread);
				}
				rv = 0;
				break;
			}
		}
	}
	return rv;
}

/** @brief Get scheduling policy and priority.
 * 	@param thread	Thread to work with.
 * 	@param policy	Pointer to a threadSchedPolicy_t type to store thread sched. policy.
 * 	@param param	Pointer to a sched_param_t type to store thread priorities.
 * 	@return 0 on success, -1 on error.
 */
int pthread_getschedparam(pthread_t thread, threadSchedPolicy_t *policy,
		sched_param_t *param)
{
	int rv = -1;
	uint8_t i;

	for(i=0; i<thread_list.index; i++)
	{
		if(thread == thread_list.list[i])
		{
			*policy = thread->schedpolicy;
			*param = thread->schedparam;
			rv = 0;
			break;
		}
	}

	return rv;
}

/** @brief Mutex creation and initialization function.
 * 	@param mutex	#pthread_mutex_t variable (passed by reference).
 * 	@param attr		Mutex attributes (not used, use 0).
 * 	@return 0 on successful creation, -1 on error.
 */
int pthread_mutex_init(pthread_mutex_t *mutex, const pthread_mutexattr_t *attr)
{
	int rv = -1;

	if(mutex != 0)
	{
		*mutex = kmalloc(sizeof(pthread_static_list));

		if(*mutex != 0)
		{
			(*mutex)->index = 0;
			rv = 0;
		}
	}

	return rv;
}

/** @brief Mutex deinitialization.
 * 	@param mutex	Created mutex #pthread_mutex_t variable (passed by reference).
 * 	@return 0 on successful resource release, -1 on error (invalid argument).
 */
int pthread_mutex_destroy(pthread_mutex_t *mutex)
{
	int rv = -1;

	if(mutex && *mutex)
	{
		kfree(*mutex);

		rv = 0;
	}

	return rv;
}

/** @brief 	Mutex lock (take). This function blocks the calling thread if another thread
 * 			previously called pthread_mutex_lock.
 * 	@param mutex	Created mutex #pthread_mutex_t variable (passed by reference).
 * 	@return 0 on successful lock, -1 on error.
 */
int pthread_mutex_lock(pthread_mutex_t *mutex)
{
	int rv = -1;
	pthread_t t;

	if(mutex && *mutex)
	{
		arch_critical_section_start();

		t = getCurrentContext();
		if(t != INVALID_CONTEXT)
		{
			rv = pthread_static_list_add(*mutex, t);

			if(rv == 0)
			{
				while((*mutex)->index > 1)
				{
					t->state = PTHREAD_STATE_BLOCKED;
					arch_critical_section_end();
					if(kernel_get_state() != KERNEL_STATE_IRQ)
					{
						schedule();
					}
				}
			}
			else
			{
				kernel_panic();
			}
		}
		else
		{
			kernel_panic();
		}
		arch_critical_section_end();
	}

	return rv;
}

/** @brief 	Mutex try lock (take). This function tells the calling thread if the mutex would block.
 * 	@param mutex	Created mutex #pthread_mutex_t variable (passed by reference).
 * 	@return 0 if the mutex is not locked, EBUSY if the mutex would block.
 */
int pthread_mutex_trylock(pthread_mutex_t *mutex)
{
	int rv = -1;

	if(mutex && *mutex)
	{
		if((*mutex)->index > 0)
		{
			rv = EBUSY;
		}
		else
		{
			rv = 0;
		}
	}

	return rv;
}

/** @brief 	Mutex unlock (give). This function releases the mutex.
 * 	@param mutex	Created mutex #pthread_mutex_t variable (passed by reference).
 * 	@return 0 on successful unlock, -1 on error.
 */
int pthread_mutex_unlock(pthread_mutex_t *mutex)
{
	int rv = -1;
	pthread_t t;

	if(mutex && *mutex)
	{
		arch_critical_section_start();

		pthread_static_list_remove(*mutex, getCurrentContext());

		if((*mutex)->index > 0)
		{
			t = (*mutex)->list[0];
			rv = pthread_static_list_remove(*mutex, t);

			if(rv == 0)
			{
				if(t->state == PTHREAD_STATE_BLOCKED)
				{
					t->state = PTHREAD_STATE_READY;
					rv = pthread_add_ready(t);
					if(rv == 0)
					{
						pthread_t curr_ctx = getCurrentContext();
						if(t->schedparam.sched_curpriority > curr_ctx->schedparam.sched_curpriority)
						{
							if(kernel_get_state() != KERNEL_STATE_IRQ)
							{
								rv = schedule();
							}
						}
					}
					else
					{
						kernel_panic();
					}
				}
			}
			else
			{
				kernel_panic();
			}
		}
		arch_critical_section_end();
	}

	return rv;
}

/** @brief	obtain ID of the calling thread
 *  @return This function always succeeds, returning the calling thread's ID.
 */
pthread_t pthread_self(void)
{
	return getCurrentContext();
}

/** @brief	send a cancellation request to a thread
 *  @param thread The thread to be cancelled.
 *  @return On success, pthread_cancel() returns 0; on error, it returns a nonzero error number.
 */
int pthread_cancel(pthread_t thread)
{
	int rv = -1;

	if( (thread->cancelstate == PTHREAD_CANCEL_ENABLE) &&
		(thread->canceltype == PTHREAD_CANCEL_ASYNCHRONOUS) &&
		(thread->state != PTHREAD_STATE_RUNNING) )
	{
		arch_critical_section_start();

		if(thread->state == PTHREAD_STATE_READY)
		{
			pthread_remove_ready(thread);
		}

		thread->state = PTHREAD_STATE_DETACHED;

		arch_critical_section_end();

		rv = 0;
	}

	return rv;
}

/** @brief set cancelability state
 *
 */
int pthread_setcancelstate(threadCancelState_t state, threadCancelState_t *oldstate)
{
	int rv = -1;
	pthread_t t = getCurrentContext();

	if(oldstate)
	{
		*oldstate = t->cancelstate;
		t->cancelstate = state;
		rv = 0;
	}

	return rv;
}

/** @brief set cancelability type
 *
 */
int pthread_setcanceltype(threadCancelType_t type, threadCancelType_t *oldtype)
{
	int rv = -1;
	pthread_t t = getCurrentContext();

	if(oldtype)
	{
		*oldtype = t->canceltype;
		t->canceltype = type;
		rv = 0;
	}

	return rv;
}

/** @brief terminate calling thread
 *  @param retval Return value.
 */
void pthread_exit(void * retval)
{
	thread_return_hook(retval);
}

/** @brief suspend execution for microsecond intervals.
 * 	@param usec sleep time in microseconds, cannot be less than 1000 (1ms) because of
 * 				the system timer.
 * 	@return 0 on success, -1 on error.
 */
int usleep(unsigned int usec)
{
	int rv = -1;
	pthread_t t = getCurrentContext();

	if((kernel_get_state() != KERNEL_STATE_IRQ) && (usec >= 1000))
	{
		t->retval = (void *)(usec/1000);

		if(t->state == PTHREAD_STATE_RUNNING)
		{
			t->state = PTHREAD_STATE_BLOCKED;
			rv = schedule();
		}
		else
		{
			/* if the thread is not running, how can it call usleep? */
			kernel_panic();
		}
	}
	return rv;
}

/** @} doxygen end group definition */
/*==================[end of file]============================================*/
