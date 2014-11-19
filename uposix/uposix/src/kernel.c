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

/** @brief Main RTOS kernel source file.
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

#include "kernel.h"
#include "heap.h"
#include "arch.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

static kernel_state_t kstate = KERNEL_STATE_ERROR;

static pthread_attr_t kernel_idle_thread;

static pthread_t kernel_current_context;

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/** @brief Search for DETACHED threads and release its resources.  */
static void kernel_remove_detached_threads(void)
{
	uint8_t i;
	for(i=0; i<thread_list.index; i++)
	{
		if(thread_list.list[i]->state == PTHREAD_STATE_DETACHED)
		{
			pthread_t t = thread_list.list[i];

			if(pthread_static_list_remove(&thread_list, t)==0)
			{
				kfree(t->stackaddr);
				kfree(t);
			}
		}
	}
}

/** @brief Idle state for kernel, no user threads in ready state */
static void kernel_idle(void)
{
	while(1)
	{
		kernel_remove_detached_threads();

		arch_halt_cpu();
	}
}

/** @brief update sleep counters for sleeping threads */
static void kernel_update_sleep_counters(void)
{
	int i;
	for(i=0; i<thread_list.index; i++)
	{
		if((thread_list.list[i]->state == PTHREAD_STATE_BLOCKED) &&
		   (thread_list.list[i]->retval != 0))
		{
			(unsigned int)thread_list.list[i]->retval--;
			if(thread_list.list[i]->retval==0)
			{
				thread_list.list[i]->state = PTHREAD_STATE_READY;
				if(pthread_add_ready(thread_list.list[i]) != 0)
				{
					/* cannot add thread to ready list */
					kernel_panic();
				}
			}
		}
	}
}

/*==================[external functions definition]==========================*/

/** @brief 	Get next thread to be resumed (READY -> RUNNING). This function
 * 			searches for READY threads by descending priority (higher priorities
 * 			first).
 * 	@return Thread context to be executed.
 */
pthread_t getNextContext(void)
{
	threadPriority_t prio;
	pthread_t t = INVALID_CONTEXT;
	uint8_t i;

	for( prio = PTHREAD_PRIORITY_MAX;
		(prio > PTHREAD_PRIORITY_ERROR) && (t == INVALID_CONTEXT) && (t != kernel_current_context);
		 prio--)
	{
		for(i = 0; (i < thread_ready_list[prio].index) && (t == INVALID_CONTEXT); i++)
		{
			if(thread_ready_list[prio].list[i]->state == PTHREAD_STATE_READY)
			{
				t = thread_ready_list[prio].list[i];
			}
		}
	}

	if(t == INVALID_CONTEXT)
	{
		if(kernel_idle_thread.state == PTHREAD_STATE_READY)
		{
			t = &kernel_idle_thread;
		}
		else
		{
			kernel_panic();
		}
	}

	return t;
}

/** @brief 	Get thread context in RUNNING state (current thread).
 * 	@return Thread context currently in execution.
 */
pthread_t getCurrentContext(void)
{
	return kernel_current_context;
}

/** @brief If there is any error in kernel execution, we'll get here.  */
void kernel_panic(void)
{
	int i = 1;

	arch_critical_section_start();

	while(i)
	{

	}

	arch_critical_section_end();

}

/** @brief Call scheduler and switch context if necessary.
 *  @return 0 on success, -1 on error.
 */
int schedule(void)
{
	int rv = -1;
	pthread_t next_context, current_context;

	arch_critical_section_start();

	if(kernel_current_context->state == PTHREAD_STATE_RUNNING)
	{
		kernel_current_context->state = PTHREAD_STATE_READY;
		if(kernel_current_context != &kernel_idle_thread)
		{
			rv = pthread_add_ready(kernel_current_context);
		}
		else
		{
			rv = 0;
		}
		if(rv != 0)
		{
			kernel_panic();
		}
	}

	next_context = getNextContext();

	if(next_context != &kernel_idle_thread)
	{
		rv = pthread_remove_ready(next_context);
	}
	else
	{
		rv = 0;
	}
	if(rv == 0)
	{
		next_context->state = PTHREAD_STATE_RUNNING;

		if(next_context != kernel_current_context)
		{
			current_context = kernel_current_context;
			kernel_current_context = next_context;

			arch_critical_section_end();

			arch_context_switch(current_context, next_context);
		}
	}

	arch_critical_section_end();

	return rv;
}

/** @brief uPOSIX kernel initialization. */
void kernel_init(void)
{
	static pthread_attr_t main_attr;

	kernel_idle_thread.detachstate = PTHREAD_CREATE_DETACHED;
	kernel_idle_thread.retval = 0;
	kernel_idle_thread.schedparam.sched_curpriority = PTHREAD_PRIORITY_IDLE;
	kernel_idle_thread.schedparam.sched_priority = PTHREAD_PRIORITY_IDLE;
	kernel_idle_thread.schedpolicy = SCHED_RR;
	kernel_idle_thread.stackaddr = 0;
	kernel_idle_thread.sp = 0;
	kernel_idle_thread.state = PTHREAD_STATE_RUNNING;
	kernel_idle_thread.stacksize = PTHREAD_DEFAULT_STACK_SIZE;

	arch_init();
	heap_init();

	main_attr = pthread_default_attr;
	main_attr.detachstate = PTHREAD_CREATE_DETACHED;
	kstate = KERNEL_STATE_THREAD;

	kernel_current_context = &kernel_idle_thread;

#if defined (__REDLIB__)
	extern void __main(void);
	pthread_create(0, &main_attr, (void *(*)(void*))__main, 0);
#else
	extern int main(void);
	pthread_create(0, &main_attr, (void *(*)(void*))main, 0);
#endif

	kernel_idle();
}

/** @brief kernel tick function, normally associated to a timer interrupt. */
void kernel_tick(void)
{
	/* check for detached threads, release resources */
	kernel_remove_detached_threads();
	/* check for blocked threads, update sleep counters */
	kernel_update_sleep_counters();
}

/** @brief 	Thread return function. When a thread returns, this function is
 * 			executed. It takes the thread's return value and ends with a
 * 			scheduler call.
 * 	@param rv	Thread return value.
 */
void thread_return_hook(void * rv)
{
	pthread_t t;

	arch_critical_section_start();

	t = getCurrentContext();

	if(t->detachstate == PTHREAD_CREATE_JOINABLE)
	{
		t->state = PTHREAD_STATE_ZOMBIE;
		t->retval = rv;
	}
	else
	{
		t->state = PTHREAD_STATE_DETACHED;
	}

	arch_critical_section_end();

	schedule();

	kernel_panic();
}

/** @brief 	This call should be called as first statement of all IRQ handlers.
 * 			This call should be used with #kernel_irq_end.
 *  @param s A static variable of type kernel_state_t, where the actual kernel state
 *  		 will be stored.
 *
 *  Use example:
 *  <code>
 *
 *  void IRQ_Handler(void) {
 *
 *  	static kernel_state_t actual_state;
 *
 *  	kernel_irq_start(&actual_state);
 *
 *  	// *** IRQ processing here ***
 *
 *		kernel_irq_end(actual_state);
 *
 *  }
 *  </code>
 */
void kernel_irq_start(kernel_state_t * s)
{
	*s = kstate;
	kstate = KERNEL_STATE_IRQ;
}

/** @brief 	This call should be called as last statement of all IRQ handlers.
 * 			This call should be used with #kernel_irq_start.
 *  @param s A static variable of type kernel_state_t, where the actual kernel state
 *  		 will be restored.
 */
void kernel_irq_end(kernel_state_t s)
{
	kstate = s;
	schedule();
}

/** @brief Get kernel current #kernel_state_t state.
 *  @return Kernel current state.
 */
kernel_state_t kernel_get_state(void)
{
	return kstate;
}

/** @} doxygen end group definition */
/*==================[end of file]============================================*/
