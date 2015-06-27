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

#ifndef KERNEL_H_
#define KERNEL_H_

/** @brief Main RTOS kernel header file.
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
#include "errno.h"

#include "pthread.h"

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

/** @brief Invalid context definition. */
#define INVALID_CONTEXT ((pthread_t)0)

/** @brief 	Macro for standard function sched_yield().
 *  		Causes the calling thread to relinquish the CPU. The thread is
 *  		moved to the end of the queue for its static priority and a new
 *  		thread gets to run.
 */
#define sched_yield() schedule()

/*==================[typedef]================================================*/

typedef enum
{
	KERNEL_STATE_ERROR,
	KERNEL_STATE_SYSTEM,
	KERNEL_STATE_THREAD,
	KERNEL_STATE_IRQ
}kernel_state_t;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

void kernel_panic(void);

int schedule(void);

void kernel_tick(void);

void kernel_init(void);

void thread_return_hook(void *);

pthread_t getCurrentContext(void);

pthread_t getNextContext(void);

void kernel_irq_start(kernel_state_t * s);

void kernel_irq_end(kernel_state_t s);

kernel_state_t kernel_get_state(void);

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* KERNEL_H_ */
