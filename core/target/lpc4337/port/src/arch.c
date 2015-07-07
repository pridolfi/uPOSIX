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

/** @brief uPOSIX RTOS architecture dependent functions.
 **
 **/

/** \addtogroup uPOSIX uPOSIX
 ** @{ */
/** \addtogroup core1769 Core/LPCXpresso/LPC1769
 *
 * Architecture dependent files (LPCXpresso/LPC1769).
 *
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

#include "string.h"

#include "arch.h"
#include "heap.h"
#include "kernel.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

#define CRP_NO_CRP 0xFFFFFFFF
__attribute__ ((used,section(".crp"))) const unsigned int CRP_WORD = CRP_NO_CRP ;

static int arch_critical_section_nesting = 0;

/*==================[external data definition]===============================*/

/** @brief stores current context, used in context switch (PendSV) */
pthread_t current_context 	= INVALID_CONTEXT;

/** @brief stores next context, used in context switch (PendSV) */
pthread_t next_context 		= INVALID_CONTEXT;

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

/** @brief Initializes the stack frame for a new thread.
 *  @param t				The thread (pthread_t) to be initialized.
 *  @param start_routine	The entry point for the thread.
 *  @param arg				The argument to be passed to the thread.
 *  @return 				Zero on success, -1 on error.
 */
int arch_init_context(pthread_t t,
		void *(*start_routine)(void*),
		void *arg)
{
	int rv = -1;

	if(t->stackaddr)
	{
		memset(t->stackaddr, 0, t->stacksize);

		t->sp = t->stackaddr;

		uint32_t * stack = (uint32_t *)t->stackaddr;

		/* stack frame and SP setup (xPSR, PC, R0, LR) */
		stack[t->stacksize/4-1] = 1<<24; 						/* xPSR.T = 1 */
		stack[t->stacksize/4-2] = (uint32_t)start_routine; 		/* PC <- entry point */
		stack[t->stacksize/4-8] = (uint32_t)arg; 				/* R0 <- arg */
		stack[t->stacksize/4-3] = (uint32_t)thread_return_hook; /* LR <- gancho return*/
		stack[t->stacksize/4-9] = 0xFFFFFFFD; 					/* IRQ return LR */

		stack = (uint32_t *)t->sp;
		stack += t->stacksize/4-17;
		t->sp = stack;

		rv = 0;
	}

	return rv;
}

/** @brief Initializes hardware features needed by uPOSIX RTOS.
 */
void arch_init(void)
{
#if defined (__USE_LPCOPEN)
#if !defined(NO_BOARD_LIB)
    // Read clock settings and update SystemCoreClock variable
    SystemCoreClockUpdate();
    // Set up and initialize all required blocks and
    // functions related to the board hardware
    Board_Init();
    // Set the LED to the state of "On"
    Board_LED_Set(0, false);
#endif
#endif

	/* Activate MemFault, UsageFault and BusFault exceptions */
	SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk | SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk;

	/* Set lowest priority for SysTick and PendSV */
	NVIC_SetPriority(PendSV_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

	/* Activate SysTick */
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);
}

/** @brief Architecture context-switch function (sets PendSV Exception for Cortex-M processors).
 * 	@param curr_ctx Current thread context to be saved. If curr_ctx is 0, current context will
 * 					not be saved.
 *  @param next_ctx	Next context to be loaded.
 */
void arch_context_switch(pthread_t curr_ctx, pthread_t next_ctx)
{
	if(next_ctx != INVALID_CONTEXT)
	{
		current_context = curr_ctx;
		next_context = next_ctx;

		/* Set PendSV */
		SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
	}
}

/** @brief architecture halt CPU: executes Wait For Interrupt instruction. */
void arch_halt_cpu(void)
{
	__WFI();
}

/** @brief Critical section start. Disables IRQs. */
void arch_critical_section_start(void)
{
	__disable_irq();
	arch_critical_section_nesting++;
}

/** @brief Critical section end. Enables IRQs (depends on nesting status). */
void arch_critical_section_end(void)
{
	if(arch_critical_section_nesting > 0)
	{
		arch_critical_section_nesting--;
	}
	if(arch_critical_section_nesting == 0)
	{
		__enable_irq();
	}
}

/** @brief Check for stack overflow.
 *  @param sp	Current Stack Pointer.
 */
void arch_check_stack_overflow(uint32_t * sp)
{
	if(current_context != 0)
	{
		if(sp < (uint32_t *)current_context->stackaddr)
		{
			kernel_panic();
		}
	}
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
