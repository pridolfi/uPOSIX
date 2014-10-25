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

/** @brief uposix_app_thread.c source file
 **
 ** This file demonstrates how to use uPOSIX library for LPCXpresso/LPC4337.
 **
 **/

/** \addtogroup uPOSIX uPOSIX
 ** @{ */
/** \addtogroup Examples Examples
 ** @{ */
/** \addtogroup app4337 Simple thread management application for LPCXpresso/LPC4337 port.
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 * PR           Pablo Ridolfi
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20140925 v0.0.1   PR first version
 */

/*==================[inclusions]=============================================*/

#include "uposix.h"
#include <stdio.h>

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

pthread_t thread1, thread2;
pthread_mutex_t mtx1, mtx2;

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

void * thread1_main(void * a)
{
	printf("thread1\n");
	while(1)
	{
		pthread_mutex_lock(&mtx1);
		printf("x\n");
		pthread_mutex_unlock(&mtx2);
	}
}

void * thread2_main(void * a)
{
	printf("thread2\n");
	while(1)
	{
		pthread_mutex_lock(&mtx2);
		printf("y\n");
		pthread_mutex_unlock(&mtx1);
	}
}

/*==================[external functions definition]==========================*/

/** @brief main function: Application entry point.
 */
int main(void)
{
	printf("uPOSIX pthread test\n");

	pthread_mutex_init(&mtx1, 0);
	pthread_mutex_init(&mtx2, 0);

	pthread_create(&thread1, 0, thread1_main, 0);
	pthread_create(&thread2, 0, thread2_main, 0);

	return 0;

}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
