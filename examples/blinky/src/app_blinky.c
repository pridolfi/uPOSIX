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

/** @brief uposix_app_blinky.c source file
 **
 ** This file demonstrates how to use uPOSIX library for LPCXpresso/LPC1769.
 **
 **/

/** \addtogroup uPOSIX uPOSIX
 ** @{ */
/** \addtogroup Examples Examples
 ** @{ */
/** \addtogroup app1769 Simple blinky application for LPCXpresso/LPC1769 port.
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

#include "app_blinky.h"

/*==================[macros and definitions]=================================*/

/** @brief 	define this with 1 to see an example using pipes, use 0 to see
 * 			an example using mutexes
 */
#define USE_PIPES	1

#define	USE_MUTEX	(!USE_PIPES)


/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/** @brief user callback for systick */
static void mySysTickCallback(void);

/*==================[internal data definition]===============================*/

static devGPIO_pin_t led;

/** @brief File Descriptor for GPIO access.
 */
static int fd_gpio;

/** @brief File Descriptor for SysTick access.
 */
static int fd_systick;

#if(USE_MUTEX)
/** @brief mutex declaration */
pthread_mutex_t mutex;
#elif(USE_PIPES)
/** @brief pipe file descriptors */
int pfds[2];
#endif

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/** @brief User SysTick Callback function (hooked to IRQ handler).
 */
static void mySysTickCallback(void)
{
	static int cont;

	/* execute this every 500ms */
	cont++;
	if(cont >= 500)
	{
#if(USE_PIPES)
		/* write data to pipe */
		cont = write(pfds[1], "blink!", 7);
#elif(USE_MUTEX)
		/* release mutex */
		pthread_mutex_unlock(&mutex);
#endif
		cont = 0;
	}
}

/** @brief Thread example.
 * 	@param arg	Argument (passed from #pthread_create).
 * 	@return value to be returned, then collected by #pthread_join.
 */
void * thread_main(void * arg)
{
	int a = (int)arg;
	int cont = 0;

#if(USE_PIPES)
	char buf[10];
	int rv;
#endif

	while(a)
	{
#if(USE_PIPES)
		/* wait for data from pipe */
		rv = read(pfds[0], buf, 7);
		buf[rv] = 0;
		printf("Received: %s.\n", buf);
#elif(USE_MUTEX)
		/* wait for mutex unlock */
		pthread_mutex_lock(&mutex);
#endif
		/* toggle led */
		ioctl(fd_gpio, devGPIO_REQ_TOGGLE_BIT, &led);

		cont++;
		if(cont==10)
		{
			pthread_exit(0);
		}
	}
	return 0;
}

/*==================[external functions definition]==========================*/

/** @brief main function: Application entry point.
 */
int main(void)
{
	printf("uPOSIX blinky application.\n");

	/* init LPCOpen */
	open("/dev/lpcopen", 0);

	/* init GPIO, get descriptor */
    fd_gpio = open("/dev/gpio", 0);

    /* LED off */
    led = devGPIO_pins[0];
    led.value = 0;
    ioctl(fd_gpio, devGPIO_REQ_WRITE_BIT, &led);

#if(USE_PIPES)
    /* create pipe */
    pipe(pfds);
#elif(USE_MUTEX)
	/* create and lock mutex */
	pthread_mutex_init(&mutex, 0);
	pthread_mutex_lock(&mutex);
#endif

	/* init SysTick */
	fd_systick = open("/dev/systick", 0);

	/* set user callback for systick */
	ioctl(fd_systick, devSysTick_REQ_SET_CALLBACK, (void *)mySysTickCallback);

	/* create thread */
	pthread_t hilo;
	pthread_create(&hilo, 0, thread_main, (void*)0x1234ABCD);

	/* exit main thread */
    return 0xABCDEF98;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
