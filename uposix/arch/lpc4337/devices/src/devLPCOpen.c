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

/** @brief This file implements some methods in order to interact with LPCOpen library.
 **
 **/

/** \addtogroup uPOSIX uPOSIX
 ** @{ */
/** \addtogroup LPC4337 LPC4337 Port
 ** @{ */
/** \addtogroup LPCOpen LPCOpen
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
#include "devList.h"
#include "devLPCOpen.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

static int devLPCOpen_open (const device_t * const dev, int flags);
static int devLPCOpen_read (const device_t * const dev, void * buf, int len);
static int devLPCOpen_write(const device_t * const dev, const void * buf, int len);
static int devLPCOpen_close(const device_t * const dev);
static int devLPCOpen_ioctl(const device_t * const dev, int req, void * param);

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

const fops_t devLPCOpen_fops =
{
		devLPCOpen_open,		/* pointer to open function  */
		devLPCOpen_read,		/* pointer to read function  */
		devLPCOpen_write,		/* pointer to write function */
		devLPCOpen_close,		/* pointer to close function */
		devLPCOpen_ioctl		/* pointer to ioctl function */
};

const device_t devLPCOpen =		/* device struct for LPCOpen */
{
		"lpcopen",				/* path (relative to /dev/)  */
		(void *)0,				/* peripheral base address 	 */
		&devLPCOpen_fops,
};

/*==================[internal functions definition]==========================*/

/** @brief This function initializes LPCOpen library.
 *
 * @param path	String containing the path to the device, should be "/dev/lpcopen".
 * 				Not used.
 * @param flags	Access flags. Not used.
 * @return 		Zero if initialization was ok, -1 on error.
 */
static int devLPCOpen_open(const device_t * const dev, int flags)
{
#ifndef __USE_UPOSIX_RTOS
#if defined (__USE_LPCOPEN)
#if !defined(NO_BOARD_LIB)
    // Read clock settings and update SystemCoreClock variable
    SystemCoreClockUpdate();
    // Set up and initialize all required blocks and
    // functions related to the board hardware
    Board_Init();
    // Set the LED to the state of "On"
    Board_LED_Set(0, true);
#endif
#endif
#endif
    return 0;

}

/** @brief 	read function is not used in LPCOpen.
 *
 * @param fd	Not used.
 * @param buf	Not used.
 * @param len	Not used.
 * @return 		Always -1.
 */
static int devLPCOpen_read(const device_t * const dev, void * buf, int len)
{
	return -1;
}

/** @brief 	write function is not used in LPCOpen.
 *
 * @param fd	Not used.
 * @param buf	Not used.
 * @param len	Not used.
 * @return 		Always -1.
 */
static int devLPCOpen_write(const device_t * const dev, const void * buf, int len)
{
	return -1;
}

/** @brief 	close function is not used in LPCOpen.
 *
 * @param fd	Not used.
 * @return 		Always -1.
 */
static int devLPCOpen_close(const device_t * const dev)
{
	return -1;
}

/** @brief 	ioctl function, used to get information from LPCOpen.
 *
 * @param fd	Device file descriptor, returned by open.
 *
 * @param req	ioctl request defined in #devGPIO_ioctl_requests, should be:
 *		- #devLPCOpen_REQ_GET_SYSTEMCORECLOCK:	Read CMSIS SystemCoreClock variable.
 *
 * @param param	if req is devLPCOpen_REQ_GET_SYSTEMCORECLOCK, param should be a pointer
 * 				to an uint32_t variable where SystemCoreClock will be stored.
 *
 * @return 		Zero if request completed successfully, -1 on error.
 */
static int devLPCOpen_ioctl(const device_t * const dev, int req, void * param)
{
	int rv = -1;

	switch(req)
	{
		case devLPCOpen_REQ_GET_SYSTEMCORECLOCK:
			*(uint32_t *)param = SystemCoreClock;
			rv = 0;
			break;
		default:
			break;
	}

	return rv;
}

/*==================[external functions definition]==========================*/

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
