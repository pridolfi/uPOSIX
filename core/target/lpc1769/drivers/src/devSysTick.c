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

/** @brief This file implements access methods to SysTick peripheral.
 **
 **/

/** \addtogroup uPOSIX uPOSIX
 ** @{ */
/** \addtogroup LPC1769 LPC1769 Port
 ** @{ */
/** \addtogroup SysTick SysTick
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
#include "devSysTick.h"

#include "string.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

static int devSysTick_open (const device_t * const dev, int flags);
static int devSysTick_read (const device_t * const dev, void * buf, int len);
static int devSysTick_write(const device_t * const dev, const void * buf, int len);
static int devSysTick_close(const device_t * const dev);
static int devSysTick_ioctl(const device_t * const dev, int req, void * param);

/*==================[internal data definition]===============================*/

/**
 * @brief Store the current user callback for SysTick IRQ.
 */
static devSysTick_callback_t userCallback = 0;

/*==================[external data definition]===============================*/

/** @brief SysTick file operations */
const fops_t devSysTick_fops =
{
		devSysTick_open,		/**< pointer to open function  */
		devSysTick_read,		/**< pointer to read function  */
		devSysTick_write,		/**< pointer to write function */
		devSysTick_close,		/**< pointer to close function */
		devSysTick_ioctl		/**< pointer to ioctl function */
};

/**
 * @struct devSysTick
 * @brief Device struct for SysTick.
 */
const device_t devSysTick =
{
		"systick",				/**< path (relative to /dev/)  */
		(void *)SysTick,		/**< peripheral base address   */
		&devSysTick_fops,
};

/*==================[internal functions definition]==========================*/

/** @brief This function initializes the SysTick timer.
 *
 * @param dev	Device structure.
 * @param flags	Not used.
 * @return 		Zero if the initialization was correct, -1 otherwise.
 */
static int devSysTick_open(const device_t * const dev, int flags)
{
#ifndef __USE_UPOSIX_RTOS /* SysTick is used by the uPOSIX RTOS */
	int rv = -1;
	rv = SysTick_Config(SystemCoreClock/1000);
	if(rv)
	{
		rv = -1;
	}
	return rv;
#else
	return 0;
#endif
}

/** @brief 	This function reads SysTick current value register.
 *
 * @param dev	Device structure.
 * @param buf	Buffer where port data will be stored.
 * @param len	Bytes to be read, should be 4 or more.
 * @return 		Number of bytes actually read, normally 4, or -1 in case of an error.
 */
static int devSysTick_read(const device_t * const dev, void * buf, int len)
{
	int rv = -1;
#ifndef __USE_UPOSIX_RTOS
	if((len >= sizeof(uint32_t)) && (SysTick == dev->ptr))
	{
		memcpy(buf, (void *)&(SysTick->VAL), sizeof(uint32_t));
		rv = sizeof(uint32_t);
	}
#endif
	return rv;
}

/** @brief 	This function writes SysTick current value register.
 *
 * @param dev	Device structure.
 * @param buf	Buffer pointing to the data to be written.
 * @param len	Buffer length, should be 4 bytes (32 bits).
 * @return 		Number of bytes actually written (4 bytes) or -1 in case of an error.
 *
 */
static int devSysTick_write(const device_t * const dev, const void * buf, int len)
{
	int rv = -1;
#ifndef __USE_UPOSIX_RTOS
	if((len >= sizeof(uint32_t)) && (SysTick == dev->ptr))
	{
		memcpy((void *)&(SysTick->VAL), buf, sizeof(uint32_t));
		rv = sizeof(uint32_t);
	}
#endif
	return rv;
}

/** @brief This function deactivates SysTick.
 *
 * @param dev	Device structure.
 * @return 		Device dependent, normally 0 on success.
 *
 */
static int devSysTick_close(const device_t * const dev)
{
	int rv = -1;
#ifndef __USE_UPOSIX_RTOS
	if(SysTick == dev->ptr)
	{
		SysTick->CTRL = 0;
		SysTick->LOAD = 0;
		SysTick->VAL  = 0;

		userCallback = 0;

		rv = 0;
	}
#endif
	return rv;
}

/** @brief This function is used to interact with SysTick.
 *
 * @param dev	Device structure.
 * @param req	ioctl request defined in #devSysTick_ioctl_requests, should be:
 *				- #devSysTick_REQ_SET_TICKS:	Restarts SysTick with a new interrupt
 *												frequency.
 *				- #devSysTick_REQ_SET_CALLBACK:	Sets a new callback for SysTick IRQ handler.
 *
 * @param param	Depends on the request:
 * 				- If <b>req</b> is #devSysTick_REQ_SET_TICKS, <b>param</b> should be a 32-bit unsigned
 * 				  integer (number of ticks to reload).
 * 				- If <b>req</b> is #devSysTick_REQ_SET_CALLBACK, <b>param</b> should be a
 * 				  #devSysTick_callback_t callback function to be used hooked to SysTick IRQ handler.
 *
 * @return 		Zero if request completed successfully, -1 on error.
 *
 */
static int devSysTick_ioctl(const device_t * const dev, int req, void * param)
{
	int rv = -1;
	if(SysTick == dev->ptr)
	{
		switch(req)
		{
			case devSysTick_REQ_SET_CALLBACK:
				userCallback = (devSysTick_callback_t)param;
				rv = 0;
				break;
			case devSysTick_REQ_SET_TICKS:
#ifndef __USE_UPOSIX_RTOS
				if(SysTick_Config((uint32_t)param)==0)
				{
					rv = 0;
				}
#endif
				break;
			default:
				break;
		}
	}
	return rv;
}

/*==================[external functions definition]==========================*/

/** @brief SysTick global IRQ handler (as defined in CMSIS).
 */
void SysTick_Handler(void)
{
#ifdef __USE_UPOSIX_RTOS
	static kernel_state_t s;
	kernel_irq_start(&s);

	/* kernel timer tick handler */
	kernel_tick();
#endif

	/* user application tick handler */
	if(userCallback)
	{
		userCallback();
	}

	/* device driver tick handlers */
	devListExecuteHandlers();

#ifdef __USE_UPOSIX_RTOS
	kernel_irq_end(s);
#endif

}
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
