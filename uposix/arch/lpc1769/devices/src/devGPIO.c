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

/** @brief This file implements access methods to LPC1769 GPIO peripheral.
 **
 **/

/** \addtogroup uPOSIX uPOSIX
 ** @{ */
/** \addtogroup LPC1769 LPC1769 Port
 ** @{ */
/** \addtogroup GPIO GPIO
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

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

static int devGPIO_open(const char * path, int flags);
static int devGPIO_read(int fd, void * buf, int len);
static int devGPIO_write(int fd, const void * buf, int len);
static int devGPIO_close(int fd);
static int devGPIO_ioctl(int fd, int req, void * param);

/*==================[internal data definition]===============================*/

/**
 * @brief Store the current port number used in read and write functions.
 */
static uint32_t devGPIO_portNum = 0;

/*==================[external data definition]===============================*/

/**
 * @brief device struct for GPIO
 */
const device_t devGPIO =
{
		"gpio",				/**< path (relative to /dev/)  	*/
		(void *)LPC_GPIO,	/**< peripheral base address 	*/
		devGPIO_open,		/**< pointer to open function	*/
		devGPIO_read,		/**< pointer to read function  	*/
		devGPIO_write,		/**< pointer to write function 	*/
		devGPIO_close,		/**< pointer to close function 	*/
		devGPIO_ioctl		/**< pointer to ioctl function 	*/
};

/**
 * @defgroup devGPIOgroup1 Common GPIO
 * @brief Some #devGPIO_pin_t common pins for LPC1769 Stick, BaseBoard and RubenBoard.
 * @{ */
/* LPCXpresso LPC1769 Stick */
const devGPIO_pin_t devGPIO_LPCXpresso1769_LED		= {0, 22, 1};

/* RubenBoard 2 (red) */
/* Outputs */
const devGPIO_pin_t devGPIO_RubenBoardRed_LED_BLUE 	= {2,  2, 1};
const devGPIO_pin_t devGPIO_RubenBoardRed_LED_RED	= {2,  4, 1};
const devGPIO_pin_t devGPIO_RubenBoardRed_LED_GREEN	= {2,  3, 1};
/* Inputs */
const devGPIO_pin_t devGPIO_RubenBoardRed_SW2		= {0, 18, 0};
const devGPIO_pin_t devGPIO_RubenBoardRed_SW3		= {0,  1, 0};

/* LPCXpresso BaseBoard rev. B */
/* Inputs */
const devGPIO_pin_t devGPIO_BaseBoardRevB_SW3		= {2, 10, 0};
const devGPIO_pin_t devGPIO_BaseBoardRevB_SW4		= {1, 31, 0};
const devGPIO_pin_t devGPIO_BaseBoardRevB_JOY_UP	= {2,  3, 0};
const devGPIO_pin_t devGPIO_BaseBoardRevB_JOY_DOWN	= {0, 15, 0};
const devGPIO_pin_t devGPIO_BaseBoardRevB_JOY_LEFT	= {2,  4, 0};
const devGPIO_pin_t devGPIO_BaseBoardRevB_JOY_RIGHT	= {0, 16, 0};
const devGPIO_pin_t devGPIO_BaseBoardRevB_JOY_PRESS	= {0, 17, 0};
/* Outputs */
const devGPIO_pin_t devGPIO_BaseBoardRevB_LED_RED 	= {2,  0, 1};
const devGPIO_pin_t devGPIO_BaseBoardRevB_LED_GREEN	= {2,  1, 1};
const devGPIO_pin_t devGPIO_BaseBoardRevB_LED_BLUE 	= {0, 26, 1};
/** @} */


/*==================[internal functions definition]==========================*/

/** @brief This function initializes GPIO peripheral for LPC1769.
 *
 * @param path	String containing the path to the device, should be "/dev/gpio".
 * 				Not used.
 * @param flags	Access flags, device dependent.	Not used.
 * @return 		Always zero.
 */
static int devGPIO_open(const char * path, int flags)
{
	Chip_GPIO_Init(LPC_GPIO);
	Chip_IOCON_Init(LPC_IOCON);

	return 0;
}

/** @brief 	This function reads GPIO ports (Ports 0 through len -max 4-).
 *
 * @param fd	Device file descriptor, returned by open.
 * @param buf	Buffer where port data will be stored.
 * @param len	Max port to read (0 through 4).
 * @return 		Number of bytes actually read, normally len, or -1 in case of an error.
 *
 */
static int devGPIO_read(int fd, void * buf, int len)
{
	int rv = -1;
	int i;

	if((len <= 4)&&(devList[fd]==&devGPIO))
	{
		for(i=0; i<=len; i++)
		{
			((uint32_t*)buf)[i] = Chip_GPIO_ReadValue(LPC_GPIO, i);
		}
		rv = len;
	}

	return rv;
}

/** @brief 	This function writes an entire GPIO port. Use ioctl #devGPIO_REQ_SET_PORT
 * 			to select the port to be read. Port 0 is selected by default.
 *
 * @param fd	Device file descriptor, returned by open.
 * @param buf	Buffer pointing to the data to be written.
 * @param len	Buffer length, should be 4 bytes (32 bits).
 * @return 		Number of bytes actually written (4 bytes) or -1 in case of an error.
 *
 */
static int devGPIO_write(int fd, const void * buf, int len)
{
	int rv = -1;

	if((len >= sizeof(uint32_t))&&(devList[fd]==&devGPIO))
	{
		Chip_GPIO_SetPortValue(LPC_GPIO, devGPIO_portNum, *((uint32_t *)buf));
		rv = sizeof(uint32_t);
	}

	return rv;
}

/** @brief This function deactivates GPIO.
 *
 * @param fd	Device file descriptor, returned by open.
 * @return 		Device dependent, normally 0 on success.
 *
 */
static int devGPIO_close(int fd)
{
	Chip_GPIO_DeInit(LPC_GPIO);

	return 0;
}

/** @brief This function is used to interact with GPIO peripheral.
 *
 * @param fd	Device file descriptor, returned by open.
 * @param req	ioctl request defined in #devGPIO_ioctl_requests, should be:
 *		- #devGPIO_REQ_READ_BIT:	Read a single bit specified in the #devGPIO_pin_t
 *									structure passed as 3rd argument.
 *		- #devGPIO_REQ_WRITE_BIT: 	Write a single bit specified in the #devGPIO_pin_t
 *									structure passed as 3rd argument.
 *		- #devGPIO_REQ_TOGGLE_BIT:	Toggle a single bit specified in the #devGPIO_pin_t
 *									structure passed as 3rd argument.
 *		- #devGPIO_REQ_WRITE_DIR:	Set direction for specified bit in 3rd argument. The
 *									value field of the #devGPIO_pin_t structure should be
 *									0 (input) or 1 (output).
 *		- #devGPIO_REQ_SET_PORT:	Set current port number to be used with read and write
 *									functions.
 *		- #devGPIO_REQ_SET_FUNC:	Set function for pin specified in #devGPIO_pin_t passed
 *									as 3rd argument. <b>value</b> field should be a combination
 *									of IOCON_MODE_* and IOCON_FUNC* macros: <b>IOCON_FUNC0,
 *									IOCON_FUNC1, IOCON_FUNC2, IOCON_FUNC3</b> and
 *									<b>IOCON_MODE_INACT, IOCON_MODE_PULLDOWN, IOCON_MODE_PULLUP,
 *									IOCON_MODE_REPEATER</b>.
 *
 * @param param	Depends on the request, normally you will use a #devGPIO_pin_t pointer
 * 				to select a bit to interact with.
 * @return 		Zero if request completed successfully, -1 on error.
 *
 */
static int devGPIO_ioctl(int fd, int req, void * param)
{
	int rv = -1;
	devGPIO_pin_t * pin = (devGPIO_pin_t *)param;

	if(devList[fd] == &devGPIO)
	{
		switch(req)
		{
			case devGPIO_REQ_READ_BIT:
				pin->value = Chip_GPIO_GetPinState(LPC_GPIO, pin->port, pin->bit);
				rv = 0;
				break;

			case devGPIO_REQ_WRITE_BIT:
				Chip_GPIO_WritePortBit(LPC_GPIO, pin->port, pin->bit, pin->value);
				rv = 0;
				break;

			case devGPIO_REQ_SET_PORT:
				devGPIO_portNum = pin->port;
				rv = 0;
				break;

			case devGPIO_REQ_WRITE_DIR:
				Chip_GPIO_WriteDirBit(LPC_GPIO, pin->port, pin->bit, pin->value);
				rv = 0;
				break;

			case devGPIO_REQ_TOGGLE_BIT:
				Chip_GPIO_SetPinToggle(LPC_GPIO, pin->port, pin->bit);
				rv = 0;
				break;

			case devGPIO_REQ_SET_FUNC:
				Chip_IOCON_PinMux(LPC_IOCON, pin->port, pin->bit, pin->value & 0x6, pin->value & 0x3);
				rv = 0;
				break;

			default:
				break;
		}
	}
	return rv;
}

/*==================[external functions definition]==========================*/

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
