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

static int devGPIO_open (const device_t * const dev, int flags);
static int devGPIO_read (const device_t * const dev, void * buf, int len);
static int devGPIO_write(const device_t * const dev, const void * buf, int len);
static int devGPIO_close(const device_t * const dev);
static int devGPIO_ioctl(const device_t * const dev, int req, void * param);

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/**
 * @brief file operations for GPIO
 */
const fops_t devGPIO_fops =
{
		devGPIO_open,		/**< pointer to open function	*/
		devGPIO_read,		/**< pointer to read function  	*/
		devGPIO_write,		/**< pointer to write function 	*/
		devGPIO_close,		/**< pointer to close function 	*/
		devGPIO_ioctl		/**< pointer to ioctl function 	*/
};

/**
 * @brief device struct for GPIO
 */
const device_t devGPIO =
{
		"gpio",				/**< device name (relative to /dev/)  	*/
		(void *)LPC_GPIO,	/**< peripheral base address 			*/
		&devGPIO_fops		/**< file operations */
};

/**
 * @brief Some #devGPIO_pin_t common pins for LPC1769 Stick, BaseBoard and RubenBoard.
 */
const devGPIO_pin_t devGPIO_pins[] =
{
	/* Outputs */
	{0, 22, 1}, /* LPCXpresso Stick LED */
	{2,  2, 1}, /* RubenBoard 2 (red) LED Blue */
	{2,  4, 1}, /* RubenBoard 2 (red) LED Red */
	{2,  3, 1}, /* RubenBoard 2 (red) LED Green */
	{2,  0, 1}, /* LPCXpresso BaseBoard Rev. B LED Red */
	{2,  1, 1}, /* LPCXpresso BaseBoard Rev. B LED Green */
	{0, 26, 1}, /* LPCXpresso BaseBoard Rev. B LED Blue */
	/* Inputs */
	{0, 18, 0}, /* RubenBoard 2 (red) SW2 */
	{0,  1, 0}, /* RubenBoard 2 (red) SW3 */
	{2, 10, 0}, /* LPCXpresso BaseBoard Rev. B SW3 */
	{1, 31, 0}, /* LPCXpresso BaseBoard Rev. B SW4 */
	{2,  3, 0}, /* LPCXpresso BaseBoard Rev. B JOY UP */
	{0, 15, 0}, /* LPCXpresso BaseBoard Rev. B JOY DOWN */
	{2,  4, 0}, /* LPCXpresso BaseBoard Rev. B JOY LEFT */
	{0, 16, 0}, /* LPCXpresso BaseBoard Rev. B JOY RIGHT */
	{0, 17, 0}, /* LPCXpresso BaseBoard Rev. B JOY PRESS */
};

/** @brief managed gpio pin count */
const uint32_t devGPIO_pins_count = sizeof(devGPIO_pins)/sizeof(devGPIO_pin_t);

/*==================[internal functions definition]==========================*/

/** @brief This function initializes GPIO peripheral for LPC1769.
 *
 * @param dev	Device structure.
 * @param flags	Access flags, device dependent.	Not used.
 * @return 		Always zero.
 */
static int devGPIO_open(const device_t * const dev, int flags)
{
	uint32_t i;

	Chip_GPIO_Init(dev->ptr);
	Chip_IOCON_Init(LPC_IOCON);

	for(i=0; i<devGPIO_pins_count; i++)
	{
		Chip_GPIO_WriteDirBit(dev->ptr,
			devGPIO_pins[i].port,
			devGPIO_pins[i].bit,
			devGPIO_pins[i].value);
		if(devGPIO_pins[i].value == 1)
		{
			Chip_GPIO_SetPinState(dev->ptr,
				devGPIO_pins[i].port,
				devGPIO_pins[i].bit,
				0);
		}
	}
	return 0;
}

/** @brief 	This function reads GPIO ports (Ports 0 through len -max 4-).
 *
 * @param dev	Device structure.
 * @param buf	Buffer where port data will be stored.
 * @param len	Max port to read (0 through 4).
 * @return 		Number of bytes actually read, normally len, or -1 in case of an error.
 */
static int devGPIO_read(const device_t * const dev, void * buf, int len)
{
	int rv = -1;
	int i;

	devGPIO_pin_t * p = (devGPIO_pin_t *)buf;

	if((len <= devGPIO_pins_count)&&(dev->ptr == LPC_GPIO))
	{
		for(i=0; i<len; i++)
		{
			p[i].value = Chip_GPIO_GetPinState(dev->ptr, p[i].port, p[i].bit);
		}
		rv = len;
	}

	return rv;
}

/** @brief 	This function writes an entire GPIO port. Use ioctl #devGPIO_REQ_SET_PORT
 * 			to select the port to be read. Port 0 is selected by default.
 *
 * @param dev	Device structure.
 * @param buf	Buffer pointing to the data to be written.
 * @param len	Buffer length, should be 4 bytes (32 bits).
 * @return 		Number of bytes actually written (4 bytes) or -1 in case of an error.
 *
 */
static int devGPIO_write(const device_t * const dev, const void * buf, int len)
{
	int rv = -1;
	int i;

	devGPIO_pin_t * p = (devGPIO_pin_t *)buf;

	if((len <= devGPIO_pins_count)&&(dev->ptr == LPC_GPIO))
	{
		for(i=0; i<len; i++)
		{
			Chip_GPIO_SetPinState(dev->ptr, p[i].port, p[i].bit, p[i].value);
		}
		rv = len;
	}

	return rv;
}

/** @brief This function deactivates GPIO.
 *
 * @param dev	Device structure.
 * @return 		Device dependent, normally 0 on success.
 *
 */
static int devGPIO_close(const device_t * const dev)
{
	int rv = -1;

	if(dev->ptr == LPC_GPIO)
	{
		Chip_GPIO_DeInit(dev->ptr);
	}

	return rv;
}

/** @brief This function is used to interact with GPIO peripheral.
 *
 * @param dev	Device structure.
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
static int devGPIO_ioctl(const device_t * const dev, int req, void * param)
{
	int rv = -1;
	devGPIO_pin_t * pin = (devGPIO_pin_t *)param;

	if(dev->ptr == LPC_GPIO)
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
