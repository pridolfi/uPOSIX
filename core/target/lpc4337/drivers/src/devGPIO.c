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

/** @brief GPIO device driver.
 **/

/** \addtogroup uPOSIX uPOSIX
 ** @{ */
/** \addtogroup LPC4337	LPC4337 Port
 ** @{ */
/** \addtogroup Template Template
 * 	devTemplate.c and devTemplate.h are template files you can use to define
 * 	your own peripheral driver. You need to replace <b>devTemplate</b> with
 * 	your device name, for example <b>devUART</b> or <b>devSPI</b>.
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
#include "devGPIO.h"

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
		"gpio",					/**< device name (relative to /dev/)  	*/
		(void *)LPC_GPIO_PORT,	/**< peripheral base address 			*/
		&devGPIO_fops			/**< file operations */
};

/**
 * @brief Some #devGPIO_pin_t common pins for EDU-CIAA-NXP.
 */
const devGPIO_pin_t devGPIO_pins[] =
{
	/* Outputs */
	{0, 14, 1}, /* EDU-CIAA-NXP LED1 */
	{5,  2, 1}, /* EDU-CIAA-NXP LED Blue */
	{5,  0, 1}, /* EDU-CIAA-NXP LED Red */
	{5,  1, 1}, /* EDU-CIAA-NXP LED Green */
	{1, 11, 1}, /* EDU-CIAA-NXP LED2 */
	{1, 12, 1}, /* EDU-CIAA-NXP LED3 */
	/* Inputs */
	{0,  4, 0}, /* EDU-CIAA-NXP SW1 */
	{0,  8, 0}, /* EDU-CIAA-NXP SW2 */
	{0,  9, 0}, /* EDU-CIAA-NXP SW3 */
	{1,  9, 0}, /* EDU-CIAA-NXP SW4 */
};

/** @brief managed gpio pin count */
const uint32_t devGPIO_pins_count = sizeof(devGPIO_pins)/sizeof(devGPIO_pin_t);

/*==================[internal functions definition]==========================*/

/** @brief This function initializes GPIO peripheral for LPC4337.
 *
 * @param dev	Device structure.
 * @param flags	Access flags, device dependent.	Not used.
 * @return 		Always zero.
 */
static int devGPIO_open(const device_t * const dev, int flags)
{
	Chip_GPIO_Init(dev->ptr);

	/* LEDs */
   Chip_SCU_PinMux(2,0,MD_PUP,FUNC4);  /* GPIO5[0], LED0R */
   Chip_SCU_PinMux(2,1,MD_PUP,FUNC4);  /* GPIO5[1], LED0G */
   Chip_SCU_PinMux(2,2,MD_PUP,FUNC4);  /* GPIO5[2], LED0B */
   Chip_SCU_PinMux(2,10,MD_PUP,FUNC0); /* GPIO0[14], LED1 */
   Chip_SCU_PinMux(2,11,MD_PUP,FUNC0); /* GPIO1[11], LED2 */
   Chip_SCU_PinMux(2,12,MD_PUP,FUNC0); /* GPIO1[12], LED3 */

   Chip_GPIO_SetDir(dev->ptr, 5,(1<<0)|(1<<1)|(1<<2),1);
   Chip_GPIO_SetDir(dev->ptr, 0,(1<<14),1);
   Chip_GPIO_SetDir(dev->ptr, 1,(1<<11)|(1<<12),1);

   Chip_GPIO_ClearValue(dev->ptr, 5,(1<<0)|(1<<1)|(1<<2));
   Chip_GPIO_ClearValue(dev->ptr, 0,(1<<14));
   Chip_GPIO_ClearValue(dev->ptr, 1,(1<<11)|(1<<12));

   /* Switches */
   Chip_SCU_PinMux(1,0,MD_PUP|MD_EZI|MD_ZI,FUNC0); /* GPIO0[4], SW1 */
   Chip_SCU_PinMux(1,1,MD_PUP|MD_EZI|MD_ZI,FUNC0); /* GPIO0[8], SW2 */
   Chip_SCU_PinMux(1,2,MD_PUP|MD_EZI|MD_ZI,FUNC0); /* GPIO0[9], SW3 */
   Chip_SCU_PinMux(1,6,MD_PUP|MD_EZI|MD_ZI,FUNC0); /* GPIO1[9], SW4 */

   Chip_GPIO_SetDir(dev->ptr, 0,(1<<4)|(1<<8)|(1<<9),0);
   Chip_GPIO_SetDir(dev->ptr, 1,(1<<9),0);

	return 0;
}

/** @brief 	This function reads GPIO ports
 */
static int devGPIO_read(const device_t * const dev, void * buf, int len)
{
	int rv = -1;
	int i;

	devGPIO_pin_t * p = (devGPIO_pin_t *)buf;

	if((len <= devGPIO_pins_count)&&(dev->ptr == LPC_GPIO_PORT))
	{
		for(i=0; i<len; i++)
		{
			p[i].value = Chip_GPIO_GetPinState(dev->ptr, p[i].port, p[i].bit);
		}
		rv = len;
	}

	return rv;
}

/** @brief 	This function writes an array of devGPIOpin_t variables
 */
static int devGPIO_write(const device_t * const dev, const void * buf, int len)
{
	int rv = -1;
	int i;

	devGPIO_pin_t * p = (devGPIO_pin_t *)buf;

	if((len <= devGPIO_pins_count)&&(dev->ptr == LPC_GPIO_PORT))
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
	Chip_GPIO_DeInit(dev->ptr);

	return 0;
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
	devGPIO_pin_t * p = (devGPIO_pin_t *)param;

	if(LPC_GPIO_PORT == dev->ptr)
	{
		switch(req)
		{
			case devGPIO_REQ_READ_BIT:
				p->value = Chip_GPIO_GetPinState(dev->ptr, p->port, p->bit);
				rv = 0;
				break;

			case devGPIO_REQ_WRITE_BIT:
				Chip_GPIO_WritePortBit(dev->ptr, p->port, p->bit, p->value);
				rv = 0;
				break;

			case devGPIO_REQ_WRITE_DIR:
				Chip_GPIO_WriteDirBit(dev->ptr, p->port, p->bit, p->value);
				rv = 0;
				break;

			case devGPIO_REQ_TOGGLE_BIT:
				Chip_GPIO_SetPinToggle(dev->ptr, p->port, p->bit);
				rv = 0;
				break;

			case devGPIO_REQ_SET_FUNC:
				Chip_SCU_PinMux(p->port, p->bit, p->value & ~0x3, p->value & 0x3);
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
