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

#ifndef DEVGPIO_H_
#define DEVGPIO_H_

/** @brief This header defines a generic device. It can be used to define new peripherals.
 ** Replace this sentence with a brief description of the peripheral.
 **
 **/

/** \addtogroup uPOSIX uPOSIX
 ** @{ */
/** \addtogroup GenericPort Generic Port
 ** @{ */
/** \addtogroup Template Template
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

#include "device.h"

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

/**
 * @brief This structure defines a port number, bit number and a value, to be
 * used with devGPIO functions.
 */
typedef struct
{
	/*@{*/
	uint32_t port;	/**< Port number */
	uint32_t bit;	/**< Bit number */
	uint32_t value;	/**< Value */
	/*@}*/
}devGPIO_pin_t;

/**
 * @var	devGPIO_ioctl_requests
 * @brief ioctl requests for #devGPIO.
 */
typedef enum
{
		devGPIO_REQ_READ_BIT,	/**< Read a single bit. */
		devGPIO_REQ_WRITE_BIT,	/**< Write a single bit. */
		devGPIO_REQ_TOGGLE_BIT,	/**< Toggle a single bit. */
		devGPIO_REQ_WRITE_DIR,	/**< Set direction for a single pin. */
		devGPIO_REQ_SET_FUNC	/**< Set pin function. */
}devGPIO_ioctl_requests;

/*==================[external data declaration]==============================*/

extern const device_t devGPIO;

/* Pins */
extern const devGPIO_pin_t devGPIO_pins[];

/* Number of pins */
extern const uint32_t devGPIO_pins_count;


/*==================[external functions declaration]=========================*/

/*
 * NOTE: You shouldn't need to define any external function. You have to use
 * the standard functions pointed by your device_t structure.
 */

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* DEVGPIO_H_ */
