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

/** @brief This file describes a template file. You can copy/paste this format to create
 **	your own device drivers.
 **/

/** \addtogroup uPOSIX uPOSIX
 ** @{ */
/** \addtogroup GenericPort Generic Port
 ** @{ */
/** \addtogroup Template Template
 * 	devTemplate.c and devTemplate.h are template files you can use to define your own
 * 	peripheral driver. You need to replace <b>devTemplate</b> with your device name,
 * 	for example <b>devUART</b> or <b>devSPI</b>.
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
#include "devTemplate.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

static int devTemplate_open(const char * path, int flags);
static int devTemplate_read(int fd, void * buf, int len);
static int devTemplate_write(int fd, const void * buf, int len);
static int devTemplate_close(int fd);
static int devTemplate_ioctl(int fd, int req, void * param);

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/** @brief Template device struct. */
const device_t devTemplate =
{
		"template",				/**< path (relative to /dev/)  */
		(void *)0,				/**< peripheral base address   */
		devTemplate_open,		/**< pointer to open function  */
		devTemplate_read,		/**< pointer to read function  */
		devTemplate_write,		/**< pointer to write function */
		devTemplate_close,		/**< pointer to close function */
		devTemplate_ioctl		/**< pointer to ioctl function */
};

/*==================[internal functions definition]==========================*/

/** @brief Generic device open function.
 */
static int devTemplate_open(const char * path, int flags)
{
	int rv = -1;

	/* TODO: Add your device initialization code here */

	return rv;
}

/** @brief Generic device read function.
 */
static int devTemplate_read(int fd, void * buf, int len)
{
	int rv = -1;

	/* TODO: Add your device read access code here */

	return rv;
}

/** @brief Generic device write function.
 */
static int devTemplate_write(int fd, const void * buf, int len)
{
	int rv = -1;

	/* TODO: Add your device write access code here */

	return rv;
}

/** @brief Generic device close function.
 */
static int devTemplate_close(int fd)
{
	int rv = -1;

	/* TODO: Add your device deinitialization code here */

	return rv;
}

/** @brief Generic device ioctl function.
 */
static int devTemplate_ioctl(int fd, int req, void * param)
{
	int rv = -1;

	/* TODO: Add your device I/O request code here */
	/* Example: */
	if(devList[fd] == &devTemplate)
	{
		switch(req)
		{
			case devTemplate_REQ_EXAMPLE1:
				/* Code for REQ_EXAMPLE1 request. */
				break;

			case devTemplate_REQ_EXAMPLE2:
				/* Code for REQ_EXAMPLE2 request. */
				break;

			case devTemplate_REQ_EXAMPLE3:
				/* Code for REQ_EXAMPLE3 request. */
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
