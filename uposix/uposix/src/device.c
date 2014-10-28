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

/** @brief device.c source file
 **
 ** This is the main uPOSIX source file. It contains standard device access functions.
 **
 **/

/** \addtogroup uPOSIX uPOSIX
 ** @{ */
/** \addtogroup uPOSIXdev Device Management
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

#include "string.h"

#include "device.h"
#include "devList.h"
#include "pipe.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/** @brief This internal function takes a path string and searches for its file
 * descriptor and device_t struct.
 *
 * @warning This is an internal (static) function. It should not be called by the user.
 *
 * @param path	String containing the path to the device, for example "/dev/gpio".
 * @param dev	device_t structure (by ref.) to be stored.
 * @return 		Device file descriptor.
 *
 */
static int getDeviceFromPath(const char * path, device_t ** dev)
{
	int i = devListSize;

	if(strstr(path, "/dev/"))
	{
		for(i=0; i<devListSize; i++)
		{
			if(strstr(path, devList[i]->name))
			{
				*dev = (device_t *)devList[i];
				break;
			}
		}
	}

	if(i == devListSize)
	{
		i = -1;
		*dev = 0;
	}

	return i;
}

/** @brief This internal function takes a file descriptor and returns its associated
 * device_t structure.
 *
 * @warning This is an internal (static) function. It should not be called by the user.
 *
 * @param fd	File descriptor.
 * @return 		device_t structure associated to fd.
 *
 */
static const device_t * getDeviceFromDesc(int fd)
{
	const device_t * dev = 0;

	if(fd < devListSize)
	{
		dev = devList[fd];
	}

	return dev;
}

/*==================[external functions definition]==========================*/

/** @brief This function returns a file descriptor associated to the device we
 * want to interact with.
 *
 * @param path	String containing the path to the device, for example "/dev/gpio".
 * @param flags	Access flags, device dependent.
 * @return 		Device file descriptor.
 *
 */
int open(const char * path, int flags)
{
	int rv = -1;
	device_t * dev = 0;

	int fd = getDeviceFromPath(path, &dev);

	if(dev)
	{
		rv = dev->fops->open(dev, flags);
		if(rv < 0)
		{
			fd = -1;
		}
	}
	else
	{
		fd = -1;
	}

	return fd;
}

/** @brief This function reads data from a device. Its behavior depends on the device.
 *
 * @param fd	Device file descriptor, returned by open.
 * @param buf	Buffer where data will be stored.
 * @param len	Maximum bytes to be read.
 * @return 		Number of bytes actually read.
 *
 */
int read(int fd, void * buf, int len)
{
	int rv = -1;

	if(fd < devListSize)
	{
		const device_t * dev = getDeviceFromDesc(fd);
		if(dev)
		{
			rv = dev->fops->read(dev, buf, len);
		}
	}
	else if(fd < MAX_PIPES_FDS)
	{
		rv = devPipes.fops->read((device_t *)fd, buf, len);
	}

	return rv;
}

/** @brief This function writes data to a device. Its behavior depends on the device.
 *
 * @param fd	Device file descriptor, returned by open.
 * @param buf	Buffer pointing to the data to be written.
 * @param len	Maximum data length in bytes to be written.
 * @return 		Number of bytes actually written.
 *
 */
int write(int fd, const void * buf, int len)
{
	int rv = -1;

	if(fd < devListSize)
	{
		const device_t * dev = getDeviceFromDesc(fd);
		if(dev)
		{
			rv = dev->fops->write(dev, buf, len);
		}
	}
	else if(fd < MAX_PIPES_FDS)
	{
		rv = devPipes.fops->write((device_t *)fd, buf, len);
	}

	return rv;
}

/** @brief This function closes access to the device. Its behavior depends on the device.
 *
 * @param fd	Device file descriptor, returned by open.
 * @return 		Device dependent, normally 0 on success.
 *
 */
int close(int fd)
{
	int rv = -1;

	if(fd < devListSize)
	{
		const device_t * dev = getDeviceFromDesc(fd);
		if(dev)
		{
			rv = dev->fops->close(dev);
		}
	}

	return rv;
}

/** @brief This function is used to interact with a device. Its behavior depends on the device.
 *
 * @param fd	Device file descriptor, returned by open.
 * @param req	ioctl request, normally a constant like devDeviceName_REQ_REQUEST, for example
 * 				devGPIO_REQ_WRITE_BIT.
 * @param param	Generic parameter to be sent to the device driver. This is device dependent.
 * @return 		ioctl request status, normally 0 if completed with no errors,
 * 				but this is device dependent.
 *
 */
int ioctl(int fd, int req, void * param)
{
	int rv = -1;

	if(fd < devListSize)
	{
		const device_t * dev = getDeviceFromDesc(fd);
		if(dev)
		{
			rv = dev->fops->ioctl(dev, req, param);
		}
	}
	else
	{

	}

	return rv;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
