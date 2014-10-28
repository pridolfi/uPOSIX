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

#ifndef DEVICE_H_
#define DEVICE_H_

/** \addtogroup uPOSIX uPOSIX
 ** @brief Main uPOSIX module group.
 ** @{ */
/** \addtogroup uPOSIXdev Device Management
 ** @brief Functions and types for peripheral management.
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

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

typedef struct device_t_struct device_t;

typedef int (*open_t) (const device_t * const dev, int flags);
typedef int (*read_t) (const device_t * const dev, void * buf, int len);
typedef int (*write_t)(const device_t * const dev, const void * buf, int len);
typedef int (*close_t)(const device_t * const dev);
typedef int (*ioctl_t)(const device_t * const dev, int req, void * param);

/**@brief File operation functions structure */
typedef struct
{
	open_t open;            /**< pointer to open function  */
	read_t read;            /**< pointer to read function  */
	write_t write;          /**< pointer to write function */
	close_t close;          /**< pointer to close function */
	ioctl_t ioctl;          /**< pointer to ioctl function */
}fops_t;

/**@brief Generic Device definition structure */
typedef struct device_t_struct
{
	const char * name;		/**< path (relative to /dev/)  */
	void * ptr;         	/**< peripheral base address   */
	const fops_t * fops;	/**< file op functions 		   */
}device_t;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

int open(const char * path, int flags);
int read(int fd, void * buf, int len);
int write(int fd, const void * buf, int len);
int close(int fd);
int ioctl(int fd, int req, void * param);

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* DEVICE_H_ */
