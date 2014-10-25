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

/** @brief Pipe management source file.
 **
 **/

/** \addtogroup uPOSIX uPOSIX
 ** @{ */
/** \addtogroup pipes Pipe management
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 * PR			Pablo Ridolfi
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20141001 v0.0.1   PR first version
 */

/*==================[inclusions]=============================================*/

#include "string.h"

#include "pipe.h"
#include "heap.h"
#include "device.h"
#include "devList.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

static int devPipes_open(const char * path, int flags);
static int devPipes_read(int fd, void * buf, int len);
static int devPipes_write(int fd, const void * buf, int len);
static int devPipes_close(int fd);
static int devPipes_ioctl(int fd, int req, void * param);

/*==================[internal data definition]===============================*/

/** @brief Available pipes */
pipe_t * pipes[MAX_PIPES];

/** @brief Currently used pipes */
uint8_t pipeCount = 0;

/*==================[external data definition]===============================*/

/** @brief Pipes device struct. */
const device_t devPipes =
{
		"pipes",			/**< path (relative to /dev/)  */
		(void *)pipes,		/**< peripheral base address   */
		devPipes_open,		/**< pointer to open function  */
		devPipes_read,		/**< pointer to read function  */
		devPipes_write,		/**< pointer to write function */
		devPipes_close,		/**< pointer to close function */
		devPipes_ioctl		/**< pointer to ioctl function */
};

/*==================[internal functions definition]==========================*/

/** @brief 		Get pipe structure from its descriptor
 * 	@param pfd 	Pipe file descriptor.
 * 	@param rw	Pipe file descriptor type (PIPE_READ_END or PIPE_WRITE_END).
 * 	@return		Pipe structure, NULL on error.
 */
static pipe_t * getPipeFromDescriptor(int pfd, pipeEnd_t rw)
{
	pipe_t * p = 0;
	uint8_t i;

	for(i=0; i<pipeCount; i++)
	{
		if(pipes[i]->fd[rw] == pfd)
		{
			p = pipes[i];
			break;
		}
	}

	return p;
}

/** @brief 		Open function, not used.
 */
static int devPipes_open(const char * path, int flags)
{
	return -1;
}

/** @brief 		Get data from pipe. This function blocks the current thread if there isn't <b>len</b>
 * 				bytes available in the pipe.
 * 	@param fd 	Pipe file descriptor (PIPE_READ_END).
 * 	@param buf	Data buffer.
 * 	@param len	Data buffer length.
 * 	@return		Actually read bytes, 0 if pipe empty, -1 on error.
 */
static int devPipes_read(int fd, void * buf, int len)
{
	int rv = -1;
	pipe_t * p = getPipeFromDescriptor(fd, PIPE_READ_END);

	if((p != 0) && (buf != 0) && (len != 0) && (len < PIPE_BUFFER_LENGTH))
	{
		int i;

		while(p->count < len)
		{
			pthread_mutex_lock(&(p->mtx));
		}

		memcpy(buf, p->buffer, len);

		for(i=len; i < p->count; i++)
		{
			p->buffer[i-len] = p->buffer[i];
		}

		p->count -= len;
		rv = len;

		if(p->count < PIPE_BUFFER_LENGTH)
		{
			pthread_mutex_unlock(&(p->mtx));
		}
	}

	return rv;
}

/** @brief 		Write data to pipe. This function blocks the calling thread if there is
 * 				not enough space to write the data.
 * 	@param fd 	Pipe file descriptor (PIPE_WRITE_END).
 * 	@param buf	Data buffer.
 * 	@param len	Data buffer length.
 * 	@return		Actually written bytes, 0 if pipe full, -1 on error.
 */
static int devPipes_write(int fd, const void * buf, int len)
{
	int rv = -1;
	pipe_t * p = getPipeFromDescriptor(fd, PIPE_WRITE_END);

	if((p != 0) && (buf != 0) && (len != 0) && (len < PIPE_BUFFER_LENGTH))
	{
		int space;

		do
		{
			space = PIPE_BUFFER_LENGTH - p->count;
			if(space < len)
			{
				pthread_mutex_lock(&(p->mtx));
			}
		}while(space < len);

		uint8_t * pbuf = p->buffer + p->count;

		rv = (space >= len ? len : space);

		memcpy(pbuf, buf, rv);

		p->count += rv;

		pthread_mutex_unlock(&(p->mtx));
	}
	return rv;
}

/** @brief 		Close function, not used.
 */
static int devPipes_close(int fd)
{
	return -1;
}

/** @brief 		Ioctl function, not used.
 */
static int devPipes_ioctl(int fd, int req, void * param)
{
	return -1;
}

/*==================[external functions definition]==========================*/

/** @brief		Pipe creation.
 *  @param pfds	Pipe file descriptors for reading (pfds[0]) and writing
 *  			(pfds[1]) operations.
 *  @return		0 on successful creation, -1 on error.
 */
int pipe(int * pfds)
{
	int rv = -1;
	pipe_t * p;

	if(pipeCount < MAX_PIPES)
	{
		p = (pipe_t *)kmalloc(sizeof(pipe_t));
		if(p != 0)
		{
			memset(p->buffer, 0, PIPE_BUFFER_LENGTH);

			p->count = 0;

			p->fd[0] = pfds[0] = devListSize + 2 * pipeCount;
			p->fd[1] = pfds[1] = devListSize + 2 * pipeCount + 1;

			pthread_mutex_init(&(p->mtx), 0);

			pipes[pipeCount] = p;
			pipeCount++;

			rv = 0;
		}
	}
	return rv;
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
