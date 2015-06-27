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

#ifndef PIPE_H_
#define PIPE_H_

/** @brief Pipe management header file.
 **
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

#include "stdint.h"

#include "devList.h"

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

/** @brief Pipe buffer lenght in bytes */
#define PIPE_BUFFER_LENGTH	256

/** @brief Maximum pipes that can be created */
#define MAX_PIPES			10

/** @brief Maximum file descriptor numbers associated to pipes. */
#define MAX_PIPES_FDS		(devListSize + 2 * MAX_PIPES)

/*==================[typedef]================================================*/

/** @brief Pipe definition structure */
typedef struct
{
	uint8_t buffer[PIPE_BUFFER_LENGTH];	/**< Pipe data buffer */
	int count;							/**< Pipe data count */
	int fd[2];							/**< Pipe in/out file descriptors */
	pthread_mutex_t mtx;				/**< Pipe mutex */
}pipe_t;

/** @brief Pipe endpoints definition */
typedef enum
{
	PIPE_READ_END  = 0,	/**< fd[0]: read end. */
	PIPE_WRITE_END = 1	/**< fd[1]: write end. */
}pipeEnd_t;

/*==================[external data declaration]==============================*/

extern const device_t devPipes;

/*==================[external functions declaration]=========================*/

int pipe(int * pfds);

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* PIPE_H_ */
