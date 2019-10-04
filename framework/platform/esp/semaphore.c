/*
 * semaphore.c
 *
 * This module contains the POSIX wrapper for Semaphore.
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*********************************************************************
 * INCLUDES
 */

#include <errno.h>
#include <stdint.h>
#include "semaphore.h"


#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS DECLARATIONS
 */
static int sem_pend(sem_t * sem, uint32_t timeout);

/*********************************************************************
 * API FUNCTIONS
 */

sem_t *sem_open(const char *name, int oflag)
{
    return 0;
}

int sem_close(sem_t *sem)
{
    return 0;
}

int sem_init(sem_t *sem, int pshared, unsigned int value)
{
	int ret = 0;
    (*sem) = xSemaphoreCreateMutex();
	if (*sem == NULL)
	{
		// error
		ret = -1;

		// set errno
		errno = ENOMEM;
	}

	return ret;
}

int sem_destroy(sem_t *sem)
{
	int ret = -1;

	if (sem != SEM_FAILED)
	{
		// just delete semaphore
        vSemaphoreDelete(*sem);

		// return success
		ret = 0;
	}
	else
	{
		// set errno
		errno = EINVAL;
	}

	return ret;
}

int sem_wait(sem_t * sem)
{
    return (sem_pend(sem, portMAX_DELAY));
}

int sem_timedwait(sem_t * sem, uint32_t timeout)
{
	return (sem_pend(sem, timeout));
}

int sem_post(sem_t * sem)
{
	int ret = -1;

	if (sem != SEM_FAILED)
	{
		//consolePrint("Semaphore_post++\n");

		// post semaphore
        xSemaphoreGive( *sem );

		// return success
		ret = 0;
	}
	else
	{
		// set errno
		errno = EINVAL;
	}

	return ret;
}

/*********************************************************************
 * LOCAL FUNCTIONS DEFINITIONS
 */
static int sem_pend(sem_t * sem, uint32_t timeout)
{
	int ret = -1;

	if (sem != SEM_FAILED)
	{
        if ( xSemaphoreTake( *sem, timeout/portTICK_RATE_MS ) == pdFALSE ) return -1;

        ret=0;
	}
	else
	{
		// set errno
		errno = EINVAL;
	}

	//consolePrint("Semaphore_pend--: error\n");

	return ret;
}
