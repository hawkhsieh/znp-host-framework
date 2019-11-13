/*
 * mtsys.h
 *
 * This module contains the API for the MT SYS Interface.
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
#ifndef ZBMTOTA_H
#define ZBMTOTA_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/***************************************************************************************************
 * Ota COMMANDS
 ***************************************************************************************************/


typedef enum {
    ZBDevTypeSwitch=0x11,
    ZBDevTypeColorLamp=0x12,
    ZBDevTypeWarmLamp=0x13
}ZBDevType;
typedef uint8_t (*mtpfnOtaFileReadCb_t)( ZBDevType type,int offset , uint8_t *data,  int dataLen );
typedef uint8_t (*mtpfnNextImgCb_t)();
typedef int (*mtpfnGetFilePath_t)(char *filepath , int filepathLen , ZBDevType type);


typedef struct
{
    mtpfnOtaFileReadCb_t pfnOtaFileReadCb;
    mtpfnNextImgCb_t pfnOtaNextImgCb;
    mtpfnGetFilePath_t pfnGetFilePathCb;

} mtOtaCb_t;


/***************************************************************************************************
* CMD1
***************************************************************************************************/
#define MT_OTA_FILE_READ_REQ                  0x00
#define MT_OTA_NEXT_IMG_REQ                   0x01

#define MT_OTA_FILE_READ_RSP                  0x80
#define MT_OTA_NEXT_IMG_RSP                   0x81
#define MT_OTA_STATUS_IND                     0x82

/*MACROS*/
#define SUCCESS 0x00
#define FAILURE 0x01
#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)
#define BREAK_UINT32(var, ByteNum) \
                (uint8_t)((uint32_t)(((var)>>((ByteNum) * 8)) & 0x00FF))

void otaRegisterCallbacks(mtOtaCb_t cbs);

void otaProcess(uint8_t *rpcBuff, uint8_t rpcLen);

#ifdef __cplusplus
}
#endif

#endif /* ZBMTSYS_H */
