/*
 * rpc.c
 *
 * This module contains the RPC (Remote Procedure Call) API for the
 * ZigBee Network Processor (ZNP) Host Interface.
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <fcntl.h>      // For O_* constants
#include <errno.h>
#include <signal.h>
#include <semaphore.h>
#include <time.h>
#include "queue.h"

#include "rpc.h"
#include "rpcTransport.h"
#include "mtParser.h"
#include "dbgPrint.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SB_FORCE_BOOT              (0xF8)
#define SB_FORCE_RUN               (SB_FORCE_BOOT ^ 0xFF)

#define SRSP_TIMEOUT_MS            (500) // 500ms timeout
/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// semaphore for sending RPC frames (used for mutual exclusion for
// calling rpcSendFrame() function - from application thread(s) )
static sem_t rpcSem;

// semaphore for SRSP (Synchronous Response) used by application thread
// to wait for any response from the ZNP. The RPC thread will post
// the semaphore if any incoming message exists
static sem_t srspSem;

// expected SRSP command ID
static uint8_t expectedSrspCmdId;

// RPC message queue for passing RPC frame from RPC process to APP process
static llq_t rpcLlq;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS DECLARATION
 */

// function for calculating FCS in RPC UART frame
static uint8_t calcFcs(uint8_t *msg, uint8_t len);

// function for printing out RPC frames
static void printRpcMsg(char* preMsg, uint8_t sof, uint8_t len, uint8_t *msg);

/*********************************************************************
 * API FUNCTIONS
 */

/*********************************************************************
 * @fn      rpcOpen
 *
 * @brief   opens the serial port to the CC253x.
 *
 * @param   devicePath - path to the UART device
 *
 * @return  status
 */
int32_t rpcOpen(char *_devicePath, uint32_t port)
{
	int fd;

	// open RPC transport
	fd = rpcTransportOpen(_devicePath, port);
	if (fd < 0)
	{
		perror(_devicePath);
		dbg_print(PRINT_LEVEL_ERROR, "rpcOpen: %s device open failed\n",
		        _devicePath);
		return (-1);
	}

	sem_init(&rpcSem, 0, 1); // initialize mutex to 1 - binary semaphore
	sem_init(&srspSem, 0, 0); // initialize mutex to 0 - binary semaphore

	//rpcForceRun();

	return fd;
}

/*********************************************************************
 * @fn      rpcInitMq
 *
 * @brief   init message queue
 *
 * @param   -
 *
 * @return  status
 */
int32_t rpcInitMq(void)
{

	llq_open(&rpcLlq);
	return 0;
}

/*********************************************************************
 * @fn      rpcGetMqClientMsg
 *
 * @brief   wait (blocking function) for incoming message and process
 *          it
 *
 * @param   -
 *
 * @return  status
 */
int32_t rpcGetMqClientMsg(void)
{
    uint8_t rpcFrame[RPC_MAX_LEN + 1];
	int32_t rpcLen;

	dbg_print(PRINT_LEVEL_INFO, "rpcWaitMqClient: waiting on queue\n");

	// wait for incoming message queue
	rpcLen = llq_receive(&rpcLlq, (char *) rpcFrame, RPC_MAX_LEN + 1);

	if (rpcLen != -1)
	{
		dbg_print(PRINT_LEVEL_VERBOSE, "rpcWaitMqClient: processing MT[%d]\n",
		        rpcLen);

		// process incoming message
		mtProcess(rpcFrame, rpcLen);
	}
	else
	{
        dbg_print(PRINT_LEVEL_WARNING, "rpcWaitMqClient: Timeout\n");
		return -1;
	}

	return 0;
}

/*********************************************************************
 * @fn      rpcWaitMqClientMsg
 *
 * @brief   wait (with timeout) for incoming message and process
 *          it
 *
 * @param   -
 *
 * @return  status
 */
int32_t rpcWaitMqClientMsg(uint32_t timeout)
{
    uint8_t *rpcFrame=malloc(RPC_MAX_LEN + 1);
    if (rpcFrame==0){
        errf("malloc failed\n");
        sleep(1);
        return -1;
    }
	int32_t rpcLen, timeLeft = 0, mBefTime, mAftTime;

    struct timeval befTime, aftTime;

    dbg_print(PRINT_LEVEL_VERBOSE, "rpcWaitMqClientMsg: timeout=%d\n", timeout);

    gettimeofday(&befTime, NULL);
    rpcLen = llq_timedreceive(&rpcLlq, (char *) rpcFrame, RPC_MAX_LEN + 1, timeout);
	gettimeofday(&aftTime, NULL);
	if (rpcLen != -1)
	{
		mBefTime = befTime.tv_sec * 1000;
		mBefTime += befTime.tv_usec / 1000;
		mAftTime = aftTime.tv_sec * 1000;
		mAftTime += aftTime.tv_usec / 1000;
		timeLeft = mAftTime - mBefTime;
		timeLeft = timeout - timeLeft;
        dbg_print(PRINT_LEVEL_VERBOSE, "rpcWaitMqClientMsg: processing MT[%d]\n", rpcLen);
		// process incoming message
		mtProcess(rpcFrame, rpcLen);
	}
	else
	{
        dbg_print(PRINT_LEVEL_INFO, "rpcWaitMqClientMsg: Timed out [%d] - %s\n", rpcLen, strerror(errno));
        free(rpcFrame);
		return -1;
	}

    free(rpcFrame);
	return timeLeft;
}

/*********************************************************************
 * @fn      rpcForceRun
 *
 * @brief   send force run bootloader command
 *
 * @param   -
 *
 * @return  -
 */
void rpcForceRun(void)
{
	uint8_t forceBoot = SB_FORCE_RUN;

	// send the bootloader force boot incase we have a bootloader that waits
	rpcTransportWrite(&forceBoot, 1);
}

#if 1
typedef struct {
    uint8_t *head;
    int pktSize;
}pkt_t;
#define HEAD_SIZE 4

int collectPacket( pkt_t *pkt,uint8_t *buf , int dataLen ){

    int readLen=0;

    bzero(pkt,sizeof(pkt_t));
    if(dataLen < HEAD_SIZE){
        return 0;
    }
    int i;
    for (i=0;i<dataLen;i++){
        if ( buf[i]==MT_RPC_SOF ){
            break;
        }
    }
    if (i>=dataLen){
        errf("cant found 0xfe\n");
        return dataLen;
    }

    if (i != 0){
        infof("get rid of trash in front of header\n");
        memmove(buf,&buf[i],i);
        readLen=i;
        dataLen-=i;
    }

/*
    if (TAG_VERSION[0]=='0' &&TAG_VERSION[0]=='.' &&TAG_VERSION[0]=='1'){
       char *mac=recvBPacket.mac;
       memrev(mac,8);
    }
    */

    int rpcLen=buf[1];
    char cmd0,cmd1;

    cmd0=buf[2];
    cmd1=buf[3];
#define FCS_LEN 1

    int onePktLen=rpcLen+HEAD_SIZE+FCS_LEN; //0xfe,rpcLen,cmd0,cmd1,data...,fcs
    if (dataLen < onePktLen){
        return readLen;
    }
    print_hexdump("uart content",buf,dataLen);
    readLen += onePktLen;

#define SOF_LEN 1
    uint8_t fcs;
    //Verify FCS of incoming MT frames
    fcs = calcFcs(&buf[1], (onePktLen-SOF_LEN-FCS_LEN));
    if (buf[onePktLen-1] != fcs)
    {
        dbg_print(PRINT_LEVEL_WARNING, "rpcProcess: fcs error %x:%x,callen:%d\n",
                buf[onePktLen-1], fcs,onePktLen-SOF_LEN-FCS_LEN);
        goto END;
    }

    pkt->head=buf;
    pkt->pktSize=onePktLen;

END:
    return readLen;
}
/*************************************************************************************************
 * @fn      rpcProcess()
 *
 * @brief   Read bytes from transport layer and form an RPC frame
 *
 * @param   none
 *
 * @return  length of current Rx Buffer
 *************************************************************************************************/
int32_t rpcProcess(void)
{
    unsigned char buf[RPC_MAX_LEN];
    time_t pktHold=0;
    int totalSize=0;

    while(1){
        uint8_t d;
        int len = rpcTransportRead( &d, 1 );
        if ( len>0 ){
            debugf("0x%02x\n",d);
            if (totalSize>=RPC_MAX_LEN){
                errf("uart buffer is full clean it\n");
                totalSize=0;
            }
            buf[totalSize]=d;
            pktHold=time(0);
            totalSize += len;
        }
        pkt_t pkt;
        int ret = collectPacket( &pkt, buf , totalSize );
        if (ret>0){
            debugf("processing %d\n",ret);
            totalSize-=ret;
        }

        if (pkt.head==0){
            if ( pktHold > 0 && (time(0)-pktHold) > 3){
                totalSize=0;
            }
            continue;
        }

        int rpcLen=pkt.head[1];
        char cmd0,cmd1;

        cmd0=pkt.head[2];
        cmd1=pkt.head[3];

        if ((cmd0 & MT_RPC_CMD_TYPE_MASK) == MT_RPC_CMD_SRSP)
        {
            // SRSP command ID deteced
            if (expectedSrspCmdId == (cmd0 & MT_RPC_SUBSYSTEM_MASK))
            {
                dbg_print(PRINT_LEVEL_VERBOSE,
                        "rpcProcess: processing expected srsp [0x%02X]\n",
                        cmd0 & MT_RPC_SUBSYSTEM_MASK);

                //unblock waiting sreq
                sem_post(&srspSem);

                dbg_print(PRINT_LEVEL_VERBOSE,
                        "rpcProcess: writing %d bytes SRSP to head of the queue\n",
                        rpcLen);

                // send message to queue
                llq_add(&rpcLlq, (char*) &pkt.head[2], rpcLen+2, 1);
            }
            else
            {
                // unexpected SRSP discard
                dbg_print(PRINT_LEVEL_WARNING,
                        "rpcProcess: UNEXPECTED SREQ!: %02X%02X",
                        expectedSrspCmdId,
                        (cmd0 & MT_RPC_SUBSYSTEM_MASK));
                continue;
            }
        }
        else if ((cmd0 & MT_RPC_CMD_TYPE_MASK) == MT_RPC_CMD_SREQ){
            // should be SREQ frame
            dbg_print(PRINT_LEVEL_VERBOSE,
                    "rpcProcess: writing %d bytes SREQ to tail of the que\n",
                    rpcLen);
            llq_add(&rpcLlq, (char*) &pkt.head[2], rpcLen+2, 0);

        }else{
            // should be AREQ frame
            dbg_print(PRINT_LEVEL_VERBOSE,
                    "rpcProcess: writing %d bytes AREQ to tail of the que\n",
                    rpcLen);

            // send message to queue
            llq_add(&rpcLlq, (char*) &pkt.head[2], rpcLen+2, 0);
        }
    }
    return 0;
}
#else

/*************************************************************************************************
 * @fn      rpcProcess()
 *
 * @brief   Read bytes from transport layer and form an RPC frame
 *
 * @param   none
 *
 * @return  length of current Rx Buffer
 *************************************************************************************************/
int32_t rpcProcess(void)
{
    uint8_t rpcLen, rpcTempLen, bytesRead, sofByte, rpcBuffIdx;
    uint8_t retryAttempts = 0, len, rpcBuff[RPC_MAX_LEN];
    uint8_t fcs;

#ifndef HAL_UART_IP //No SOF for IP	//read first byte and check it is a SOF
    bytesRead = rpcTransportRead(&sofByte, 1);

    if ((sofByte == MT_RPC_SOF) && (bytesRead == 1))
#endif
    {
        // clear retry counter
        retryAttempts = 0;

        // read length byte
        bytesRead = rpcTransportRead(&rpcLen, 1);

        if (bytesRead == 1)
        {
            len = rpcLen;
            rpcBuff[0] = rpcLen;

#ifdef HAL_UART_IP //No FCS for IP			//allocating RPC payload (+ cmd0, cmd1)
            rpcLen += RPC_CMD0_FIELD_LEN + RPC_CMD1_FIELD_LEN;
#else
            //allocating RPC payload (+ cmd0, cmd1 and fcs)
            rpcLen +=
            RPC_CMD0_FIELD_LEN + RPC_CMD1_FIELD_LEN + RPC_UART_FCS_LEN;
#endif

            //non blocking read, so we need to wait for the rpc to be read
            rpcBuffIdx = 1;
            rpcTempLen = rpcLen;
            while (rpcTempLen > 0)
            {
                // read RPC frame
                bytesRead = rpcTransportRead(&(rpcBuff[rpcBuffIdx]),
                        rpcTempLen);

                // check for error
                if (bytesRead > rpcTempLen)
                {
                    //there was an error
                    dbg_print(PRINT_LEVEL_WARNING,
                            "rpcProcess: read of %d bytes failed - %s\n",
                            rpcTempLen, strerror(errno));

                    // check whether retry limits has been reached
                    if (retryAttempts++ < 5)
                    {
                        // sleep for 10ms
                        usleep(10000);

                        // try again
                        bytesRead = 0;
                    }
                    else
                    {
                        // something went wrong, abort
                        dbg_print(PRINT_LEVEL_ERROR,
                                "rpcProcess: transport read failed too many times\n");

                        return -1;
                    }
                }

                // update counters
                if (rpcTempLen > bytesRead)
                {
                    rpcTempLen -= bytesRead;
                }
                else
                {
                    rpcTempLen = 0;
                }
                rpcBuffIdx += bytesRead;
            }

            // print out incoming RPC frame
            printRpcMsg("SOC IN  <--", MT_RPC_SOF, len, &rpcBuff[1]);

            //Verify FCS of incoming MT frames
            fcs = calcFcs(&rpcBuff[0], (len + 3));
            if (rpcBuff[len + 3] != fcs)
            {
                dbg_print(PRINT_LEVEL_WARNING, "rpcProcess: fcs error %x:%x\n",
                        rpcBuff[len + 3], fcs);
                return -1;
            }

            if ((rpcBuff[1] & MT_RPC_CMD_TYPE_MASK) == MT_RPC_CMD_SRSP)
            {
                // SRSP command ID deteced
                if (expectedSrspCmdId == (rpcBuff[1] & MT_RPC_SUBSYSTEM_MASK))
                {
                    dbg_print(PRINT_LEVEL_INFO,
                            "rpcProcess: processing expected srsp [%02X]\n",
                            rpcBuff[1] & MT_RPC_SUBSYSTEM_MASK);

                    //unblock waiting sreq
                    sem_post(&srspSem);

                    dbg_print(PRINT_LEVEL_INFO,
                            "rpcProcess: writing %d bytes SRSP to head of the queue\n",
                            rpcLen);

                    // send message to queue
                    llq_add(&rpcLlq, (char*) &rpcBuff[1], rpcLen, 1);
                }
                else
                {
                    // unexpected SRSP discard
                    dbg_print(PRINT_LEVEL_WARNING,
                            "rpcProcess: UNEXPECTED SREQ!: %02X:%02X",
                            expectedSrspCmdId,
                            (rpcBuff[1] & MT_RPC_SUBSYSTEM_MASK));
                    return 0;
                }
            }
            else
            {
                // should be AREQ frame
                dbg_print(PRINT_LEVEL_INFO,
                        "rpcProcess: writing %d bytes AREQ to tail of the que\n",
                        rpcLen);

                // send message to queue
                llq_add(&rpcLlq, (char*) &rpcBuff[1], rpcLen, 0);
            }

            return 0;
        }
        else
        {
            dbg_print(PRINT_LEVEL_WARNING, "rpcProcess: Len Not read [%x]\n",
                    bytesRead);
        }
    }
    else
    {
        dbg_print(PRINT_LEVEL_WARNING,
                "rpcProcess: No valid Start Of Frame found [%x:%x]\n", sofByte,
                bytesRead);
    }

    return -1;
}

#endif


/*************************************************************************************************
 * @fn      sendRpcFrame()
 *
 * @brief   builds the Frame and sends it to the transport layer - usually called by the
 *          application thread(s)
 *
 * @param   cmd0 System, cmd1 subsystem, ptr to payload, lenght of payload
 *
 * @return  length of current Rx Buffer
 *************************************************************************************************/
uint8_t rpcSendFrame(uint8_t cmd0, uint8_t cmd1, uint8_t *payload,
        uint8_t payload_len)
{
    uint8_t *buf=malloc(RPC_MAX_LEN);
	int32_t status = MT_RPC_SUCCESS;

	// block here if SREQ is in progress
    dbg_print(PRINT_LEVEL_VERBOSE, "rpcSendFrame: Blocking on RPC sem\n");
	sem_wait(&rpcSem);
    dbg_print(PRINT_LEVEL_VERBOSE, "rpcSendFrame: Sending RPC,len:%d\n",payload_len);

	// fill in header bytes
	buf[0] = MT_RPC_SOF;
	buf[1] = payload_len;
	buf[2] = cmd0;
	buf[3] = cmd1;

	if ((cmd0 & MT_RPC_CMD_TYPE_MASK) == MT_RPC_CMD_SREQ)
	{
		// calculate expected SRSP
		expectedSrspCmdId = (cmd0 & MT_RPC_SUBSYSTEM_MASK);
	}

	if (payload_len > 0)
	{
		// copy payload to buffer
		memcpy(buf + RPC_UART_HDR_LEN, payload, payload_len);
	}

	// calculate FCS field
	buf[payload_len + RPC_UART_HDR_LEN] = calcFcs(
	        &buf[RPC_UART_FRAME_START_IDX], payload_len + RPC_HDR_LEN);

#ifdef HAL_UART_IP
	// No SOF or FCS
	rpcTransportWrite(buf+1, payload_len + RPC_HDR_LEN + RPC_UART_FCS_LEN);
#else
	// send out RPC  message
	rpcTransportWrite(buf, payload_len + RPC_UART_HDR_LEN + RPC_UART_FCS_LEN);
#endif

	// print out message to be sent
    print_hexdump("uart send",buf,payload_len+5);

	// wait for SRSP if necessary
	if ((cmd0 & MT_RPC_CMD_TYPE_MASK) == MT_RPC_CMD_SREQ)
    {
        dbg_print(PRINT_LEVEL_INFO, "rpcSendFrame: waiting for SRSP [%02x]\n",
                expectedSrspCmdId);

		//Wait for the SRSP
        status = sem_timedwait(&srspSem, SRSP_TIMEOUT_MS );
		if (status == -1)
		{
			dbg_print(PRINT_LEVEL_WARNING,
			        "rpcSendFrame: SRSP Error - CMD0: 0x%02X CMD1: 0x%02X\n",
			        cmd0, cmd1);
			status = MT_RPC_ERR_SUBSYSTEM;
		}
		else
		{
			dbg_print(PRINT_LEVEL_INFO, "rpcSendFrame: Receive SRSP\n");
			status = MT_RPC_SUCCESS;
		}

		//set expected SRSP to invalid
		expectedSrspCmdId = 0xFF;
	}

	//Unlock RPC sem
	sem_post(&rpcSem);

    free(buf);
	return status;
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      calcFcs
 *
 * @brief   calculate the FCS (Frame Check Sequence) of the RPC payload.
 *
 * @param   msg  - pointer to the RPC general format frame message
 * @param   size - RPC general format frame size
 *
 * @return  FCS value
 */
static uint8_t calcFcs(uint8_t *msg, uint8_t size)
{
	uint8_t result = 0;

	// calculate FCS by XORing all bytes
	while (size--)
	{
		result ^= *msg++;
	}

	return result;
}

/*********************************************************************
 * @fn      printRpcMsg
 *
 * @brief   print out RPC message
 *
 * @param   preMsg - initial string
 * @param   sof - SOF (Start of Frame) bytes
 * @param   len - length byte
 * @param   msg  - pointer to the RPC message starting from Cmd0 byte
 *
 * @return  FCS value
 */
static void printRpcMsg(char* preMsg, uint8_t sof, uint8_t len, uint8_t *msg)
{
	// print headers
    dbg_print(PRINT_LEVEL_INFO,
	        "%s %d Bytes: SOF:%02X, Len:%02X, CMD0:%02X, CMD1:%02X, Payload:",
            preMsg, len + 4, sof, len, msg[0], msg[1]);
    if (len>2){
        print_hexdump("znp",msg,len);
    }

}
