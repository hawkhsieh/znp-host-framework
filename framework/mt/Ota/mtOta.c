
#include "mtOta.h"
#include "asdLog.h"
#include "rpc.h"
#include "dbgPrint.h"

static mtOtaCb_t mtOtaCbs;
extern uint8_t srspRpcBuff[RPC_MAX_LEN];
extern uint8_t srspRpcLen;


/*********************************************************************
 * @fn      sysRegisterCallbacks
 *
 * @brief
 *
 * @param
 *
 */
void otaRegisterCallbacks(mtOtaCb_t cbs)
{
    memcpy(&mtOtaCbs, &cbs, sizeof(mtOtaCb_t));
}





void otaProcess(uint8_t *rpcBuff, uint8_t rpcLen)
{
    dbg_print(PRINT_LEVEL_VERBOSE, "otaProcess: processing CMD0:%x, CMD1:%x\n",
            rpcBuff[0], rpcBuff[1]);

    //process the synchronous SRSP from SREQ
    if ((rpcBuff[0] & MT_RPC_CMD_TYPE_MASK) == MT_RPC_CMD_AREQ ){
        switch(rpcBuff[1]){
        case MT_OTA_FILE_READ_REQ:
            infof("MT_OTA_FILE_READ_REQ\n");

            /*+++++++++  From 2530 ++++++++++
            // Add the file ID
            p = OTA_FileIdToStream(pFileId, p);

            // Add the device address
            p = OTA_AfAddrToStream(pAddr, p);

            // File ofset to read from
            *p++ = BREAK_UINT32(offset, 0);
            *p++ = BREAK_UINT32(offset, 1);
            *p++ = BREAK_UINT32(offset, 2);
            *p++ = BREAK_UINT32(offset, 3);

            *p = len;
            ++++++++++++++++++++++++++++++++++*/

            //copies sresp to local buffer
            memcpy(srspRpcBuff, rpcBuff, rpcLen);
            //srspRpcLen = rpcLen;
            if ( mtOtaCbs.pfnOtaSrsp ){
                OtaRsp_t msg = {
                    .result=rpcBuff[1]
                };
                mtOtaCbs.pfnOtaSrsp(&msg);
            }else{
                dbg_print(PRINT_LEVEL_WARNING, "not handled\n");
            }

      //      status = rpcSendFrame(MT_RPC_SYS_OTA , MT_OTA_FILE_READ_RSP, rpcBuff, rpcLen);

            break;
        case MT_OTA_NEXT_IMG_REQ:
            /*+++++++++  From 2530 ++++++++++
            ++++++++++++++++++++++++++++++++++*/
        //    status = rpcSendFrame(MT_RPC_SYS_OTA , MT_OTA_NEXT_IMG_RSP, cmd, cmdLen);

            infof("MT_OTA_NEXT_IMG_REQ\n");
            break;
        case MT_OTA_STATUS_IND:
            infof("MT_OTA_STATUS_IND\n");
            break;
        default:
            errf("OTA CMD0:%x, CMD1:%x, not handle\n",rpcBuff[0], rpcBuff[1]);
            break;
        }
    }else{
        errf("OTA CMD0:%x, CMD1:%x, not handle\n",rpcBuff[0], rpcBuff[1]);
    }
}


