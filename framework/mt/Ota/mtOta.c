
#include "mtOta.h"
#include "asdLog.h"
#include "rpc.h"
#include "dbgPrint.h"
#include "ota_common.h"
#include "mtAf.h"
#include "hal_defs.h"

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
    uint8_t *p=&rpcBuff[2];
    //process the synchronous SRSP from SREQ
    if ((rpcBuff[0] & MT_RPC_CMD_TYPE_MASK) == MT_RPC_CMD_AREQ ){
        switch(rpcBuff[1]){
        case MT_OTA_NEXT_IMG_REQ:
        {
            /*+++++++++  From 2530 ++++++++++
            ++++++++++++++++++++++++++++++++++*/
            zclOTA_FileID_t fid;
            afAddrType_t addr;
            p=OTA_StreamToFileId(&fid,p);
            p=OTA_StreamToAfAddr(&addr,p);
            p++;
            uint16_t hwver = BUILD_UINT16(p[0],p[1]);

            print_hexdump("addr",addr.addr.extAddr,Z_EXTADDR_LEN);
            infof("hwver:%d,fwver:%04x,endpoint:%02x\n",hwver,fid.version,addr.endPoint);

            uint8_t *s=&rpcBuff[2];
            p=&rpcBuff[2];
            fid.version++;
            p=OTA_FileIdToStream(&fid,p);
            p=OTA_AfAddrToStream(&addr,p);
            *p++=0; //status
            *p++=0; //option
            uint32_t size=1024;
            *p++ = BREAK_UINT32(size, 0);
            *p++ = BREAK_UINT32(size, 1);
            *p++ = BREAK_UINT32(size, 2);
            *p++ = BREAK_UINT32(size, 3);
            infof("MT_OTA_NEXT_IMG_REQ\n");

            int status = rpcSendFrame(MT_RPC_SYS_OTA , MT_OTA_NEXT_IMG_RSP, s , p-s);
            infof("MT_OTA_NEXT_IMG_RSP:%d\n",status);

            break;
        }
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


