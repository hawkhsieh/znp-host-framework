
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
         //   fid.version=0xbbbb;
            fid.version=0xabcd;

            p=OTA_FileIdToStream(&fid,p);
            p=OTA_AfAddrToStream(&addr,p);
            *p++=0; //status
            *p++=0; //option
            uint32_t size=137068;
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
        {

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
            infof("MT_OTA_FILE_READ_REQ\n");
            zclOTA_FileID_t fid;
            afAddrType_t addr;
            p=OTA_StreamToFileId(&fid,p);
            p=OTA_StreamToAfAddr(&addr,p);
            uint32_t offset = BUILD_UINT32(p[0],p[1],p[2],p[3]);
            p+=4;
            uint8_t readLen=*p;

            infof("read offset:%d,readLen:%d\n",offset,readLen);

            uint8_t *s=&rpcBuff[2];
            p=&rpcBuff[2];
            p=OTA_FileIdToStream(&fid,p);
            p=OTA_AfAddrToStream(&addr,p);
            *p++=0; //status
            *p++ = BREAK_UINT32(offset, 0);
            *p++ = BREAK_UINT32(offset, 1);
            *p++ = BREAK_UINT32(offset, 2);
            *p++ = BREAK_UINT32(offset, 3);
            *p++ = readLen;
            if ( mtOtaCbs.pfnOtaFileReadCb){
                int ret=mtOtaCbs.pfnOtaFileReadCb(offset,p,readLen);
                if (ret>=0){
                    p+=ret;
                    int status = rpcSendFrame(MT_RPC_SYS_OTA , MT_OTA_FILE_READ_RSP, s, p-s);
                    infof("MT_OTA_FILE_READ_RSP:%d\n",status);
                    break;
                }else{
                    infof("pfnOtaFileReadCb failed:%d\n",ret);
                }
            }
            infof("MT_OTA_FILE_READ_RSP not handled\n");
            break;
        }
        case MT_OTA_STATUS_IND:
        {
            uint8_t *p=&rpcBuff[2];
            uint16_t panid = BUILD_UINT16(p[0],p[1]);
            p+=2;
            uint16_t shortaddr = BUILD_UINT16(p[0],p[1]);
            p+=2;
            infof("MT_OTA_STATUS_IND,panid:%04x,shortaddr:%04x,type:%02x,status:%02x,option:%02x\n",panid,shortaddr,p[0],p[1],p[2]);
            break;
        }
        default:
            errf("OTA CMD0:%x, CMD1:%x, not handle\n",rpcBuff[0], rpcBuff[1]);
            break;
        }
    }else{
        errf("OTA CMD0:%x, CMD1:%x, not handle\n",rpcBuff[0], rpcBuff[1]);
    }
}


