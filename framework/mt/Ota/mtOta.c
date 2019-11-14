
#include "mtOta.h"
#include "asdLog.h"
#include "rpc.h"
#include "dbgPrint.h"
#include "ota_common.h"
#include "mtAf.h"
#include "hal_defs.h"
#include "rom/md5_hash.h"

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




void md5( char *buf , int len ,unsigned char *digest, int reset , int final ){
    static struct MD5Context ctx;
    if (reset){
        MD5Init(&ctx);
    }
    MD5Update(&ctx, (unsigned char*)buf, len);

    if (final){
        MD5Final(digest, &ctx);
    }

}

void otaProcess(uint8_t *rpcBuff, uint8_t rpcLen)
{
    dbg_print(PRINT_LEVEL_VERBOSE, "JACK otaProcess: processing CMD0:%x, CMD1:%x\n",
            rpcBuff[0], rpcBuff[1]);
    uint8_t *p=&rpcBuff[2];
    //process the synchronous SRSP from SREQ
    if ((rpcBuff[0] & MT_RPC_CMD_TYPE_MASK) == MT_RPC_CMD_AREQ ){
        switch(rpcBuff[1]){
        case MT_OTA_NEXT_IMG_REQ:
        {
            zclOTA_FileID_t curFid;
            zclOTA_FileID_t fid;
            afAddrType_t addr;
            p=OTA_StreamToFileId(&curFid,p);
            p=OTA_StreamToAfAddr(&addr,p);
            p++;
            uint16_t hwver = BUILD_UINT16(p[0],p[1]);
            uint8_t *s=&rpcBuff[2];
            p=&rpcBuff[2];
         //   fid.version=0xbbbb;
//            fid.version=0xabcd;
            char *filepath=malloc(64);
            if (mtOtaCbs.pfnGetFilePathCb(filepath,64,curFid.type)){
                infof("No image available for type:%02x\n",curFid.type);
                free(filepath);
                break;
            }
            infof("hwver:%d,endpoint:%02x,shortAddr:%04x,devver:%04x,devtype:%02x,file:%s\n",hwver,addr.endPoint,addr.addr.shortAddr,curFid.version,curFid.type,filepath);

            uint32_t size=0;
            FILE *fp=fopen(filepath,"r");
            free(filepath);
            if (fp==0){
                errf("Open:%s failed\n",filepath);
                break;
            }

            if ( fseek(fp, (size_t)10, SEEK_SET) == 0 ){
                fread(&fid, 1, sizeof(zclOTA_FileID_t), fp);

                p=OTA_FileIdToStream(&fid,p);
                p=OTA_AfAddrToStream(&addr,p);

                if ( fseek(fp, (size_t)0, SEEK_END) == 0 ){
                    size=ftell(fp);
                }
                infof("fwver:%04x,fwtype:%02x,size:%d\n",fid.version,fid.type,size);
                *p++=0; //status
            }else{
                errf("fseek failed\n");
                *p++=1; //status
            }
            fclose(fp);

            *p++=0; //option
            *p++ = BREAK_UINT32(size, 0);
            *p++ = BREAK_UINT32(size, 1);
            *p++ = BREAK_UINT32(size, 2);
            *p++ = BREAK_UINT32(size, 3);
//            infof("JACK MT_OTA_NEXT_IMG_REQ\n");

            if(fid.version != curFid.version) {
                if ( mtOtaCbs.pfnOtaNextImgCb){
                    mtOtaCbs.pfnOtaNextImgCb();
                }
                int status = rpcSendFrame(MT_RPC_SYS_OTA , MT_OTA_NEXT_IMG_RSP, s , p-s);
                infof("MT_OTA_NEXT_IMG_RSP start update firmware,len:%d,status:%d\n",p-s,status);
            }
            else {
                infof("MT_OTA_NEXT_IMG_RSP the same firmware\n");
            }
//            infof("JACK MT_OTA_NEXT_IMG_RSP:%d\n",status);

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
//            infof("JACK MT_OTA_FILE_READ_REQ\n");
            zclOTA_FileID_t fid;
            afAddrType_t addr;
            p=OTA_StreamToFileId(&fid,p);
            p=OTA_StreamToAfAddr(&addr,p);
            uint32_t offset = BUILD_UINT32(p[0],p[1],p[2],p[3]);
            p+=4;
            uint8_t readLen=*p;

            debugf("read offset:%d,readLen:%d,shortAddr:%04x,type:%02x\n",offset,readLen,addr.addr.shortAddr,fid.type);

            uint8_t *s=&rpcBuff[2];
            p=&rpcBuff[2];
            p=OTA_FileIdToStream(&fid,p);
            p=OTA_AfAddrToStream(&addr,p);
            *p++=(uint8_t)0; //status
            *p++ = BREAK_UINT32(offset, 0);
            *p++ = BREAK_UINT32(offset, 1);
            *p++ = BREAK_UINT32(offset, 2);
            *p++ = BREAK_UINT32(offset, 3);
            if ( mtOtaCbs.pfnOtaFileReadCb){
                char *readBuf=malloc(readLen);
                int ret=mtOtaCbs.pfnOtaFileReadCb(fid.type,offset,readBuf,readLen);
                if (ret>=0){
                    memcpy(p+1,readBuf,ret);
                    free(readBuf);
                    *p++ = (uint8_t)ret;
#if 0
                    struct MD5Context ctx;
                    MD5Init(&ctx);
                    MD5Update(&ctx, (unsigned char*)p, ret);
                    unsigned char digest[16];
                    MD5Final(digest, &ctx);

                    char calmd5='N';
                    static int last_offset;
                    if (last_offset != offset || offset==0){
                        last_offset=offset;
                        if (offset==0){
                            md5((char*)p,ret,0,1,0);
                            calmd5='1';
                        }else {
                            if (ret != readLen ){
                                unsigned char digestfull[16];
                                md5((char*)p,ret,digestfull,0,1);
                                print_hexdump("md5",digestfull,16);
                                calmd5='3';
                            }else{
                                md5((char*)p,ret,0,0,0);
                                calmd5='2';
                            }
                        }
                    }
                    //infof("state=%c,md5#%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",calmd5,digest[0], digest[1], digest[2], digest[3], digest[4], digest[5], digest[6], digest[7], digest[8], digest[9], digest[10], digest[11], digest[12], digest[13], digest[14], digest[15] );
#endif
                    p+=ret;

                    int status = rpcSendFrame(MT_RPC_SYS_OTA , MT_OTA_FILE_READ_RSP, s, p-s);
                    infof("MT_OTA_FILE_READ_RSP status:%d,len:%d\n",status,p-s);
                    break;
                }else{
                    infof("pfnOtaFileReadCb failed:%d\n",ret);
                }
                free(readBuf);
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
            infof("JACK MT_OTA_STATUS_IND,panid:%04x,shortaddr:%04x,type:%02x,status:%02x,option:%02x\n",panid,shortaddr,p[0],p[1],p[2]);
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


