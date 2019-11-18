
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
static uint16_t shortAddr;
static time_t lastOtaTime;

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

            uint32_t size=0;
            FILE *fp=fopen(OTAZB_FILEPATH,"r");
            uint8_t status=1;
            if (fp){
                if ( fseek(fp, (size_t)10, SEEK_SET) == 0 ){
                    fread(&fid, 1, sizeof(zclOTA_FileID_t), fp);
                }

                p=OTA_FileIdToStream(&fid,p);
                p=OTA_AfAddrToStream(&addr,p);

                if ( fseek(fp, (size_t)0, SEEK_END) == 0 ){
                    size=ftell(fp);
                    debugf("Got size:%u\n",size);
                    status=0;
                }
                fclose(fp);
            }
            infof("fwver:%08x,hwver:%d,endpoint:%02x,shortAddr:%04x,devver:%08x,devtype:%02x\n",fid.version,hwver,addr.endPoint,addr.addr.shortAddr,curFid.version,curFid.type);

            *p++=status; //status
            *p++=0; //option
            *p++ = BREAK_UINT32(size, 0);
            *p++ = BREAK_UINT32(size, 1);
            *p++ = BREAK_UINT32(size, 2);
            *p++ = BREAK_UINT32(size, 3);
//            infof("JACK MT_OTA_NEXT_IMG_REQ\n");

            time_t now=time(0);
            if(fid.type == curFid.type && fid.version != curFid.version && ((now-lastOtaTime)>180 || lastOtaTime==0 || shortAddr == addr.addr.shortAddr )) {
                shortAddr=addr.addr.shortAddr;
                lastOtaTime=now;
                infof("shortaddr:%04x ota start\n",shortAddr);

                if ( mtOtaCbs.pfnOtaNextImgCb){
                    mtOtaCbs.pfnOtaNextImgCb();
                }

                int status = rpcSendFrame(MT_RPC_SYS_OTA , MT_OTA_NEXT_IMG_RSP, s , p-s);
                debugf("MT_OTA_NEXT_IMG_RSP start update firmware,status:%d\n",status);
            }
            else {
                debugf("MT_OTA_NEXT_IMG_RSP ver %04x != %04x\n",fid.version,curFid.version);
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

            if ( shortAddr==addr.addr.shortAddr){
                lastOtaTime=time(0);
            }
            infof("read offset:%d,readLen:%d,shortAddr:%04x\n",offset,readLen,addr.addr.shortAddr);

            uint8_t *s=&rpcBuff[2];
            p=&rpcBuff[2];
            p=OTA_FileIdToStream(&fid,p);
            p=OTA_AfAddrToStream(&addr,p);
            *p++=0; //status
            *p++ = BREAK_UINT32(offset, 0);
            *p++ = BREAK_UINT32(offset, 1);
            *p++ = BREAK_UINT32(offset, 2);
            *p++ = BREAK_UINT32(offset, 3);
            if ( mtOtaCbs.pfnOtaFileReadCb){
                int ret=mtOtaCbs.pfnOtaFileReadCb(offset,p+1,readLen);
                if (ret>=0){

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
                    infof("state=%c,md5#%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",calmd5,digest[0], digest[1], digest[2], digest[3], digest[4], digest[5], digest[6], digest[7], digest[8], digest[9], digest[10], digest[11], digest[12], digest[13], digest[14], digest[15] );
#endif
                    p+=ret;
                    int status = rpcSendFrame(MT_RPC_SYS_OTA , MT_OTA_FILE_READ_RSP, s, p-s);
//                    infof("JACK MT_OTA_FILE_READ_RSP:%d\n",status);
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
            infof("JACK MT_OTA_STATUS_IND,panid:%04x,shortaddr:%04x,type:%02x,status:%02x,option:%02x\n",panid,shortaddr,p[0],p[1],p[2]);
            if (shortAddr==shortaddr){
                shortAddr=0;
                lastOtaTime=0;
                infof("shortaddr:%04x ota done\n",shortaddr);
            }

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


