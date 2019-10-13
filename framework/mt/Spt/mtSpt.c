
#include "mtSpt.h"
#include "asdLog.h"
#include "rpc.h"
#include "dbgPrint.h"

static mtSptCb_t mtSptCbs;
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
void sptRegisterCallbacks(mtSptCb_t cbs)
{
    memcpy(&mtSptCbs, &cbs, sizeof(mtSptCb_t));
}





void writeUartBuf(char *buf,int len){

    uint8_t status;
    status = rpcSendFrame((MT_RPC_CMD_SREQ | MT_RPC_SYS_SPT),  0, (uint8_t*)buf,len);

    if (status == MT_RPC_SUCCESS)
    {
        rpcWaitMqClientMsg(50);
    }


}


void sptProcess(uint8_t *rpcBuff, uint8_t rpcLen)
{
    dbg_print(PRINT_LEVEL_VERBOSE, "sptProcess: processing CMD0:%x, CMD1:%x\n",
            rpcBuff[0], rpcBuff[1]);

    //process the synchronous SRSP from SREQ
    if ((rpcBuff[0] & MT_RPC_CMD_TYPE_MASK) == MT_RPC_CMD_SRSP)
    {
        //copies sresp to local buffer
        memcpy(srspRpcBuff, rpcBuff, rpcLen);
        //srspRpcLen = rpcLen;

        if ( mtSptCbs.pfnSptSrsp ){
            SptRsp_t msg = {
                .result=rpcBuff[1]
            };
            mtSptCbs.pfnSptSrsp(&msg);
        }else{
            dbg_print(PRINT_LEVEL_WARNING, "not handled\n");
        }
    }else if ((rpcBuff[0] & MT_RPC_CMD_TYPE_MASK) == MT_RPC_CMD_SREQ){

        if ( mtSptCbs.pfnSptSreq ){
            SptReq_t msg = {
                .cmd=rpcBuff[1],
                .data=&rpcBuff[2],
                .dataLen=rpcLen,
            };
            mtSptCbs.pfnSptSreq(&msg);
        }else{
            dbg_print(PRINT_LEVEL_WARNING, "not handled\n");
        }

        rpcSendFrame((MT_RPC_CMD_SRSP | MT_RPC_SYS_SPT), mtSptCmdAPMode, 0, 0);

    }else{
        errf("SPT CMD0:%x, CMD1:%x, not handle\n",rpcBuff[0], rpcBuff[1]);
    }
}


