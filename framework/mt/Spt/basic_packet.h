#ifndef _BASIC_PACKET_
#define _BASIC_PACKET_



/*
包结构
头部              2个字节                    55 aa
长度              1个字节                    所有字节的长度，包括crc
类型              1个字节
mac地址           8个字节
功能              1个字节
数据              长度-13个字节
CRC               2个字节                    除去crc两个字节以外，所有的字节的crc
*/


/*
0xff 查询设备状态
0xfe 回复状态查询
0xfd 上传状态

0xe0 设备查询是否可以加入这个网络
0xe1 设备允许加入这个网络
0xe2 设备禁止加入这个网络
0xe3 设备入网查询中
0xe4 告诉父节点已经入网成功


0x01 控制器开关，灯泡开关控制

0x02 彩色灯颜色控制
0x03 暖色灯色温控制
0x04 灯泡亮度控制

*/


#define Type_Coordinator                0x01
#define Type_Router                     0x02
#define Type_Control                    0x03
#define Type_Light_Color                0x04
#define Type_Light_Temp                 0x05

#define Fun_Check                       0xff
#define Fun_Check_Back                  0xfe
#define Fun_Upload_Status               0xfd

#define Fun_OTA_EndReq      			0xFB
#define Fun_OTA_BlockReq      			0xFA
#define Fun_OTA_NxtImageReq          	0xF9
#define Fun_OTA_Start         			0xF8

#define Fun_WiFi_Reset					0xF7
#define Fun_LogMode            			0xF6



#define Fun_Dev_Ask_Join                0xe0
#define Fun_Dev_allow_Join              0xe1
#define Fun_Dev_refuse_Join             0xe2
#define Fun_Dev_Check_Join              0xe3
#define Fun_Dev_Ok_Join                 0xe4

#define Fun_Dev_OnOff                   0x01
#define Fun_Dev_Light_Color             0x02
#define Fun_Dev_Light_Temp              0x03
#define Fun_Dev_Light_Lux               0x04
#define Fun_Dev_OnOff_Tog               0x05
#define Fun_Dev_Reset                   0x06
#define Fun_Dev_Light_Blink             0x07
#define Fun_Dev_Alive                   0x08
#define Fun_Dev_UnpairAll               0x09



#define PACKET_SIZE 13



typedef struct BPacket{
    char *head;
    char *mac;
    char *data;
    unsigned char len;
    char func;
    char type;
}BPacket;

int checkPacket(unsigned char *buf, unsigned char len, BPacket *recvBPacket );
BPacket* packPacket(char type,char *mac,char func,unsigned char *data,unsigned char datalen);


typedef enum {
    UartCmdSetLog_None,
    UartCmdSetLog_Error,
    UartCmdSetLog_Info,
    UartCmdSetLog_Debug

}UartCmdSetLog;
void setZBLogLevel( UartCmdSetLog loglv );




#endif
