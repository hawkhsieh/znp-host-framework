#include "basic_packet.h"
#include "string.h"
#include "stdio.h"
#include "asdLog.h"

#include <stdlib.h>


/*
**  reverse "count" bytes starting at "buf"
*/


void memrev(char  *buf, size_t count)
{
      char *r;

      for (r = buf + count - 1; buf < r; buf++, r--)
      {
            *buf ^= *r;
            *r   ^= *buf;
            *buf ^= *r;
      }
}


BPacket sendBPacket;
unsigned char bpacketSendBuf[128];


BPacket* packPacket(char type,char *mac,char func,unsigned char *data,unsigned char datalen){
  memset(bpacketSendBuf,0,128);
  bpacketSendBuf[0] = 0x55;bpacketSendBuf[1] = 0xaa;
  bpacketSendBuf[2] = (13+datalen);
  bpacketSendBuf[3] = type;
  memcpy(&bpacketSendBuf[4],mac,8);
  bpacketSendBuf[12] = func;
  if (datalen>0){
      memcpy(&bpacketSendBuf[13],data,datalen);
  }
  sendBPacket.head = (char *)bpacketSendBuf;
  sendBPacket.mac = mac;
  sendBPacket.len = (13+datalen);

  return &sendBPacket;
}
