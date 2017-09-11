/******************************************************************
    NoiseSample.h created by HyungJune Lee (abbado@stanford.edu)
 ******************************************************************/

#ifndef _NOISESAMPLE_H_
#define _NOISESAMPLE_H_


enum
{
	ALARM_PERIOD = 8, //i.e. 4KHz
	AM_SERIAL_ID = 6,
	BUF_SIZE = 512,
	TOTAL_SIZE = 512,
	FLASH_ADDR = 0x00,
};

typedef nx_struct Msg
{
	nx_uint16_t NodeId;
	nx_uint32_t SeqNo;
	nx_uint8_t RssiVal;
} SerialMsg;

#endif
