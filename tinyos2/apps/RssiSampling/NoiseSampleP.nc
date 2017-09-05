/******************************************************************
    NoiseSampleP.nc created by HyungJune Lee (abbado@stanford.edu)
 ******************************************************************/

#include "NoiseSample.h"

module NoiseSampleP
{
	uses interface Boot;
	uses interface Leds;
	uses interface Alarm<T32khz, uint32_t> as Alarm0;
	uses interface Resource;
	uses interface GeneralIO as CSN;
	uses interface CC2420Register as RSSI;
	uses interface SplitControl as RadioControl;
	uses interface SplitControl as SerialControl;
	uses interface AMSend;
	uses interface Packet;
	uses interface BlockRead;
	uses interface BlockWrite;
}

implementation
{
	message_t packet;
	bool sendBusy;
	bool resultReport;
	uint8_t *Buffer;
	uint32_t Buf_len;
	uint32_t Total_len;
	uint32_t Addr_offset;
	uint8_t Buf1[BUF_SIZE];
	uint8_t Buf2[BUF_SIZE];
	uint8_t rdata[BUF_SIZE];
	uint16_t Rssi_val = 0;

	void SystemInit()
	{
		call Leds.led0Off();
		call Leds.led1Off();
		call Leds.led2Off();

		sendBusy = FALSE;
		resultReport = FALSE;
		Buffer = Buf1;
		Buf_len = 0;
		Total_len = 0;
		Addr_offset = 0;
		call RadioControl.start();
	}

	void DataReport(SerialMsg data)
	{
		SerialMsg *msg = (SerialMsg *)(call AMSend.getPayload(&packet,sizeof(SerialMsg)));
		memcpy(msg, &data, sizeof(SerialMsg));

		if (sendBusy == TRUE)
			return;

		if (call AMSend.send(AM_BROADCAST_ADDR, &packet, sizeof(SerialMsg)) == SUCCESS)
		{
			sendBusy = TRUE;
		}
		else
		{
			call Leds.led0On();
			call Leds.led1Off();
			call Leds.led2On();
		}
	}

	void Report(error_t e)
	{
		uint8_t *msg = call AMSend.getPayload(&packet,sizeof(SerialMsg));

		msg[0] = e;
		if (sendBusy == TRUE)
			return;

		if (call AMSend.send(AM_BROADCAST_ADDR, &packet, 1) == SUCCESS)
		{
			sendBusy = TRUE;
		}
		else
		{
			call Leds.led0On();
			call Leds.led1Off();
			call Leds.led2On();
		}
	}

	event void Boot.booted()
	{
		SystemInit();
	}

	task void DataWrite()
	{
		uint8_t *ptr = NULL;
		if (Buffer == Buf1)
			ptr = Buf2;
		else if (Buffer == Buf2)
			ptr = Buf1;

		if (call BlockWrite.write(FLASH_ADDR+Addr_offset, ptr, BUF_SIZE) != SUCCESS)
		{
			Report(0x42);
		}
		else
		{
			Addr_offset += BUF_SIZE;
		}
	}

	task void DataRead()
	{
		if (call BlockRead.read(FLASH_ADDR+Addr_offset, rdata, BUF_SIZE) != SUCCESS)
		{
			Report(0x48);
		}
		else
		{
			Addr_offset += BUF_SIZE;
		}
	}

	task void SendSerial()
	{
		SerialMsg msg;

		msg.NodeId = TOS_NODE_ID;
		msg.SeqNo = Buf_len + 1 + (Addr_offset - BUF_SIZE);
		msg.RssiVal = rdata[Buf_len];
		Buf_len++;
		DataReport(msg);	
	}

	event void Resource.granted()
	{
		call CSN.clr();
		call RSSI.read(&Rssi_val);
		call CSN.set();
		call Resource.release();
		Buffer[Buf_len++] = (uint8_t)(Rssi_val & 0x00ff);
		Total_len++;
		if (Total_len == TOTAL_SIZE)
		{
			call Alarm0.stop();
		}
		if (Buf_len == BUF_SIZE)
		{
			Buf_len = 0;
			if (Buffer == Buf1)
				Buffer = Buf2;
			else if (Buffer == Buf2)
				Buffer = Buf1;
			post DataWrite();
		}
		call Leds.led0Toggle();
		call Leds.led1Toggle();
		call Leds.led2Toggle();
	}

	async event void Alarm0.fired()
	{
		call Resource.request();
		call Alarm0.start(ALARM_PERIOD);
	}

	event void RadioControl.startDone(error_t err)
	{
		if (err != SUCCESS)
		{
			call RadioControl.start();
		}
		else
		{
			call Leds.led0On();  // red light on
			call Leds.led1Off(); // green light off
			call Leds.led2Off(); // yellow light off
			call SerialControl.start();
		}
	}

	event void RadioControl.stopDone(error_t err)
	{
		if (err != SUCCESS)
		{
			call RadioControl.stop();
		}
		else
		{
		}
	}

	event void SerialControl.startDone(error_t err)
	{
		if (err != SUCCESS)
		{
			call SerialControl.start();
		}
		else
		{
			if (call BlockWrite.erase() == SUCCESS)
			{
#if 0
				Report(0x20);
#else
				call Leds.led0Off();
				call Leds.led1On();
				call Leds.led2Off();
#endif
			}
			else
			{
				Report(0x40);
			}
		}
	}

	event void SerialControl.stopDone(error_t err)
	{
		if (err != SUCCESS)
		{
			call SerialControl.stop();
		}
		else
		{
		}
	}

	event void AMSend.sendDone(message_t *bufPtr, error_t err)
	{
		if (&packet == bufPtr)
		{
			sendBusy = FALSE;
			if (resultReport == TRUE)
			{
				if (Buf_len == BUF_SIZE)
				{
					resultReport = FALSE;
					Buf_len = 0;
					if (Addr_offset == TOTAL_SIZE)
					{
						call Leds.led0Off();
						call Leds.led1On();
						call Leds.led2On();
					}
					else
					{
						post DataRead();
					}
				}
				else
				{
					post SendSerial();
				}	
			}
		}
	}

	event void BlockWrite.writeDone(storage_addr_t addr, void *buf, 
									storage_len_t len, error_t err)
	{
		if (err != SUCCESS)
		{
			Report(0x43);
		}
		else
		{
			if (Total_len == TOTAL_SIZE)
			{
				if (call BlockWrite.sync() != SUCCESS)
				{
					Report(0x44);
				}
			}
		}
	}

	event void BlockWrite.eraseDone(error_t err)
	{
		if (err != SUCCESS)
		{
			Report(0x41);
		}
		else
		{
#if 0
			Report(0x21);
#else
			call Leds.led0On();
			call Leds.led1On();
			call Leds.led2Off();
#endif
			call Alarm0.start(ALARM_PERIOD);
		}
	}

	event void BlockWrite.syncDone(error_t err)
	{
		if (err != SUCCESS)
		{
			Report(0x45);
		}
		else
		{
			Addr_offset = 0;
			post DataRead();
		}
	}

	event void BlockRead.readDone(storage_addr_t addr, void *buf, 
									storage_len_t len, error_t err)
	{
		if (err != SUCCESS)
		{
			Report(0x49);
		}
		else
		{
			Buf_len = 0;
			resultReport = TRUE;
			post SendSerial();
		}
	}

	event void BlockRead.computeCrcDone(storage_addr_t addr, storage_len_t len,
										uint16_t z, error_t err)
	{
		if (err != SUCCESS)
		{
		}
	}
}

