/*
 * Copyright (c) 2002, Vanderbilt University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the copyright holders nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @author: Miklos Maroti, Brano Kusy (kusy@isis.vanderbilt.edu)
 * Ported to T2: 3/17/08 by Brano Kusy (branislav.kusy@gmail.com)
 */

#include "TestFtsp.h"
#include "RadioCountToLeds.h"
#include "Timer.h"
#include "printf.h"

module TestFtspC
{
    uses
    {
        interface GlobalTime<TMilli>;
        interface TimeSyncInfo;
        interface Receive;
        interface AMSend;
        interface Packet;
        interface Leds;
        interface PacketTimeStamp<TMilli,uint32_t>;
        interface Boot;
        interface SplitControl as RadioControl;
        interface SplitControl as SerialControl; 
        // microsecond local time
        interface LocalTime<TMicro>;
        interface LogRead;
        interface LogWrite;
        interface Read<uint16_t> as ReadRssi;
        interface CC2420Config as Config;
    }
}

implementation
{
    message_t my_msg;

    bool m_busy = TRUE;
    logentry_t m_entry;

    uint16_t my_counter;
    uint32_t rxTimestamp;
    uint16_t writes;

    /******** Declare Tasks *******************/
    task void readRssi();

    event void Boot.booted() {
        call RadioControl.start();
        call SerialControl.start(); 
        my_counter=0;
        rxTimestamp=0;
        writes = 1;
    }

    event void RadioControl.startDone(error_t err) {
      if (err == SUCCESS) {
        if (call LogRead.read(&m_entry, sizeof(logentry_t)) != SUCCESS) {
	  // Handle error.
        }
      }
      else {
        call RadioControl.start();
      }
    }

    event void RadioControl.stopDone(error_t error){}

    event void SerialControl.startDone(error_t error) {}
    event void SerialControl.stopDone(error_t error) {}

    event void LogRead.readDone(void* buf, storage_len_t len, error_t err) {
      if ( (len == sizeof(logentry_t)) && (buf == &m_entry) ) {
        logentry_t* logptr = (logentry_t*)(call Packet.getPayload(&my_msg, sizeof(logentry_t)));
        logptr->src_addr = m_entry.src_addr;
        logptr->counter = m_entry.counter;
        logptr->local_rx_timestamp = m_entry.local_rx_timestamp;
        logptr->rss = m_entry.rss;
        call AMSend.send(AM_BROADCAST_ADDR, &my_msg, sizeof(logentry_t));
        call Leds.led1On();  // green light is on
      }
      else {
        if (call LogWrite.erase() != SUCCESS) {
	  // Handle error.
        }
        call Leds.led0On(); //red light is on
      }
    }

    event void AMSend.sendDone(message_t* ptr, error_t err) {
      call Leds.led1Off(); // green light is off
      if ((err == SUCCESS) && (ptr == &my_msg)) {
        call Packet.clear(&my_msg);
        if (call LogRead.read(&m_entry, sizeof(logentry_t)) != SUCCESS) {
	  // Handle error.
        }
      }

      return;
    }

    event void LogWrite.eraseDone(error_t err) {
      if (err == SUCCESS) {
        m_busy = FALSE;
      }
      else {
        // Handle error.
      }
      call Leds.led0Off(); //red light is off
    }

    event message_t* Receive.receive(message_t* msgPtr, void* payload, uint8_t len)
    {
        printf(" (received) ");
        call Leds.led0Toggle(); // red light
      
        if (call PacketTimeStamp.isValid(msgPtr)) {
            radio_count_msg_t* rcm = (radio_count_msg_t*)call Packet.getPayload(msgPtr, sizeof(radio_count_msg_t));
            rxTimestamp = call LocalTime.get();
            my_counter=rcm->counter;
 	    printf(" (valid) ");
	    writes = 1;

	    post readRssi();
        }

        return msgPtr;
    }

    event void ReadRssi.readDone(error_t result, uint16_t val ){
    
      if(result != SUCCESS){
        post readRssi();
        return;
      }

      call Leds.led2On(); // blue light is on
      if (!m_busy) {
      	m_busy = TRUE;
        m_entry.src_addr = TOS_NODE_ID;
        m_entry.counter = my_counter;
	m_entry.local_rx_timestamp = rxTimestamp;
        m_entry.rss = val;
      printf("\n (%u |%u :%u :%u :%u) \n",writes,m_entry.src_addr,m_entry.counter,m_entry.local_rx_timestamp,m_entry.rss);
      printfflush();
        if (call LogWrite.append(&m_entry, sizeof(logentry_t)) != SUCCESS) {
          m_busy = FALSE;
        }
      }   
    
    }

    event void LogWrite.appendDone(void* buf, storage_len_t len, 
                                 bool recordsLost, error_t err) {
      m_busy = FALSE;
      call Leds.led2Off(); // blue light is off

      atomic{
        writes ++;
      }
    
      if(writes > (1<<LOG2SAMPLES)){
        return;
      } else {
        post readRssi();
      }
    }

    event void LogRead.seekDone(error_t err) {}
    event void LogWrite.syncDone(error_t err) {}
    event void Config.syncDone(error_t error) {}

    /***************** TASKS *****************************/  
    task void readRssi(){
      if(call ReadRssi.read() != SUCCESS){
        post readRssi();
      }
    }

}
