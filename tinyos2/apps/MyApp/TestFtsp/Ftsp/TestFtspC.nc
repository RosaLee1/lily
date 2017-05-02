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
        // microsecond local time
        interface LocalTime<TMicro>;
        interface LogRead;
        interface LogWrite;
    }
}

implementation
{
    message_t msg;
    message_t my_msg;

    bool locked = FALSE;

    bool m_busy = TRUE;
    logentry_t m_entry;

    event void Boot.booted() {
        call RadioControl.start();
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

    event void LogRead.readDone(void* buf, storage_len_t len, error_t err) {
      if ( (len == sizeof(logentry_t)) && (buf == &m_entry) ) {
        logentry_t* logptr = (logentry_t*)(call Packet.getPayload(&my_msg, sizeof(logentry_t)));
        logptr->src_addr = m_entry.src_addr;
        logptr->counter = m_entry.counter;
        logptr->local_rx_timestamp = m_entry.local_rx_timestamp;
        call AMSend.send(AM_BROADCAST_ADDR, &my_msg, sizeof(logentry_t));
        call Leds.led1On();  
      }
      else {
        if (call LogWrite.erase() != SUCCESS) {
	  // Handle error.
        }
        call Leds.led0On(); //red light is on
      }
    }

    event void AMSend.sendDone(message_t* ptr, error_t err) {
      call Leds.led1Off();
      if ((err == SUCCESS) && (ptr == &my_msg)) {
        call Packet.clear(&my_msg);
        if (call LogRead.read(&m_entry, sizeof(logentry_t)) != SUCCESS) {
	  // Handle error.
        }
      }

      locked = FALSE;
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
        call Leds.led0Toggle(); // red light
        //Temp = call LocalTime.get();
        if (!locked && call PacketTimeStamp.isValid(msgPtr)) {
            radio_count_msg_t* rcm = (radio_count_msg_t*)call Packet.getPayload(msgPtr, sizeof(radio_count_msg_t));
            logentry_t* report = (logentry_t*)call Packet.getPayload(&msg, sizeof(logentry_t));

            uint32_t rxTimestamp = call LocalTime.get();

            report->src_addr = TOS_NODE_ID;
            report->counter = rcm->counter;
            report->local_rx_timestamp = rxTimestamp;

	    ///////////broadcast for basestation///////////////
            if (call AMSend.send(AM_BROADCAST_ADDR, &msg, sizeof(logentry_t)) == SUCCESS) {
              locked = TRUE;
            }
	    //-----------------------------------------//

        }

        return msgPtr;
    }

    event void LogWrite.appendDone(void* buf, storage_len_t len, 
                                 bool recordsLost, error_t err) {}

    event void LogRead.seekDone(error_t err) {}
    event void LogWrite.syncDone(error_t err) {}

}
