/*
 * Copyright (c) 2008 Dimas Abreu Dutra
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the Stanford University nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL DIMAS ABREU
 * DUTRA OR HIS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @author Dimas Abreu Dutra
 */

#include "RssiDemoMessages.h"
#include "RadioCountToLeds.h"
#include "printf.h"

module SendingMoteC {
  uses interface Boot;
  uses interface Timer<TMilli> as SendTimer;
  uses interface Leds;
  
  uses interface AMSend as RssiMsgSend;
  uses interface Receive;
  uses interface SplitControl as RadioControl;
} implementation {
  message_t msg;
  uint16_t  packet_num = 0;
  uint16_t  counter_packet_num = 1;
  
  /******** Declare Tasks *******************/
  task void sendMsg();
  
  event void Boot.booted(){
    call RadioControl.start();
  }

  event void RadioControl.startDone(error_t result){}
  
  event void RadioControl.stopDone(error_t result){}
  
  event message_t* Receive.receive(message_t* msgPtr, void* payload, uint8_t len)
  {
    
    call SendTimer.startOneShot(10);
    
    printf(" (%u ", counter_packet_num);
    printfflush();
    
    counter_packet_num++;
    
    if(counter_packet_num == (1<<LOG2SAMPLES)){ 
        counter_packet_num = 1;
    }
	return msgPtr;
  }
  
  event void SendTimer.fired(){
    post sendMsg();
  }

  event void RssiMsgSend.sendDone(message_t *m, error_t error){
  
    packet_num++;
    
    if(packet_num == 18){ // stop sending packets
      call Leds.led1Toggle(); // green light
      
      printf(" finish) ");
      printfflush();
    
      packet_num = 0;
      return;
    }
    
    post sendMsg();
  }

  /***************** TASKS *****************************/  
  task void sendMsg(){
    call RssiMsgSend.send(AM_BROADCAST_ADDR, &msg, sizeof(sender_msg_t)); 
  }

}
