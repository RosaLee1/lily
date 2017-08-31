/*
 * Copyright (c) 2005-2006 Rincon Research Corporation
 * All rights reserved.
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
 * - Neither the name of the Rincon Research Corporation nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * RINCON RESEARCH OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE
 */
 
#include "Timer.h"
#include "RssiToSerial.h"
#include "printf.h"
 
 /**
  * This is more of a general demonstration than a test.
  *
  * Install this application to one node, connected to the computer.
  * The node will measure the environmental RSSI from the CC2420 and
  * sending those readings over the serial port.
  *
  * Use the Java application to display the relative RSSI readings.
  *
  * @author Jared Hill
 * @date   23 March 2007
  */
 
module RssiToSerialP {
  uses {
    interface Leds;
    interface Boot;
   /** interface AMSend; **/
    interface SplitControl as AMControl;
    interface SplitControl as SerialControl; 
   /** interface Packet;  **/
    interface Read<uint16_t> as ReadRssi;
    interface CC2420Config as Config;
    interface LocalTime<TMicro>;
    interface LogRead;
    interface LogWrite;
  }
}
implementation {

  typedef nx_struct logentry_t {
    nx_uint16_t rss;
  } logentry_t;

  logentry_t m_entry;

  /******* Global Variables ****************/
  uint16_t reads;
  uint32_t time;
  bool m_busy = TRUE; // TRUE: cannot write to log; FALSE: can write to log
  
  /******** Declare Tasks *******************/
  task void readRssi();
  task void getTime();
  
  /************ Boot Events *****************/
  event void Boot.booted() {
    call AMControl.start();
    reads = 0;
    time = 0;
  }

  /************ AMControl Events ******************/
  event void AMControl.startDone(error_t err) {
    if (err == SUCCESS) {
      call SerialControl.start(); 
    }
    else {
      call AMControl.start();
    }
  }

  event void AMControl.stopDone(error_t err) {
    // do nothing
  }
  
  /***************SerialControl Events*****************/
  event void SerialControl.startDone(error_t error){
    if (error == SUCCESS) {
      post readRssi();
    }
    else {
      call AMControl.start();
    }
  }

  event void SerialControl.stopDone(error_t error){
    //do nothing
  }
  
  /**************** ReadRssi Events *************************/
  event void ReadRssi.readDone(error_t result, uint16_t val ){
    
    if(result != SUCCESS){
      post readRssi();
      return;
    }

    atomic{
      reads ++;
      m_entry.rss = val;
     // call LogWrite.append(&m_entry, sizeof(logentry_t));
call LogRead.read(&m_entry, sizeof(logentry_t));
    } 

    if(reads == (1<<LOG2SAMPLES)){
      // cannot write to log before finish
      m_busy = TRUE;

 //     // send end time of rss reading to terminal
 //     post getTime();

      // read log 
      if (call LogRead.read(&m_entry, sizeof(logentry_t)) != SUCCESS) {
	// Handle error.
      }
    }
    
    if(!m_busy){
      post readRssi();
    }
    
  }

 /********************* log append Events *************************/
  event void LogWrite.appendDone(void* buf, storage_len_t len, 
                                 bool recordsLost, error_t err) {
      bool m_busy = FALSE;
      call Leds.led1Toggle(); // green light
  }

  /********************* log read Events *************************/
  event void LogRead.readDone(void* buf, storage_len_t len, error_t err) {
    if ( (len == sizeof(logentry_t)) && (buf == &m_entry) ) {
      printf(" %u ", m_entry.rss);
      printfflush();
      call LogRead.read(&m_entry, sizeof(logentry_t));
    }
    else {
      if (call LogWrite.erase() != SUCCESS) {
	// Handle error.
      }
      call Leds.led0On(); // red light is on
    }
  }

  /********************* log erase Events *************************/
  event void LogWrite.eraseDone(error_t err) {
    if (err == SUCCESS) {
      // reset reads
      reads=0;

      // can write to log before finish
      m_busy = FALSE;

//      // send start time of rss reading to terminal
//      post getTime();

//      post readRssi();
    }
    else {
      // Handle error.
    }

    call Leds.led0Off(); // red light is off
  }

  event void LogRead.seekDone(error_t err) {
  }

  event void LogWrite.syncDone(error_t err) {
  }
  
  /********************* Config Events *************************/
  event void Config.syncDone(error_t error){
  
  }

  /***************** TASKS *****************************/  
  task void getTime(){
      time = call LocalTime.get();
      printf("\n %lu \n", time);
      printfflush();
  }

  task void readRssi(){
   
    if(call ReadRssi.read() != SUCCESS){
      post readRssi();
    }
  }

}



