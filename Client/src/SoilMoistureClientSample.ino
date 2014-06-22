/*
 * SoilMoisture.ino
 *
 *                      The BSD License
 *
 *           Copyright (c) 2014, tomoaki@tomy-tech.com
 *                    All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: 2014/06/01
 *    Modified:
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 0.0.0
 */

#include <MQTTSN_Application.h>
#include <mqttsnClientAppFw4Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
/*============================================
 *
 *   MQTT-SN Client Application for Arduino
 *
 *===========================================*/
 #ifdef NETWORK_XBEE
 XBEE_APP_CONFIG = {
    {
        9600,           //Baudrate
        0,              //Serial PortNo (for Arduino App)
        0               //Device (for linux App)
    }, 
    {             
        "ARD01",       //ClientId
        300,            //KeepAlive
        true,           //Clean session
        true,           //EndDevice
        "willTopic",    //WillTopic   or 0   DO NOT USE NULL STRING "" !
        "willMessage"   //WillMessage or 0   DO NOT USE NULL STRING "" !
    }
 };
#endif

#ifdef NETWORK_UDP
UDP_APP_CONFIG = {
    { 
        {225,1,1,1},         // Multicast group IP
        1883,                // Multicast group Port
        {192,168,11,18},     // Local IP     (for Arduino App)
        12001,               // Local PortNo
        {0x0,0x0,0x0,0x0,0x0,0x0}   // MAC address  (for Arduino App)
    },
    { 
        "ARD01",       //ClientId
        300,            //KeepAlive
        true,           //Clean session
        true,           //EndDevice
        "willTopic",    //WillTopic   or 0   DO NOT USE NULL STRING "" !
        "willMessage"   //WillMessage or 0   DO NOT USE NULL STRING "" !
    } 
};
#endif
/*------------------------------------------------------
 *             Create Topic
 *------------------------------------------------------*/
MQString* tpMeasure = new MQString("ty4tw/resistance");


/*------------------------------------------------------
 *             Tasks invoked by Timer
 *------------------------------------------------------*/
#define PIN5  5    // 3.3V supply port 
#define MCOUNT 10  // measurement count
#define PIN0  0    // measurement port
#define RP   20    // resistance [K ohom] 

int measure(){
  int val = 0;
  pinMode(PIN5,OUTPUT);
  digitalWrite(PIN5,1);
  
  for(uint8_t cnt = 0; cnt < MCOUNT; cnt++){
    delay(50);
    val += analogRead(PIN0);
  }
  digitalWrite(PIN5,0);
  int soilR = 1023 * RP / (val /MCOUNT) - RP;
  if(soilR < 0){
    soilR = 9999;
  }
  char buf[30];
  //GET_DATETIME(buf);
  //sprintf(buf + 17," %4d[Kohom]",soilR);
  return PUBLISH(tpMeasure,buf, 29,1);
}


/*---------------  List of task invoked by Timer ------------*/

TASK_LIST = {  //{ MQString* topic, executing duration in second},
            {measure, 40},
END_OF_TASK_LIST};


/*------------------------------------------------------
 *       Tasks invoked by PUBLISH command Packet
 *------------------------------------------------------*/
//void on_publish1(MqttsnPublish* msg){
//
//} 

/*-------------- List of Task invoked by PUBLISH  -----------*/

SUBSCRIBE_LIST = { //{ MQString* topic, on_publish1, QOS1 },

END_OF_SUBSCRIBE_LIST};

/*------------------------------------------------------
 *            Tasks invoked by INT0 interuption 
 *------------------------------------------------------*/
void interruptCallback(){
  //NOP
}

/*------------------------------------------------------
 *            Arduino setup() function 
 *------------------------------------------------------*/
 void setup(){
    // nop
 }


