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
 *    Modified: 2014/09/05
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 1.0.0
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
        "ARD02",       //ClientId
        300,            //KeepAlive
        true,           //Clean session
        false,           //EndDevice
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
        {0x90,0xa2,0xda,0x0f,0x53,0xa5}       // MAC address  (for Arduino App)
    },
    { 
        "ARD02",       //ClientId
        300,            //KeepAlive
        true,           //Clean session
        false,           //EndDevice
        "willTopic",    //WillTopic   or 0   DO NOT USE NULL STRING "" !
        "willMessage"   //WillMessage or 0   DO NOT USE NULL STRING "" !
    } 
};
#endif

/*------------------------------------------------------
 *             Create Topic
 *------------------------------------------------------*/
MQString* topic1 = new MQString("ty4tw/tp1");
MQString* topic2 = new MQString("ty4tw/tp2");
MQString* tpMeasure = new MQString("ty4tw/soilReg");
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
  Payload pl(30);
  pl.set_array(3);
  pl.set_uint32(GETUTC());
  pl.set_int32(soilR);
  pl.set_str("Kohom");
  return PUBLISH(tpMeasure,&pl,QOS1);
}


int task1(){
  Payload pl = Payload(36);
  pl.set_array(9);
  pl.set_int32(30);
  pl.set_int32(255);
  pl.set_int32(70000);
  pl.set_str("abcdef");
  pl.set_int32(-16);
  pl.set_int32(-60);
  pl.set_int32(-300);
  pl.set_int32(-70000);
  pl.set_float(1000.01);
  return PUBLISH(topic1,&pl,QOS1);
}

/*---------------  List of task invoked by Timer ------------*/

TASK_LIST = {  //{ MQString* topic, executing duration in second},
           {measure, 40},
           {task1,6},
           //{task2,6},
END_OF_TASK_LIST};


/*------------------------------------------------------
 *       Tasks invoked by PUBLISH command Packet
 *------------------------------------------------------*/
 TOPICS_IN_CALLBACK = {

   END_OF_TOPICS
 };
 

int on_publish2(MqttsnPublish* msg){
    theApplication->indicatorOff();
    return 0;
}

/*------------ Link Callback to Topic -------------*/

SUBSCRIBE_LIST = {
                    //{topic1, on_publish1, QOS1},
                    //{topic2, on_publish2, QOS1},

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

 }


