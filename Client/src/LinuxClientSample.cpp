/*
 * LinuxClientSample.cpp
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

#include "lib/MQTTSN_Application.h"

#ifdef LINUX
#include "lib/mqttsnClientAppFw4Linux.h"
#include <stdio.h>
#include <string.h>

using namespace std;
using namespace tomyClient;

/*============================================
 *
 *   MQTT-SN Client Application for Linux
 *
 *===========================================*/

/*------------------------------------------------------
 *             Create Topic
 *------------------------------------------------------*/

MQString* tp1 = new MQString("topic/01");
MQString* tp2 = new MQString("topic/02");
MQString* tp3 = new MQString("topic/03");
MQString* tp4 = new MQString("topic/04");
MQString* tp5 = new MQString("topic/05");

MQString* tp_request_01 = new MQString("topic/req/01");
MQString* tp_response_01 = new MQString("topic/rsp/01");

MQString* tp_request_02 = new MQString("topic/req/02");
MQString* tp_response_02 = new MQString("topic/rsp/02");

/*------------------------------------------------------
 *             Tasks invoked by Timer
 *------------------------------------------------------*/

int f_publish_all(){
  PUBLISH(tp1,"topic/01", 8,QOS1);
  PUBLISH(tp2,"topic/02", 8,QOS1);
  PUBLISH(tp3,"topic/03", 8,QOS1);
  PUBLISH(tp4,"topic/04", 8,QOS1);
  PUBLISH(tp5,"topic/05", 8,QOS1);

  Payload pl = Payload(40);
  pl.set_uint32( 200);
  pl.set_uint32( 10);
  pl.set_uint32( 50000);
  pl.set_uint32( 70000);
  pl.set_int32( -300);
  pl.set_int32(-70000);
  pl.set_float((float)1000.01);
  pl.set_str("abcdef");
  PUBLISH(tp_request_01,&pl,1);

  return 0;
}


/*---------------  List of task invoked by Timer ------------*/

TASK_LIST = {  //{ MQString* topic, executing duration in second},
  {f_publish_all, 10},
  END_OF_TASK_LIST
};


/*------------------------------------------------------
 *       Tasks invoked by PUBLISH command Packet
 *------------------------------------------------------*/
/*----- Topic list used in callback functions -----*/
TOPICS_IN_CALLBACK = {
	tp_response_01,
	tp_response_02,
	END_OF_TOPICS
};

int f_onRequest_01(MqttsnPublish* msg){
	printf("inside f_onRequest_01()\n");
	Payload pl;
	pl.getPayload(msg);
	pl.print();
	printf("uint8: %d\n",pl.get_uint32(0));
	printf("uint16: %d\n",pl.get_uint32(1));
	printf("uint32: %d\n",pl.get_uint32(2));
	printf("int8: %d\n",pl.get_uint32(3));
	printf("int16: %d\n",pl.get_int32(4));
	printf("int32: %d\n",pl.get_int32(5));
	printf("float: %g \n",pl.get_float(6));
	uint16_t len;
	printf("str: %s  \n",pl.get_str(7,&len));

  PUBLISH(tp_response_01,&pl,QOS1);
  return 0;
}

int f_onRequest_02(MqttsnPublish* msg){
	printf("inside f_onRequest_02()\n");
  PUBLISH(tp_response_02,"topic/rsp/02", 12,1);
  return 0;
}

/*-------------- List of Task invoked by PUBLISH  -----------*/

SUBSCRIBE_LIST = { //{ MQString* topic, on_publish1, QOS },
  {tp_request_01, f_onRequest_01, QOS1},
  {tp_request_02, f_onRequest_02, QOS1},
  END_OF_SUBSCRIBE_LIST
};


/*==================================================
 *      Application setup
 *=================================================*/
 void setup(){
    printf("Client start\n");
 }
/*==============    End of Program    ==============*/

#endif  // LINUX
