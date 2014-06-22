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
 *    Modified:
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 0.0.0
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
#ifdef NETWORK_XBEE
XBEE_APP_CONFIG = { { 0, 0, 0 },{ 0, 0, false, false, 0, 0 } };
#endif

#ifdef NETWORK_UDP
UDP_APP_CONFIG = {{ {0,0,0,0}, 0, {0,0,0,0}, 0, {0,0,0,0,0,0} },{ 0, 0, false, false, 0, 0 } };
#endif
/*==================================================
 *             Create Topic
 *=================================================*/
MQString* topic1 = new MQString("dev/indicator");

/*==================================================
 *             Tasks invoked by Timer
 *=================================================*/
bool led_flg = false;

int task1(){
	if(led_flg){
		PUBLISH(topic1, "off", 3, QOS1);
		led_flg = false;
	}else{
		PUBLISH(topic1, "on", 2, QOS1);
		led_flg = true;
	}
	return 0;
}

/*---------  Link Tasks to the Application --------*/

TASK_LIST = {
	          {task1, 60},

END_OF_TASK_LIST};

/*==================================================
 *      Tasks invoked by PUBLISH command Packet
 *=================================================*/
int on_publish1(MqttsnPublish* msg){
	char buf[] ={0,0,0,0};
	memcpy(buf,msg->getData(),msg->getDataLength());
	printf("Received Message =  %s\n",buf);
	return 0;
}

/*------------ Link Callback to Topic -------------*/

SUBSCRIBE_LIST = {
	                {topic1, on_publish1, QOS1},

END_OF_SUBSCRIBE_LIST};

/*==================================================
 *      Application setup
 *=================================================*/
 void setup(){
    printf("Client start\n");
 }
/*==============    End of Program    ==============*/

#endif  // LINUX
