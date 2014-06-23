/*
 * BrokerSendTask.cpp
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

#include "BrokerSendTask.h"
#include "GatewayResourcesProvider.h"
#include "GatewayDefines.h"
#include "lib/ProcessFramework.h"
#include "lib/Messages.h"
#include "lib/Defines.h"
#include <string.h>
#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>

extern char* currentDateTime();


BrokerSendTask::BrokerSendTask(GatewayResourcesProvider* res){
	_res = res;
	_res->attach(this);
}

BrokerSendTask::~BrokerSendTask(){

}


void BrokerSendTask::run(){
	Event* ev = 0;
	MQTTMessage* srcMsg = 0;
	ClientNode* clnode = 0;

	uint8_t buffer[SOCKET_MAXBUFFER_LENGTH];

	const char* host = BROKER_HOST_NAME;
	const char* service = BROKER_PORT;


	if(_res->getArgv('h') != 0){
		host = strdup(_res->getArgv('h'));
	}
	if(_res->getArgv('p') != 0){
		service =strdup( _res->getArgv('p'));
	}

	while(true){

		uint16_t length = 0;
		memset(buffer, 0, SOCKET_MAXBUFFER_LENGTH);

		ev = _res->getBrokerSendQue()->wait();

		clnode = ev->getClientNode();
		srcMsg = clnode->getBrokerSendMessage();

		if(srcMsg->getType() == MQTT_TYPE_PUBLISH){
			MQTTPublish* msg = static_cast<MQTTPublish*>(srcMsg);
			length = msg->serialize(buffer);
			printf(BLUE_FORMAT1, currentDateTime(), "PUBLISH", RIGHTARROW, BROKER, msgPrint(buffer, msg));
		}
		else if(srcMsg->getType() == MQTT_TYPE_PUBACK){
			MQTTPubAck* msg = static_cast<MQTTPubAck*>(srcMsg);
			length = msg->serialize(buffer);
			printf(GREEN_FORMAT1, currentDateTime(), "PUBACK", RIGHTARROW, BROKER, msgPrint(buffer, msg));
		}
		else if(srcMsg->getType() == MQTT_TYPE_PUBREL){
			MQTTPubRel* msg = static_cast<MQTTPubRel*>(srcMsg);
			length = msg->serialize(buffer);
			printf(GREEN_FORMAT1, currentDateTime(), "PUBREL", RIGHTARROW, BROKER, msgPrint(buffer, msg));
		}
		else if(srcMsg->getType() == MQTT_TYPE_PINGREQ){
			MQTTPingReq* msg = static_cast<MQTTPingReq*>(srcMsg);
			length = msg->serialize(buffer);
			printf(FORMAT1, currentDateTime(), "PINGREQ", RIGHTARROW, BROKER, msgPrint(buffer, msg));

		}
		else if(srcMsg->getType() == MQTT_TYPE_SUBSCRIBE){
			MQTTSubscribe* msg = static_cast<MQTTSubscribe*>(srcMsg);
			length = msg->serialize(buffer);
			printf(FORMAT1, currentDateTime(), "SUBSCRIBE", RIGHTARROW, BROKER, msgPrint(buffer, msg));
		}
		else if(srcMsg->getType() == MQTT_TYPE_UNSUBSCRIBE){
			MQTTUnsubscribe* msg = static_cast<MQTTUnsubscribe*>(srcMsg);
			length = msg->serialize(buffer);
			printf(FORMAT1, currentDateTime(), "UNSUBSCRIBE", RIGHTARROW, BROKER, msgPrint(buffer, msg));

		}
		else if(srcMsg->getType() == MQTT_TYPE_CONNECT){
			MQTTConnect* msg = static_cast<MQTTConnect*>(srcMsg);
			length = msg->serialize(buffer);
			printf(FORMAT1, currentDateTime(), "CONNECT", RIGHTARROW, BROKER, msgPrint(buffer, msg));
		}
		else if(srcMsg->getType() == MQTT_TYPE_DISCONNECT){
			MQTTDisconnect* msg = static_cast<MQTTDisconnect*>(srcMsg);
			length = msg->serialize(buffer);
			printf(FORMAT1, currentDateTime(), "DISCONNECT", RIGHTARROW, BROKER, msgPrint(buffer, msg));
		}

		int rc = 0;

		if(length > 0){
			if( clnode->getSocket()->isValid()){
				rc = clnode->getSocket()->send(buffer, length);
				if(rc == -1){
					clnode->getSocket()->disconnect();
					printf("Socket is valid. but can't send to Client:%s\n", clnode->getNodeId()->c_str());
				}
			}else{
				if(clnode->getSocket()->connect(host, service)){
					rc = clnode->getSocket()->send(buffer, length);
					if(rc == -1){
						clnode->getSocket()->disconnect();
						printf("Socket is created. but can't send to Client:%s\n", clnode->getNodeId()->c_str());
					}
				}else{
					printf("%s Can't connect to Client:%s\n",
							currentDateTime(), clnode->getNodeId()->c_str());
				}
			}
		}
		delete ev;
	}
}


char*  BrokerSendTask::msgPrint(uint8_t* buffer, MQTTMessage* msg){
	char* buf = _printBuf;

	sprintf(buf, " %02X", *buffer);
	buf += 3;

	for(int i = 0; i < msg->getRemainLength() + msg->getRemainLengthSize(); i++){
		//sprintf(buf, " %02X", *( buffer + 1 + msg->getRemainLengthSize() + i));
		sprintf(buf, " %02X", *( buffer + 1  + i));
		buf += 3;
	}
	*buf = 0;
	return _printBuf;
}

