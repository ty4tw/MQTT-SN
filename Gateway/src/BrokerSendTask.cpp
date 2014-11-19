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
#include <errno.h>
#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>

extern char* currentDateTime();


BrokerSendTask::BrokerSendTask(GatewayResourcesProvider* res){
	_res = res;
	_res->attach(this);
}

BrokerSendTask::~BrokerSendTask(){
	if(_host){
		delete _host;
	}
	if(_service){
		delete _service;
	}
}

void BrokerSendTask::run(){
	Event* ev = 0;
	MQTTMessage* srcMsg = 0;
	ClientNode* clnode = 0;
	char param[TOMYFRAME_PARAM_MAX];

		if(_res->getParam("BrokerName",param) == 0){
			_host = strdup(param);
		}
		if(_res->getParam("BrokerPortNo",param) == 0){
			_service =strdup( param);
		}
		_light = _res->getLightIndicator();


	while(true){

		uint16_t length = 0;
		memset(_buffer, 0, SOCKET_MAXBUFFER_LENGTH);

		ev = _res->getBrokerSendQue()->wait();

		clnode = ev->getClientNode();
		srcMsg = clnode->getBrokerSendMessage();

		if(srcMsg){
			if(srcMsg->getType() == MQTT_TYPE_PUBLISH){
				MQTTPublish* msg = static_cast<MQTTPublish*>(srcMsg);
				length = msg->serialize(_buffer);
				LOGWRITE(BLUE_FORMAT, currentDateTime(), "PUBLISH", RIGHTARROW, GREEN_BROKER, msgPrint(msg));

				if(send(clnode, length) == 0){
					LOGWRITE(SEND_COMPLETE);
				}
			}else if(srcMsg->getType() == MQTT_TYPE_PUBACK){
				MQTTPubAck* msg = static_cast<MQTTPubAck*>(srcMsg);
				length = msg->serialize(_buffer);
				LOGWRITE(GREEN_FORMAT, currentDateTime(), "PUBACK", RIGHTARROW, GREEN_BROKER, msgPrint(msg));

				if(send(clnode, length) == 0){
					LOGWRITE(SEND_COMPLETE);
				}
			}else if(srcMsg->getType() == MQTT_TYPE_PUBREL){
				MQTTPubRel* msg = static_cast<MQTTPubRel*>(srcMsg);
				length = msg->serialize(_buffer);
				LOGWRITE(GREEN_FORMAT, currentDateTime(), "PUBREL", RIGHTARROW, GREEN_BROKER, msgPrint(msg));

				if(send(clnode, length) == 0){
					LOGWRITE(SEND_COMPLETE);
				}

			}else if(srcMsg->getType() == MQTT_TYPE_PINGREQ){
				MQTTPingReq* msg = static_cast<MQTTPingReq*>(srcMsg);
				length = msg->serialize(_buffer);
				LOGWRITE(FORMAT, currentDateTime(), "PINGREQ", RIGHTARROW, GREEN_BROKER, msgPrint(msg));

				if(send(clnode, length) == 0){
					LOGWRITE(SEND_COMPLETE);
				}
			}else if(srcMsg->getType() == MQTT_TYPE_SUBSCRIBE){
				MQTTSubscribe* msg = static_cast<MQTTSubscribe*>(srcMsg);
				length = msg->serialize(_buffer);
				LOGWRITE(FORMAT, currentDateTime(), "SUBSCRIBE", RIGHTARROW, GREEN_BROKER, msgPrint(msg));

				if(send(clnode, length) == 0){
					LOGWRITE(SEND_COMPLETE);
				}
			}else if(srcMsg->getType() == MQTT_TYPE_UNSUBSCRIBE){
				MQTTUnsubscribe* msg = static_cast<MQTTUnsubscribe*>(srcMsg);
				length = msg->serialize(_buffer);
				LOGWRITE(FORMAT, currentDateTime(), "UNSUBSCRIBE", RIGHTARROW, GREEN_BROKER, msgPrint(msg));

				if(send(clnode, length) == 0){
					LOGWRITE(SEND_COMPLETE);
				}
			}else if(srcMsg->getType() == MQTT_TYPE_CONNECT){
				MQTTConnect* msg = static_cast<MQTTConnect*>(srcMsg);
				length = msg->serialize(_buffer);
				LOGWRITE(FORMAT, currentDateTime(), "CONNECT", RIGHTARROW, GREEN_BROKER, msgPrint(msg));

				if(send(clnode, length) == 0){
					LOGWRITE(SEND_COMPLETE);
				}
			}else if(srcMsg->getType() == MQTT_TYPE_DISCONNECT){
				MQTTDisconnect* msg = static_cast<MQTTDisconnect*>(srcMsg);
				length = msg->serialize(_buffer);
				LOGWRITE(FORMAT, currentDateTime(), "DISCONNECT", RIGHTARROW, GREEN_BROKER, msgPrint(msg));

				if(send(clnode, length) == 0){
					LOGWRITE(SEND_COMPLETE);
				}
				clnode->getStack()->disconnect();
			}
		}
		delete ev;
	}
}


int BrokerSendTask::send(ClientNode* clnode, int length){
	int rc = 0;

	if(length <= 0){
		return -1;
	}

	if( clnode->getStack()->isValid()){
		rc = clnode->getStack()->send(_buffer, length);
		if(rc == -1){
			LOGWRITE("\nSocket is valid. but BrokerSendTask can't send to the Broker. errno=%d\n", errno);
			clnode->getStack()->disconnect();
			clnode->disconnected();
			return -1;
		}else{
			_light->greenLight(true);
		}
	}else{
		if(clnode->getStack()->connect(_host, _service)){
			rc = clnode->getStack()->send(_buffer, length);
			if(rc == -1){
				LOGWRITE("\nSocket is valid. but BrokerSendTask can't send to the Broker. errno=%d\n", errno);
				clnode->getStack()->disconnect();
				clnode->disconnected();
				return -1;
			}else{
				_light->greenLight(true);
			}
		}else{
			LOGWRITE("\n%s error: BrokerSendTask can't connect to the Broker.\n", currentDateTime());
			clnode->getStack()->disconnect();
			clnode->disconnected();
			return -1;
		}
	}
	return 0;
}

char*  BrokerSendTask::msgPrint(MQTTMessage* msg){
	char* buf = _printBuf;
	_light->blueLight(true);

	sprintf(buf, " %02X", *_buffer);
	buf += 3;

	for(int i = 0; i < msg->getRemainLength() + msg->getRemainLengthSize(); i++){
		sprintf(buf, " %02X", *( _buffer + 1  + i));
		buf += 3;
	}
	*buf = 0;
	_light->blueLight(false);
	return _printBuf;
}

