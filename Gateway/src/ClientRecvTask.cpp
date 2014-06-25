/*
 * ClientRecvTask.cpp
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
#include "ClientRecvTask.h"
#include "GatewayResourcesProvider.h"
#include "GatewayDefines.h"
#include "lib/ProcessFramework.h"
#include "lib/Messages.h"
#include "lib/ZBStack.h"
#include "lib/ErrorMessage.h"


#include <unistd.h>
#include <iostream>
#include <fcntl.h>
#include <stdlib.h>

ClientRecvTask::ClientRecvTask(GatewayResourcesProvider* res){
	_res = res;
	_res->attach(this);
}

ClientRecvTask::~ClientRecvTask(){

}



void ClientRecvTask::run(){

#ifdef NETWORK_XBEE
	XBeeConfig config;
	config.baudrate = B57600;
	config.device = _res->getArgv('d');
	config.flag = O_RDONLY;

	_res->getClientList()->authorize(FILE_NAME_CLIENT_LIST);
	_network = new Network();
#endif

#ifdef NETWORK_UDP
	UdpConfig config;
	config.ipAddress = _res->getArgv('g');
	config.gPortNo = atoi(_res->getArgv('p'));
	config.uPortNo = atoi(_res->getArgv('u'));
	_network = _res->getNetwork();
#endif

#ifdef NETWORK_XXXXX
	XXXXXConfig config;
	config.param1 = _res->getArgv('X');
	config.param2 = atoi(_res->getArgv('X'));
	config.param3 = atoi(_res->getArgv('X'));
	_network = _res->getNetwork();
#endif


	if(_network->initialize(config) < 0){
		THROW_EXCEPTION(ExFatal, ERRNO_SYS_02, "can't open the client port.");  // ABORT
	}

	while(true){

		NWResponse* resp = new NWResponse();
		bool eventSetFlg = true;

		if(_network->getResponse(resp)){
			Event* ev = new Event();
			ClientNode* clnode = _res->getClientList()->getClient(resp->getClientAddress64(),
					                                              resp->getClientAddress16());

			if(!clnode){
				if(resp->getMsgType() == MQTTSN_TYPE_CONNECT){

				#ifdef NETWORK_XBEE
					ClientNode* node = _res->getClientList()->createNode(resp->getClientAddress64(),0);
				#endif
				#ifdef NETWORK_UDP
					ClientNode* node = _res->getClientList()->createNode(resp->getClientAddress64(),
																	resp->getClientAddress16());
				#endif
				#ifdef NETWORK_XXXXX
					ClientNode* node = _res->getClientList()->createNode(resp->getClientAddress64(),
																	resp->getClientAddress16());
				#endif

					if(!node){
						delete ev;
						printf("Client is not authorized.\n");
						continue;
					}

					MQTTSnConnect* msg = new MQTTSnConnect();
					msg->absorb(resp);
					node->setClientAddress16(resp->getClientAddress16());
					if(msg->getClientId()->size() > 0){
						node->setNodeId(msg->getClientId());
					}
					node->setClientRecvMessage(msg);

					ev->setClientRecvEvent(node);
				}else if(resp->getMsgType() == MQTTSN_TYPE_SEARCHGW){
					MQTTSnSearchGw* msg = new MQTTSnSearchGw();
					msg->absorb(resp);
					ev->setEvent(msg);

				}else{
					eventSetFlg = false;
				}
			}else{
				if (resp->getMsgType() == MQTTSN_TYPE_CONNECT){
					MQTTSnConnect* msg = new MQTTSnConnect();
					msg->absorb(resp);
					clnode->setClientRecvMessage(msg);
					clnode->setClientAddress16(resp->getClientAddress16());
					ev->setClientRecvEvent(clnode);

				}else if(resp->getMsgType() == MQTTSN_TYPE_PUBLISH){
					MQTTSnPublish* msg = new MQTTSnPublish();
					msg->absorb(resp);
					clnode->setClientRecvMessage(msg);
					ev->setClientRecvEvent(clnode);

				}else if(resp->getMsgType() == MQTTSN_TYPE_PUBACK){
					MQTTSnPubAck* msg = new MQTTSnPubAck();
					msg->absorb(resp);
					clnode->setClientRecvMessage(msg);
					ev->setClientRecvEvent(clnode);

				}else if(resp->getMsgType() == MQTTSN_TYPE_PUBREL){
					MQTTSnPubRel* msg = new MQTTSnPubRel();
					msg->absorb(resp);
					clnode->setClientRecvMessage(msg);
					ev->setClientRecvEvent(clnode);

				}else if (resp->getMsgType() == MQTTSN_TYPE_CONNECT){
					MQTTSnConnect* msg = new MQTTSnConnect();
					msg->absorb(resp);
					clnode->setClientRecvMessage(msg);
					ev->setClientRecvEvent(clnode);

				}else if (resp->getMsgType() == MQTTSN_TYPE_PINGREQ){
					MQTTSnPingReq* msg = new MQTTSnPingReq();
					msg->absorb(resp);
					clnode->setClientRecvMessage(msg);
					ev->setClientRecvEvent(clnode);

				}else if (resp->getMsgType() == MQTTSN_TYPE_DISCONNECT){
					MQTTSnDisconnect* msg = new MQTTSnDisconnect();
					msg->absorb(resp);
					clnode->setClientRecvMessage(msg);
					ev->setClientRecvEvent(clnode);

				}else if (resp->getMsgType() == MQTTSN_TYPE_REGISTER){
					MQTTSnRegister* msg = new MQTTSnRegister();
					msg->absorb(resp);
					clnode->setClientRecvMessage(msg);
					ev->setClientRecvEvent(clnode);

				}else if (resp->getMsgType() == MQTTSN_TYPE_REGACK){
					MQTTSnRegAck* msg = new MQTTSnRegAck();
					msg->absorb(resp);
					clnode->setClientRecvMessage(msg);
					ev->setClientRecvEvent(clnode);

				}else if (resp->getMsgType() == MQTTSN_TYPE_UNSUBSCRIBE){
					MQTTSnUnsubscribe* msg = new MQTTSnUnsubscribe();
					msg->absorb(resp);
					clnode->setClientRecvMessage(msg);
					ev->setClientRecvEvent(clnode);

				}else if (resp->getMsgType() == MQTTSN_TYPE_SUBSCRIBE){
					MQTTSnSubscribe* msg = new MQTTSnSubscribe();
					msg->absorb(resp);
					clnode->setClientRecvMessage(msg);
					ev->setClientRecvEvent(clnode);

				}else if (resp->getMsgType() == MQTTSN_TYPE_WILLTOPIC){
					MQTTSnWillTopic* msg = new MQTTSnWillTopic();
					msg->absorb(resp);
					clnode->setClientRecvMessage(msg);
					ev->setClientRecvEvent(clnode);

				}else if (resp->getMsgType() == MQTTSN_TYPE_WILLMSG){
					MQTTSnWillMsg* msg = new MQTTSnWillMsg();
					msg->absorb(resp);
					clnode->setClientRecvMessage(msg);
					ev->setClientRecvEvent(clnode);

				}else if(resp->getMsgType() == MQTTSN_TYPE_SEARCHGW){
					MQTTSnSearchGw* msg = new MQTTSnSearchGw();
					msg->absorb(resp);
					ev->setEvent(msg);
				}else{
					eventSetFlg = false;
				}
			}
			if(eventSetFlg){
				_res->getGatewayEventQue()->post(ev);
			}else{
				delete ev;
			}
		}
		delete resp;
	}
}





