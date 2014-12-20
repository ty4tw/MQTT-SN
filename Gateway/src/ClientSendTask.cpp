/*
 * ClientSendTask.cpp
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
#include "ClientSendTask.h"
#include "GatewayResourcesProvider.h"
#include "lib/ProcessFramework.h"
#include "lib/Messages.h"
#include "ErrorMessage.h"

#include <unistd.h>
#include <iostream>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>


ClientSendTask::ClientSendTask(GatewayResourcesProvider* res){
	_res = res;
	_res->attach(this);
}

ClientSendTask::~ClientSendTask(){

}


void ClientSendTask::run(){
	NETWORK_CONFIG config;
#ifdef NETWORK_XBEE

	char param[TOMYFRAME_PARAM_MAX];
	bool secure = true;

	if(_res->getParam("BaudRate",param) == 0){

		int val = atoi(param);
		switch(val){
		case 9600:
			config.baudrate = B9600;
			break;
		case 19200:
			config.baudrate =B19200;
			break;
		case 38400:
			config.baudrate =B38400;
			break;
		case 57600:
			config.baudrate =B57600;
			break;
		case 115200:
			config.baudrate = B115200;
			break;
		default:
			THROW_EXCEPTION(ExFatal, ERRNO_APL_01, "Invalid baud rate!");  // ABORT
		}
	}else{
		config.baudrate = B57600;
	}
	config.flag = O_WRONLY;
	if(_res->getParam("SerialDevice", param) == 0){
		config.device = strdup(param);
	}
	if(_res->getParam("SecureConnection",param) == 0){
		if(strcmp(param, "YES")){
			secure = false;  // TCP
		}
	}
	_res->getClientList()->authorize(FILE_NAME_CLIENT_LIST, secure);
	_network = new Network();

	if(_network->initialize(config) < 0){
		THROW_EXCEPTION(ExFatal, ERRNO_APL_01, "can't open the client port.");  // ABORT
	}

#endif

#ifdef NETWORK_UDP
	_network = _res->getNetwork();

#endif

#ifdef NETWORK_XXXXX
	_network = _res->getNetwork();
#endif


	while(true){
		Event* ev = _res->getClientSendQue()->wait();

		if(ev->getEventType() == EtClientSend){
			MQTTSnMessage msg = MQTTSnMessage();
			ClientNode* clnode = ev->getClientNode();
			msg.absorb( clnode->getClientSendMessage() );

			_network->unicast(clnode->getAddress64Ptr(), clnode->getAddress16(),
					msg.getMessagePtr(), msg.getMessageLength());
		}else if(ev->getEventType() == EtBroadcast){
			MQTTSnMessage msg = MQTTSnMessage();
			msg.absorb( ev->getMqttSnMessage() );
			_network->broadcast(msg.getMessagePtr(), msg.getMessageLength());
		}
		delete ev;
	}
}


