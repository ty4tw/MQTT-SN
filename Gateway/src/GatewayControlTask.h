/*
 * GatewayControlTask.h
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

#ifndef GATEWAYCONTROLTASK_H_
#define GATEWAYCONTROLTASK_H_

#include "lib/ZBStack.h"
#include "lib/ProcessFramework.h"
#include "GatewayResourcesProvider.h"

/*=====================================
        Class GatewayControlTask
 =====================================*/
class GatewayControlTask : public Thread{
	MAGIC_WORD_FOR_TASK;
public:
	GatewayControlTask(GatewayResourcesProvider* res);
	~GatewayControlTask();

	void run();
private:
	EventQue<Event>* _eventQue;
	GatewayResourcesProvider* _res;
	char _printBuf[512];
	uint8_t _protocol;
	uint8_t _gatewayId;
	string _loginId;
	string _password;
	bool _secure;

	void handleClientMessage(Event*);
	void handleBrokerMessage(Event*);

	void handleSnPublish(Event* ev, ClientNode* clnode, MQTTSnMessage* msg);
	void handleSnSubscribe(Event* ev, ClientNode* clnode, MQTTSnMessage* msg);
	void handleSnUnsubscribe(Event* ev, ClientNode* clnode, MQTTSnMessage* msg);
	void handleSnPingReq(Event* ev, ClientNode* clnode, MQTTSnMessage* msg);
	void handleSnPubAck(Event* ev, ClientNode* clnode, MQTTSnMessage* msg);
	void handleSnConnect(Event* ev, ClientNode* clnode, MQTTSnMessage* msg);
	void handleSnWillTopic(Event* ev, ClientNode* clnode, MQTTSnMessage* msg);
	void handleSnWillMsg(Event* ev, ClientNode* clnode, MQTTSnMessage* msg);
	void handleSnDisconnect(Event* ev, ClientNode* clnode, MQTTSnMessage* msg);
	void handleSnRegister(Event* ev, ClientNode* clnode, MQTTSnMessage* msg);
	void handleSnPubRec(Event* ev, ClientNode* clnode, MQTTSnMessage* msg);
	void handleSnPubRel(Event* ev, ClientNode* clnode, MQTTSnMessage* msg);
	void handleSnPubComp(Event* ev, ClientNode* clnode, MQTTSnMessage* msg);

	void handlePuback(Event* ev, ClientNode* clnode, MQTTMessage* msg);
	void handlePingresp(Event* ev, ClientNode* clnode, MQTTMessage* msg);
	void handleSuback(Event* ev, ClientNode* clnode, MQTTMessage* msg);
	void handleUnsuback(Event* ev, ClientNode* clnode, MQTTMessage* msg);
	void handleConnack(Event* ev, ClientNode* clnode, MQTTMessage* msg);
	void handlePublish(Event* ev, ClientNode* clnode, MQTTMessage* msg);
	void handleDisconnect(Event* ev, ClientNode* clnode, MQTTMessage* msg);
	void handlePubRec(Event* ev, ClientNode* clnode, MQTTMessage* msg);
	void handlePubRel(Event* ev, ClientNode* clnode, MQTTMessage* msg);
	void handlePubComp(Event* ev, ClientNode* clnode, MQTTMessage* msg);
	char* msgPrint(MQTTSnMessage* msg);
	char* msgPrint(MQTTMessage* msg);
};

#endif /* GATEWAYCONTROLTASK_H_ */
