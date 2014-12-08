/*
 * GatewayResourcesProvider.cpp
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
#include "GatewayResourcesProvider.h"
#include "GatewayDefines.h"
#include "lib/ProcessFramework.h"
#include "ErrorMessage.h"
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>

#ifdef RASPBERRY_LIGHT_INDICATOR
#include <wiringPi.h>
#endif

using namespace std;

extern Process* theProcess;
extern char* currentDateTime();
extern void setUint32(uint8_t* pos, uint32_t val);

extern "C"{
	int wiringPiSetup(void);
	void pinMode(int gpioNo, int mode);
	void digitalWrite(int gpioNo, int val);
}

GatewayResourcesProvider* theGatewayResources = 0;

/*=====================================
     Class GatewayResourcesProvider
 =====================================*/
GatewayResourcesProvider::GatewayResourcesProvider(): MultiTaskProcess(){
	theMultiTask = this;
	theProcess = this;
	resetRingBuffer();
	_lightIndicator.greenLight(false);
}

GatewayResourcesProvider::~GatewayResourcesProvider(){
	LOGWRITE("%s TomyGateway stop\n", currentDateTime());
	_lightIndicator.greenLight(false);
	_lightIndicator.redLightOff();
}

EventQue<Event>* GatewayResourcesProvider::getGatewayEventQue(){
	return &_gatewayEventQue;
}

EventQue<Event>* GatewayResourcesProvider::getClientSendQue(){
	return &_clientSendQue;
}

EventQue<Event>* GatewayResourcesProvider::getBrokerSendQue(){
	return &_brokerSendQue;
}

ClientList* GatewayResourcesProvider::getClientList(){
	return &_clientList;
}

Network* GatewayResourcesProvider::getNetwork(){
	return &_network;
}

LightIndicator* GatewayResourcesProvider::getLightIndicator(){
	return &_lightIndicator;
}

/*=====================================
        Class Client
 =====================================*/
ClientNode::ClientNode(){
	ClientNode(false);
}

ClientNode::ClientNode(bool secure){
	_msgId = 0;
	_snMsgId = 0;
	_status = Cstat_Disconnected;
	_keepAliveMsec = 0;
	_topics = new Topics();

	_address64 = NWAddress64();
	_nodeId = "";
	_address16 = 0;

	_mqttConnect = 0;

	_waitedPubAck = 0;
	_waitedSubAck = 0;
	if(secure){
		_stack = new TLSStack(true);
	}else{
		_stack = new TLSStack(false);
	}
	_connAckSaveFlg = false;
	_connAck = 0;
	_waitWillMsgFlg = false;
}

ClientNode::~ClientNode(){
	delete _topics;
	if(_mqttConnect){
		delete _mqttConnect;
	}
	if(_waitedPubAck){
		delete _waitedPubAck;
	}
	if(_waitedSubAck){
		delete _waitedSubAck;
	}
	if(_stack){
		delete _stack;
	}
}

void ClientNode::setWaitedPubAck(MQTTSnPubAck* msg){
	_waitedPubAck = msg;
}

void ClientNode::setWaitedSubAck(MQTTSnSubAck* msg){
	_waitedSubAck = msg;
}

MQTTSnPubAck* ClientNode::getWaitedPubAck(){
	return _waitedPubAck;
}

MQTTSnSubAck* ClientNode::getWaitedSubAck(){
	return _waitedSubAck;
}

uint16_t ClientNode::getNextMessageId(){
	_msgId++;
	if (_msgId == 0){
		_msgId++;
	}
	return _msgId;
}

uint8_t ClientNode::getNextSnMsgId(){
	_snMsgId++;
		if (_snMsgId == 0){
			_snMsgId++;
		}
		return _snMsgId;
}


MQTTMessage* ClientNode::getBrokerSendMessage(){
	return _brokerSendMessageQue.getMessage();
}

MQTTMessage* ClientNode::getBrokerRecvMessage(){
	return _brokerRecvMessageQue.getMessage();
}

MQTTSnMessage* ClientNode::getClientSendMessage(){
	return _clientSendMessageQue.getMessage();
}

MQTTSnMessage* ClientNode::getClientSleepMessage(){
	return _clientSleepMessageQue.getMessage();
}

MQTTSnMessage* ClientNode::getClientRecvMessage(){
	return _clientRecvMessageQue.getMessage();
}

MQTTConnect*   ClientNode::getConnectMessage(){
	return _mqttConnect;
}

TLSStack* ClientNode::getStack(){
	return _stack;
}


void ClientNode::setBrokerSendMessage(MQTTMessage* msg){
	_brokerSendMessageQue.push(msg);
}

void ClientNode::setBrokerRecvMessage(MQTTMessage* msg){
	_brokerRecvMessageQue.push(msg);
}

void ClientNode::setClientSendMessage(MQTTSnMessage* msg){
	_clientSendMessageQue.push(msg);

}

void ClientNode::setClientRecvMessage(MQTTSnMessage* msg){
	updateStatus(msg);
	_clientRecvMessageQue.push(msg);
}

void ClientNode::setClientSleepMessage(MQTTSnMessage* msg){
	updateStatus(msg);
	_clientSleepMessageQue.push(msg);
}

void ClientNode::setConnectMessage(MQTTConnect* msg){
	_mqttConnect = msg;
}


void ClientNode::checkTimeover(){
	if(_status == Cstat_Active && _keepAliveTimer.isTimeup()){
		_status = Cstat_Lost;
		_stack->disconnect();
	}
}

void ClientNode::setKeepAlive(MQTTSnMessage* msg){
	MQTTSnConnect* cm = static_cast<MQTTSnConnect*>(msg);
	_keepAliveMsec = cm->getDuration() * 1000UL;
	_keepAliveTimer.start(_keepAliveMsec * 1.5);
}

bool ClientNode::isConnectSendable(){
	if(_status == Cstat_Connecting || _status == Cstat_TryConnecting){
		return false;
	}else{
		return true;
	}
}

void ClientNode::updateStatus(ClientStatus stat){
	_status = stat;
}

void ClientNode::connectSended(){
	if(_status == Cstat_TryConnecting){
		_status = Cstat_Connecting;

		if(_mqttConnect){
			delete _mqttConnect;
		}
		_mqttConnect = 0;
	}
}

void ClientNode::connectQued(){
	if(_status == Cstat_Disconnected || _status == Cstat_Lost){
		_status = Cstat_TryConnecting;
	}
}

void ClientNode::disconnected(){
	_status = Cstat_Disconnected;
	_connAckSaveFlg = false;
	_waitWillMsgFlg = false;
}

bool ClientNode::isDisconnect(){
	return (_status == Cstat_Disconnected);
}

bool ClientNode::isActive(){
	return (_status == Cstat_Active);
}

bool ClientNode::isSleep(){
	return (_status == Cstat_Asleep);
}

void ClientNode::connackSended(int rc){
	if(_status == Cstat_Connecting){
		if(rc == MQTTSN_RC_ACCEPTED){
			_status = Cstat_Active;
		}else{
			disconnected();
		}
	}
}

void ClientNode::updateStatus(MQTTSnMessage* msg){
	if(((_status == Cstat_Disconnected) || (_status == Cstat_Lost)) && 
         msg->getType() == MQTTSN_TYPE_CONNECT){
		setKeepAlive(msg);
	}else if(_status == Cstat_Active){
		switch(msg->getType()){
		case MQTTSN_TYPE_PINGREQ:
		case MQTTSN_TYPE_PUBLISH:
		case MQTTSN_TYPE_SUBSCRIBE:
		case MQTTSN_TYPE_UNSUBSCRIBE:
		case MQTTSN_TYPE_PUBACK:
		case MQTTSN_TYPE_PUBCOMP:
		case MQTTSN_TYPE_PUBREL:
		case MQTTSN_TYPE_PUBREC:
			 _keepAliveTimer.start(_keepAliveMsec * 1.5);
			 break;
		case MQTTSN_TYPE_DISCONNECT:{
			MQTTSnDisconnect* dcm = static_cast<MQTTSnDisconnect*>(msg);
			if(dcm->getDuration()){
				_status = Cstat_Asleep;
				_keepAliveMsec = dcm->getDuration() * 1000UL;
			}else{
				disconnected();
			}
		}
			break;
		default:
			break;
		}
	
	}else if(_status == Cstat_Asleep){
		if(msg->getType() == MQTTSN_TYPE_CONNECT){
			setKeepAlive(msg);
			_status = Cstat_Connecting;
		}else if( msg->getType() == MQTTSN_TYPE_PINGREQ ){
			MQTTSnPingReq* pr = static_cast<MQTTSnPingReq*>(msg);
			if(pr->getClientId()) {
				_status = Cstat_Awake;
			}
		}
	}else if(_status == Cstat_Awake){
		switch(msg->getType()){
			case MQTTSN_TYPE_CONNECT:
				_status = Cstat_Connecting;
				setKeepAlive(msg);
				break;
			case MQTTSN_TYPE_DISCONNECT:
				disconnected();
				break;
			case MQTTSN_TYPE_PINGRESP:
				_status = Cstat_Asleep;
				break;
			default:
				break;
		}
	}
}

int  ClientNode::checkConnAck(MQTTSnConnack* msg){
	if(_connAckSaveFlg || _waitWillMsgFlg){
		_connAck = msg;
		return -1;
	}else{
		return 0;
	}
}

MQTTSnConnack*  ClientNode::checkGetConnAck(){
	MQTTSnConnack* connAck = 0;

	if(!_connAck){
		_waitWillMsgFlg = true;
	}else if(_connAckSaveFlg || _waitWillMsgFlg){
		_connAckSaveFlg = 0;
		_waitWillMsgFlg = false;
		connAck = _connAck;
		_connAck = 0;
		return connAck;
	}
	return 0;
}

void ClientNode::setConnAckSaveFlg(){
	_connAckSaveFlg = true;
	_connAck = 0;
}

void ClientNode::setWaitWillMsgFlg(){
	_waitWillMsgFlg = true;
}

void ClientNode::deleteBrokerSendMessage(){
	_brokerSendMessageQue.pop();
}

void ClientNode::deleteBrokerRecvMessage(){
	_brokerRecvMessageQue.pop();
}

void ClientNode::deleteClientSendMessage(){
	_clientSendMessageQue.pop();
}

void ClientNode::deleteClientRecvMessage(){
	_clientRecvMessageQue.pop();
}

Topics* ClientNode::getTopics(){
	return _topics;
}


NWAddress64* ClientNode::getAddress64Ptr(){
	return &_address64;
}

uint16_t ClientNode::getAddress16(){
    return _address16;
}

string* ClientNode::getNodeId(){
    return &_nodeId;
}

void ClientNode::setMsb(uint32_t msb){
    _address64.setMsb(msb);
}

void ClientNode::setLsb(uint32_t lsb){
    _address64.setLsb(lsb);
}

void ClientNode::setClientAddress64(NWAddress64* addr){
	 setMsb(addr->getMsb());
	 setLsb(addr->getLsb());
}

void ClientNode::setClientAddress16(uint16_t addr){
    _address16 = addr;
}

void ClientNode::setNodeId(string* id){
	_nodeId.assign(*id);
}

void ClientNode::setTopics(Topics* topics){
	_topics = topics;
}


/*=====================================
        Class ClientList
 =====================================*/
ClientList::ClientList(){
	_clientVector = new vector<ClientNode*>();
	_clientVector->reserve(MAX_CLIENT_NODES);
	_clientCnt = 0;
	_authorize = false;
}

ClientList::~ClientList(){
	_mutex.lock();
	vector<ClientNode*>::iterator client = _clientVector->begin();
	while((!_clientVector->empty()) && *client){
		delete *client;
		_clientVector->erase(client);
	}
	_mutex.unlock();
}

void ClientList::authorize(const char* fname, bool secure){
	FILE* fp;
	char buf[258];
	size_t pos;

	if((fp = fopen(fname, "r")) != 0){
		while(fgets(buf, 256, fp) != 0){
			string data = string(buf);
			while((pos = data.find_first_of(" 　\t\n")) != string::npos){
				data.erase(pos, 1);
			}
			if(data.empty()){
				continue;
			}
			pos = data.find_first_of(",");
			string addr = data.substr(0,pos);
			if(addr.size() == 16){
				unsigned long msb, lsb;
				char hex[9];
				strncpy(hex,addr.c_str(),8);
				msb = strtoul(hex,0,16);
				lsb = strtoul(addr.c_str() + 8,0,16);
				NWAddress64 addr64 = NWAddress64(msb, lsb);

				string id = data.substr(pos + 1);
				createNode(secure, &addr64,0,&id);
			}else{
				LOGWRITE("Invalid address     %s\n",data.c_str());
			}
		}
		fclose(fp);
		_authorize = true;
		LOGWRITE("Clients are authorized.\n");
	}
}

ClientNode* ClientList::createNode(bool secure, NWAddress64* addr64, uint16_t addr16, string* nodeId){
	if(_clientCnt < MAX_CLIENT_NODES && !_authorize){
		_mutex.lock();
		vector<ClientNode*>::iterator client = _clientVector->begin();
		while( client != _clientVector->end()){
			if(((*client)->getAddress64Ptr() == addr64) && ((*client)->getAddress16() == addr16)){
				return 0;
			}else{
				++client;
			}
		}
		ClientNode* node = new ClientNode(secure);
		node->setClientAddress64(addr64);
		node->setClientAddress16(addr16);
		if (nodeId){
			node->setNodeId(nodeId);
		}
		_clientVector->push_back(node);
		_clientCnt++;
		_mutex.unlock();
		return node;
	}else{
		return getClient(addr64, addr16);
	}
}

void ClientList::erase(ClientNode* clnode){

	uint16_t pos = 0;
	_mutex.lock();
	vector<ClientNode*>::iterator client = _clientVector->begin();

	while( (client != _clientVector->end()) && *client){

		if((*client) == clnode){
			delete(*client);
			_clientVector->erase(client);
			_clientCnt--;
			for(; pos < _clientCnt; pos++){
				_clientVector[pos] = _clientVector[pos + 1];
			}
		}else{
			++client;
			++pos;
		}
	}
	_mutex.unlock();
}

ClientNode* ClientList::getClient(NWAddress64* addr64, uint16_t addr16){
	_mutex.lock();
	vector<ClientNode*>::iterator client = _clientVector->begin();
	while( (client != _clientVector->end()) && *client){
			if(*((*client)->getAddress64Ptr()) == *addr64 &&
					(*client)->getAddress16() == addr16){
				_mutex.unlock();
				return *client;
			}else{
				++client;
			}
	}
	_mutex.unlock();
	return 0;
}

uint16_t ClientList::getClientCount(){
	return _clientCnt;
}

ClientNode* ClientList::operator[](int pos){
	_mutex.lock();
	ClientNode* node = (*_clientVector)[pos];
	_mutex.unlock();
	return node;
}

bool ClientList::isAuthorized(){
	return _authorize;
}

/*=====================================
        Class Event
 =====================================*/
Event::Event(){
	_eventType = Et_NA;
	_clientNode = 0;
	_mqttSnMessage = 0;
}

Event::Event(EventType type){
	_eventType = type;
	_clientNode = 0;
	_mqttSnMessage = 0;
}

Event::~Event(){
	switch(_eventType){
	case EtClientRecv:
		if(_clientNode){
			_clientNode->deleteClientRecvMessage();
		}
		break;
	case EtClientSend:
			if(_clientNode){
				_clientNode->deleteClientSendMessage();
			}
			break;
	case EtBrokerRecv:
			if(_clientNode){
				_clientNode->deleteBrokerRecvMessage();
			}
			break;
	case EtBrokerSend:
			if(_clientNode){
				_clientNode->deleteBrokerSendMessage();
			}
			break;
	case EtBroadcast:
		if(_mqttSnMessage){
			delete _mqttSnMessage;
		}
		break;
	default:     // Et_NA, EtTimeout
		break;
	}
}

EventType Event::getEventType(){
	return  _eventType;
}

void Event::setClientSendEvent(ClientNode* client){
	_clientNode = client;
	_eventType = EtClientSend;
}

void Event::setClientRecvEvent(ClientNode* client){
	_clientNode = client;
	_eventType = EtClientRecv;
}

void Event::setBrokerSendEvent(ClientNode* client){
	_clientNode = client;
	_eventType = EtBrokerSend;
}

void Event::setBrokerRecvEvent(ClientNode* client){
	_clientNode = client;
	_eventType = EtBrokerRecv;
}

void Event::setTimeout(){
	_eventType = EtTimeout;
}

void Event::setEvent(MQTTSnMessage* msg){
	_mqttSnMessage = msg;
	_eventType = EtBroadcast;
}


ClientNode* Event::getClientNode(){
	return _clientNode;
}


MQTTSnMessage* Event::getMqttSnMessage(){
	return _mqttSnMessage;
}

/*=====================================
        Class LightIndicator
 =====================================*/
LightIndicator::LightIndicator(){
	_gpioAvailable = false;
	init();
	_greenStatus = true;
	_blueStatus = true;
	greenLight(false);
	blueLight(false);
}

LightIndicator::~LightIndicator(){

}

void LightIndicator::init(){
#ifdef RASPBERRY_LIGHT_INDICATOR
	if(wiringPiSetup() != -1){
		pinMode(LIGHT_INDICATOR_GREEN, OUTPUT);
		pinMode(LIGHT_INDICATOR_RED, OUTPUT);
		pinMode(LIGHT_INDICATOR_BLUE, OUTPUT);
		_gpioAvailable = true;
	}
#endif
}

void LightIndicator::greenLight(bool on){
	if(on){
		if(_greenStatus == 0){
			_greenStatus = 1;
			//Turn Green on & turn Red off
			lit(LIGHT_INDICATOR_GREEN, 1);
			lit(LIGHT_INDICATOR_RED, 0);
		}
	}else{
		if(_greenStatus == 1){
			_greenStatus = 0;
			//Turn Green off & turn Red on
			lit(LIGHT_INDICATOR_GREEN, 0);
			lit(LIGHT_INDICATOR_RED, 1);
		}
	}
}

void LightIndicator::redLightOff(){
	lit(LIGHT_INDICATOR_RED, 0);
}

void LightIndicator::blueLight(bool on){
	if(on){
		if(_blueStatus == 0){
			_blueStatus = 1;
			lit(LIGHT_INDICATOR_BLUE, 1);
		}
	}else{
		if(_blueStatus == 1){
			_blueStatus = 0;
			lit(LIGHT_INDICATOR_BLUE, 0);
		}
	}
}

void LightIndicator::lit(int gpioNo, int onoff){
#ifdef RASPBERRY_LIGHT_INDICATOR
	if(_gpioAvailable){
		digitalWrite(gpioNo,onoff);
	}
#endif
}
