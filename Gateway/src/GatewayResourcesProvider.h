/*
 * GatewayResourcesProvider.h
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

#ifndef GATEWAY_RESOURCES_PROVIDER_H_
#define GATEWAY_RESOURCES_PROVIDER_H_

#include "lib/ProcessFramework.h"
#include "lib/Messages.h"
#include "lib/Topics.h"
#include "lib/TLSStack.h"

#define FILE_NAME_CLIENT_LIST "/usr/local/etc/tomygateway/config/clientList.conf"
#define GATEWAY_VERSION "(Ver 1.0.1)"

/*=====================================
        Class MessageQue
 =====================================*/
template<class T> class MessageQue{
public:
	MessageQue();
	~MessageQue();
	T* getMessage();
	void push(T*);
	void pop();
	bool empty();
private:
	queue<T*> _que;
	Mutex  _mutex;
};


enum ClientStatus {
	Cstat_Disconnected = 0,
	Cstat_TryConnecting,
	Cstat_Connecting,
	Cstat_Active,
	Cstat_Asleep,
	Cstat_Awake,
	Cstat_Lost
};

/*=====================================
        Class ClientNode
 =====================================*/
class ClientNode{
public:
	ClientNode();
	ClientNode(bool secure);
	~ClientNode();

	MQTTMessage*   getBrokerSendMessage();
	MQTTMessage*   getBrokerRecvMessage();
	MQTTSnMessage* getClientSendMessage();
	MQTTSnMessage* getClientRecvMessage();
	MQTTConnect*   getConnectMessage();
	MQTTSnPubAck*  getWaitedPubAck();
	MQTTSnSubAck*  getWaitedSubAck();
	MQTTSnMessage* getClientSleepMessage();

	void setBrokerSendMessage(MQTTMessage*);
	void setBrokerRecvMessage(MQTTMessage*);
	void setClientSendMessage(MQTTSnMessage*);
	void setClientRecvMessage(MQTTSnMessage*);
	void setConnectMessage(MQTTConnect*);
	void setWaitedPubAck(MQTTSnPubAck* msg);
	void setWaitedSubAck(MQTTSnSubAck* msg);
	void setClientSleepMessage(MQTTSnMessage*);

	void deleteBrokerSendMessage();
	void deleteBrokerRecvMessage();
	void deleteClientSendMessage();
	void deleteClientRecvMessage();

	void checkTimeover();
	void updateStatus(MQTTSnMessage*);
	void updateStatus(ClientStatus);
	void connectSended();
	void connackSended(int rc);
	void connectQued();
	void disconnected();
	bool isConnectSendable();
	uint16_t getNextMessageId();
	uint8_t getNextSnMsgId();
	Topics* getTopics();

	TLSStack* getStack();
	NWAddress64* getAddress64Ptr();
	uint16_t  getAddress16();
	string* getNodeId();
	void setMsb(uint32_t);
	void setLsb(uint32_t);
	void setClientAddress16(uint16_t addr);
	void setClientAddress64(NWAddress64* addr);
	void setTopics(Topics* topics);
	void setNodeId(string* id);
	int  checkConnAck(MQTTSnConnack* msg);
	MQTTSnConnack*  checkGetConnAck();
	void setConnAckSaveFlg();
	void setWaitWillMsgFlg();
	bool isDisconnect();
	bool isActive();
	bool isSleep();

private:
	void setKeepAlive(MQTTSnMessage* msg);

	MessageQue<MQTTMessage>   _brokerSendMessageQue;
	MessageQue<MQTTMessage>   _brokerRecvMessageQue;
	MessageQue<MQTTSnMessage> _clientSendMessageQue;
	MessageQue<MQTTSnMessage> _clientRecvMessageQue;
	MessageQue<MQTTSnMessage> _clientSleepMessageQue;

	MQTTConnect*   _mqttConnect;

	MQTTSnPubAck*  _waitedPubAck;
	MQTTSnSubAck*  _waitedSubAck;

	uint16_t _msgId;
	uint8_t _snMsgId;
	Topics* _topics;
	ClientStatus _status;
	uint32_t _keepAliveMsec;
	Timer _keepAliveTimer;

	TLSStack* _stack;

	NWAddress64 _address64;
    uint16_t _address16;
    string _nodeId;
    bool _connAckSaveFlg;
    bool _waitWillMsgFlg;
    MQTTSnConnack*  _connAck;

};

/*=====================================
        Class ClientList
 =====================================*/
class ClientList{
public:
	ClientList();
	~ClientList();
	void authorize(const char* fileName, bool secure);
	void erase(ClientNode*);
	ClientNode* getClient(NWAddress64* addr64, uint16_t addr16);
	ClientNode* createNode(bool secure, NWAddress64* addr64, uint16_t addr16, string* nodeId = 0);
	uint16_t getClientCount();
	ClientNode* operator[](int);
private:
	vector<ClientNode*>*  _clientVector;
	Mutex _mutex;
	uint16_t _clientCnt;
	bool _authorize;
};

/*=====================================
         Class Event
  ====================================*/
enum EventType{
	Et_NA = 0,
	EtTimeout,

	EtBrokerSend,
	EtBrokerRecv,
	EtClientSend,
	EtClientRecv,
	EtBroadcast,
	EtSocketAlive
};

class Event{
public:
	Event();
	Event(EventType);
	~Event();
	EventType getEventType();
	void setClientSendEvent(ClientNode*);
	void setBrokerSendEvent(ClientNode*);
	void setClientRecvEvent(ClientNode*);
	void setBrokerRecvEvent(ClientNode*);
	void setEvent(MQTTSnMessage*);
	void setTimeout();
	ClientNode* getClientNode();
	MQTTSnMessage* getMqttSnMessage();
private:
	EventType   _eventType;
	ClientNode* _clientNode;
	MQTTSnMessage* _mqttSnMessage;
};

/*=====================================
     Class LightIndicator
 =====================================*/
class LightIndicator{
public:
	LightIndicator();
	~LightIndicator();
	void greenLight(bool);
	void blueLight(bool);
	void redLightOff();
private:
	void init();
	void lit(int gpioNo, int onoff);
	bool _greenStatus;
	bool _blueStatus;
	bool _gpioAvailable;
};

/*=====================================
     Class GatewayResourcesProvider
 =====================================*/
class GatewayResourcesProvider: public MultiTaskProcess{
public:
	GatewayResourcesProvider();
	~GatewayResourcesProvider();

	EventQue<Event>* getGatewayEventQue();
	EventQue<Event>* getClientSendQue();
	EventQue<Event>* getBrokerSendQue();
	ClientList* getClientList();
	Network* getNetwork();
	LightIndicator* getLightIndicator();
private:
	ClientList _clientList;
	EventQue<Event> _gatewayEventQue;
	EventQue<Event> _brokerSendQue;
	EventQue<Event> _clientSendQue;
	Network _network;
	LightIndicator _lightIndicator;
};



/*=====================================
    Class MessageQue Implimentation
 =====================================*/
template<class T> MessageQue<T>::MessageQue(){

}

template<class T> MessageQue<T>::~MessageQue(){
	_mutex.lock();
	while(!_que.empty()){
		delete _que.front();
		_que.pop();
	}
	_mutex.unlock();
}

template<class T> T* MessageQue<T>::getMessage(){
	T* msg;
	if(!_que.empty()){
		_mutex.lock();
		msg = _que.front();
		_mutex.unlock();
		return msg;
	}else{
		return 0;
	}
}

template<class T> void MessageQue<T>::push(T* msg){
	_mutex.lock();
	_que.push(msg);
	_mutex.unlock();
}

template<class T> void MessageQue<T>::pop(){
	if(!_que.empty()){
		_mutex.lock();
		delete _que.front();
		_que.pop();
		_mutex.unlock();
	}
}

#endif /* GATEWAY_RESOURCES_PROVIDER_H_ */
