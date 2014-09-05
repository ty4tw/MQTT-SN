/*
 * mqttsnClient.h
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

#ifndef MQTTSCLIENT_H_
#define MQTTSCLIENT_H_

#ifdef LINUX
		#define SENDQ_SIZE   20
#else
		#define SENDQ_SIZE    5
#endif

#if defined(ARDUINO) && ARDUINO >= 100
        #include "Arduino.h"
        #include <inttypes.h>
		#include <MQTTSN_Application.h>
		#include <Network.h>
        #include <mqttsn.h>
		#include <mqUtil.h>
#else
        #if defined(ARDUINO) && ARDUINO < 100
                #include "WProgram.h"
                #include <inttypes.h>
                #include <mqttsn.h>
				#include <Network.h>
				#include <mqUtil.h>
        #else
                #ifdef LINUX
                    #include <sys/time.h>
                #endif
                #include <iostream>
				#include "MQTTSN_Application.h"
				#include "Network.h"
				#include "mqUtil.h"
                #include "mqttsn.h"
        #endif
#endif


#define GW_LOST      0
#define GW_SEARCHING 1
#define GW_FIND      2

#define CL_DISCONNECTED  0
#define CL_ACTIVE        1
#define CL_ASLEEP        2
#define CL_AWAKE         3


using namespace tomyClient;

typedef struct {
	int (*callback)(void);
	uint32_t sec;
}TaskList;

typedef struct {
	MQString* topic;
	int (*pubCallback)(MqttsnPublish*);
	uint8_t qos;
}OnPublishList;

typedef struct {
	uint32_t prevTime;
	uint32_t interval;
	int (*callback)(void);
}MQ_TimerTbl;

/*=====================================
        Class ClientStatus
 ======================================*/
class ClientStatus{
public:
	ClientStatus();
	~ClientStatus();

	bool isLost();
	bool isSearching();
	bool isConnected();
	bool isOnConnected();
	bool isSubscribing();
	bool isAvailableToSend();
	bool isPINGREQRequired();
	bool isGatewayAlive();

	uint16_t getKeepAlive();
	void setKeepAlive(uint16_t sec);
	void sendSEARCHGW();
	bool recvGWINFO(MqttsnGwInfo* gwi);
	void recvADVERTISE(MqttsnAdvertise* adv);
	void recvCONNACK();
	void recvDISCONNECT();
	void recvPINGRESP();
	void setAvailableToSend();
	void setSubscribing(bool);
	void setLastSendTime();
	void setModeSleep();
	void init();


private:
	void changeUTC();

	uint8_t _gwId;
	uint8_t _gwStat;
	uint8_t _clStat;
	bool   _onConnectFlg;
	bool   _subscribingFlg;
	uint16_t _keepAliveDuration; // PINGREQ interval
	uint16_t _advertiseDuration; // Gateway heart beat
	XTimer  _keepAliveTimer;
	XTimer  _advertiseTimer;
	bool _sleepModeFlg;
};

/*=====================================
        Class SendQue  (FIFO)
 ======================================*/
class SendQue {
public:
    SendQue();
    ~SendQue();
    int addRequest(MqttsnMessage* msg);
    int addPriorityRequest(MqttsnMessage* msg);
    void setStatus(uint8_t index, uint8_t status);
    MqttsnMessage* getMessage(uint8_t index);
    int  getStatus(uint8_t index);
    uint8_t getCount();
    int deleteRequest(uint8_t index);
    void   deleteAllRequest();
    void setQueSize(uint8_t sz);
private:
    uint8_t   _queSize;
    uint8_t   _queCnt;
    MqttsnMessage*  _msg[SENDQ_SIZE];
};


/*=====================================
        Class MqttsnClient
 ======================================*/
class MqttsnClient {
public:
    MqttsnClient();
    ~MqttsnClient();

    Topics* getTopics();

    int initialize(APP_CONFIG config);
    void subscribe();
    void setSubscribing(bool);
    void setKeepAlive(uint16_t sec);
    void setWillTopic(MQString* topic);
    void setWillMessage(MQString* msg);
    void setSleepMode(void);
    void setRetain(bool retain);
    void setClean(bool clean);
    void setRetryMax(uint8_t cnt);
    void setGwAddress();
    MQString* getClientId();
    ClientStatus* getClientStatus();
    bool isCleanSession();


    int  publish(MQString* topic, const char* data, int dataLength, uint8_t qos = 1);
    int  publish(MQString* topic, MQString* data, uint8_t qos = 1);
    int  publish(uint16_t predifinedId,  const char* data, int dataLength, uint8_t qos = 1);
    int  publish(MQString* topic, Payload* payload, uint8_t qos = 1);
    int  registerTopic(MQString* topic);
    int  subscribe(MQString* topic, TopicCallback callback, uint8_t qos = 1);
    int  subscribe(uint16_t predefinedId, TopicCallback callback, uint8_t qos);
    int  unsubscribe(MQString* topic);
    int  unsubscribe(uint16_t predefinedId);
    int  disconnect(uint16_t duration = 0);
    void createTopics();

    void recieveMessageHandler(NWResponse* msg, int* returnCode);
    void publishHdl(MqttsnPublish* msg);
    void recvMsg(uint16_t msec);
    int  exec();
    int readPacket();
    uint8_t getMsgRequestCount();

private:
    int  sendRecvMsg();
    void clearMsgRequest();
    int  requestSendMsg(MqttsnMessage* msg);
    int  requestPrioritySendMsg(MqttsnMessage* mqttsMsgPtr);
    int  broadcast(uint16_t packetReadTimeout);
    int  unicast(uint16_t packetReadTimeout);

    int  searchGw(uint8_t radius);
    int  connect();
    int  pingReq(MQString* clietnId);
    int  willTopic();
    int  willMsg();
    int  pubAck(uint16_t topicId, uint16_t msgId, uint8_t rc);
    int  pubRec(uint16_t msgId);
    int  pubRel(uint16_t msgId);
    int  regAck(uint16_t topicId, uint16_t msgId, uint8_t rc);
    int  pubComp(uint16_t msgId);

    uint8_t getMsgRequestType();
    uint8_t getMsgRequestStatus();
    void   setMsgRequestStatus(uint8_t stat);
    void createTopic(MQString* topic, TopicCallback callback);

    void delayTime(uint16_t baseTime);
    void copyMsg(MqttsnMessage* msg, NWResponse* recvMsg);
    uint16_t getNextMsgId();

    Network*         _network;
    Topics           _topics;
    SendQue*         _sendQ;
    XTimer           _respTimer;
    PublishHandller  _pubHdl;

    uint16_t         _duration;
    MQString*        _clientId;
    uint8_t          _clientFlg;
    uint8_t          _nRetry;
    uint8_t          _nRetryCnt;
    uint16_t         _tRetry;
    MQString*         _willTopic;
    MQString*         _willMessage;
    uint16_t         _msgId;
    ClientStatus     _clientStatus;
    bool             _sendFlg;
    bool             _subscribingFlg;
};



#endif /* MQTTSCLIENT_H_ */
