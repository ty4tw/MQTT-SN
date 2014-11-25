/*
 * mqttsnClient.cpp
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
 *    Modified: 2014/11/25
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 1.1.1
 */

#ifdef ARDUINO

#endif  /* ARDUINO */

#ifdef ARDUINO
	#include <MQTTSN_Application.h>
	#include <mqUtil.h>
	#include <mqttsnClient.h>
	#if defined(MQTTSN_DEBUG) || defined(NW_DEBUG) || defined(TY_DEBUG)
		#include <SoftwareSerial.h>
		extern SoftwareSerial debug;
	#endif
#else
	#include "MQTTSN_Application.h"
	#include "mqUtil.h"
	#include "mqttsnClient.h"
#endif

#ifdef MBED
  #include "mbed.h"
#endif  /* MBED */

#ifdef LINUX
  #include <stdio.h>
  #include <sys/time.h>
  #include <sys/types.h>
  #include <sys/stat.h>
  #include <unistd.h>
  #include <stdlib.h>
  #include <string.h>
  #include <fcntl.h>
  #include <errno.h>
  #include <termios.h>
#endif /* LINUX */


using namespace std;
using namespace tomyClient;

extern OnPublishList theOnPublishList[];
extern APP_CONFIG theAppConfig;
extern MQString* theTopics[];

extern uint16_t getUint16(uint8_t* pos);
extern uint32_t getUint32(uint8_t* pos);
extern float    getFloat32(uint8_t* pos);

extern void setUint16(uint8_t* pos, uint16_t val);
extern void setUint32(uint8_t* pos, uint32_t val);
extern void setFloat32(uint8_t* pos, float val);

static MqttsnClient*  theMqttsn;

/*=================================================================

        Class MqttsnnClient

 ================================================================*/
void ResponseHandler(NWResponse* resp, int* returnCode){
        theMqttsn->recieveMessageHandler(resp, returnCode);
}

MqttsnClient::MqttsnClient(){
    _network = new Network();
    _network->setRxHandler(ResponseHandler);
    _sendQ = new SendQue();
    _duration = 0;
    _clientId = new MQString();
    _clientFlg = 0;
    _nRetry = MQTTSN_RETRY_COUNT;
    _nRetryCnt = 0;
    _tRetry = 0;
    _willTopic = _willMessage = 0;
    _clientStatus.setKeepAlive(MQTTSN_DEFAULT_KEEPALIVE);
    _msgId = 0;
    _topics.allocate(MQTTSN_MAX_TOPICS + 1);
    _sendFlg = false;
    _subscribingFlg = false;
    theMqttsn = this;
}

MqttsnClient::~MqttsnClient(){
  _sendQ->deleteAllRequest();
  delete _network;
}


int MqttsnClient::initialize(APP_CONFIG config){
    _clientId->copy(config.mqttsnCfg.nodeId);
    MQString* pre1 = new MQString(MQTTSN_TOPIC_PREDEFINED_TIME);
    _topics.addTopic(pre1);
    _topics.setTopicId(pre1,MQTTSN_TOPICID_PREDEFINED_TIME);
    return _network->initialize(config.netCfg);
}

void MqttsnClient::subscribe(){
	for(int i = 0; theOnPublishList[i].pubCallback; i++){
		subscribe(theOnPublishList[i].topic, theOnPublishList[i].pubCallback, theOnPublishList[i].qos);
	}
}

void MqttsnClient::createTopics(){
	for(int i = 0; theTopics[i]; i++){
		registerTopic(theTopics[i]);
	}
}

void MqttsnClient::setSubscribing(bool flg){
	_clientStatus.setSubscribing(flg);
}

int MqttsnClient::readPacket(){
	return _network->readPacket(1); // XBee: Modem Staus, UDP: neglect
}

Topics* MqttsnClient::getTopics(){
    return &_topics;
}

void MqttsnClient::setKeepAlive(uint16_t sec){
	_clientStatus.setKeepAlive(sec);
}

void MqttsnClient::setWillTopic(MQString* topic){
    _willTopic = topic;
    _clientFlg |= MQTTSN_FLAG_WILL;
}

void MqttsnClient::setWillMessage(MQString* msg){
    _willMessage = msg;
    _clientFlg |= MQTTSN_FLAG_WILL;
}


void MqttsnClient::setRetain(bool retain){
	if(retain){
		_clientFlg |= MQTTSN_FLAG_RETAIN;
	}else{
		_clientFlg &= (~MQTTSN_FLAG_RETAIN);
	}
}

void MqttsnClient::setSleepMode(){
	_clientStatus.setModeSleep();
}

void MqttsnClient::setClean(bool clean){
	if(clean){
		_clientFlg |= MQTTSN_FLAG_CLEAN;
	}else{
		_clientFlg ^= MQTTSN_FLAG_CLEAN;
	}
}

bool MqttsnClient::isCleanSession(){
	return _clientFlg && MQTTSN_FLAG_CLEAN;
}

void MqttsnClient::setRetryMax(uint8_t cnt){
    _nRetry = cnt;
}


void MqttsnClient::setGwAddress(){
	_network->setGwAddress();
}

MQString* MqttsnClient::getClientId(){
    return _clientId;
}

ClientStatus* MqttsnClient::getClientStatus(){
	return &_clientStatus;
}


uint16_t MqttsnClient::getNextMsgId(){
    _msgId++;
    if (_msgId == 0){
        _msgId = 1;
    }
    return _msgId;
}

uint8_t MqttsnClient::getMsgRequestType(){
    if (_sendQ->getMessage(0)){
        return _sendQ->getMessage(0)->getType();
    }else{
        return 0;
    }
}
uint8_t MqttsnClient::getMsgRequestStatus(){
    return _sendQ->getStatus(0);
}

uint8_t MqttsnClient::getMsgRequestCount(){
  return _sendQ->getCount();
}

void MqttsnClient::setMsgRequestStatus(uint8_t stat){
    _sendQ->setStatus(0,stat);
}

void MqttsnClient::clearMsgRequest(){
    _sendQ->deleteRequest(0);
}

void MqttsnClient::createTopic(MQString* topic, TopicCallback callback){
    _topics.addTopic(topic);
    _topics.setCallback(topic, callback);
}

void MqttsnClient::delayTime(uint16_t maxTime){
#ifdef ARDUINO
    srand((uint32_t)millis( ));
    uint32_t tm = rand() % (maxTime * 1000);
#else
    srand((uint32_t)time( 0 ));
    uint32_t tm = (rand() % (maxTime * 1000));
#endif
    XTimer delayTimer;
    delayTimer.start(tm);
    while(!delayTimer.isTimeUp()){
       _network->readPacket();
    }
}

void MqttsnClient::copyMsg(MqttsnMessage* msg, NWResponse* recvMsg){
    memcpy(msg->getMsgBuff(), recvMsg->getPayload(), recvMsg->getPayload(0));
}



/*========================================================
    Send a MQTT-S Message (add the send request)
==========================================================*/
int MqttsnClient::requestSendMsg(MqttsnMessage* mqttsMsgPtr){
    int index = _sendQ->addRequest((MqttsnMessage*)mqttsMsgPtr);
	_sendQ->setStatus(index, MQTTSN_MSG_REQUEST);
    return MQTTSN_ERR_NO_ERROR;
}

/*========================================================
  Send a MQTT-S Message (add to the top of the send request)
==========================================================*/
int MqttsnClient::requestPrioritySendMsg(MqttsnMessage* mqttsMsgPtr){
    _sendQ->addPriorityRequest((MqttsnMessage*)mqttsMsgPtr);
    _sendQ->setStatus(0, MQTTSN_MSG_REQUEST);
    return MQTTSN_ERR_NO_ERROR;
}

/*========================================================
  Execute sending a MQTT-S Message
==========================================================*/

/*-------------  send Message once -----------------*/
int MqttsnClient::exec(){
    int rc;

    if (!_clientStatus.isGatewayAlive()){
		D_MQTTW("Gateway is Dead.\r\n");
		_clientStatus.init();
	}

    if(_sendFlg){
    	return MQTTSN_ERR_NO_ERROR;
    }else{
    	_sendFlg = true;
    }

    while(true){
        rc = sendRecvMsg();

        if (rc == MQTTSN_ERR_RETRY_OVER){
			if (getMsgRequestType() == MQTTSN_TYPE_WILLTOPIC    ||
			    getMsgRequestType() == MQTTSN_TYPE_WILLMSG      ||
			  	getMsgRequestType() == MQTTSN_TYPE_PINGREQ      ||
				getMsgRequestType() == MQTTSN_TYPE_PUBLISH      ||
				getMsgRequestType() == MQTTSN_TYPE_REGISTER     ||
				getMsgRequestType() == MQTTSN_TYPE_SUBSCRIBE    ||
				getMsgRequestType() == MQTTSN_TYPE_CONNECT      ||
				getMsgRequestType() == MQTTSN_TYPE_UNSUBSCRIBE  ||
				getMsgRequestType() == MQTTSN_TYPE_PUBREC       ||
				getMsgRequestType() == MQTTSN_TYPE_SEARCHGW     ||
				getMsgRequestType() == MQTTSN_TYPE_PUBREL) {
				_clientStatus.init();
				//clearMsgRequest();
			}
        }else if(rc == MQTTSN_ERR_REBOOT_REQUIRED){
			_clientStatus.init();
			//clearMsgRequest();
		}else if(rc != MQTTSN_ERR_NO_ERROR && rc != MQTTSN_ERR_INVALID_TOPICID){
			continue;
		}
		break;
	}
    clearMsgRequest();
    _sendFlg = false;
    return rc;
}

/*=============================
 *   Send or Receive Message
 ==============================*/
int MqttsnClient::sendRecvMsg(){
    int rc = MQTTSN_ERR_NO_ERROR;

    if (!_clientStatus.isGatewayAlive()){
		D_MQTTW("Gateway is Dead.\r\n");
		_clientStatus.init();
	}

	/*======= Establish Connection ===========*/
	if (_clientStatus.isLost() ||_clientStatus.isSearching() ){
		/*------------ Send SEARCHGW --------------*/
		if (getMsgRequestType() != MQTTSN_TYPE_SEARCHGW){
			searchGw(0);   //ZB_BROADCAST_RADIUS_MAX_HOPS
		}
		_clientStatus.sendSEARCHGW();
		_network->resetGwAddress();
		rc = broadcast(MQTTSN_TIME_RETRY);
		if ( rc != MQTTSN_ERR_NO_ERROR){
			return rc;
		}
	}

	if (!_clientStatus.isConnected() && !_clientStatus.isSearching()){
		/*-----------  Send CONNECT ----------*/
		if (getMsgRequestType() != MQTTSN_TYPE_CONNECT){
			connect();
		}
		rc = unicast(MQTTSN_TIME_RETRY);
		if ( rc != MQTTSN_ERR_NO_ERROR){
			return rc;
		}
	}

	if(_clientStatus.isOnConnected()){
		/*----------- ready to send  ----------*/
		_clientStatus.setAvailableToSend();
		/*----------- in case of clean session ----------*/
		if(isCleanSession() && !_clientStatus.isSubscribing()){
			_clientStatus.setSubscribing(true);  // re-entrant control
			subscribe();
			_clientStatus.setSubscribing(false); // re-entrant control
		}
	}

	if (getMsgRequestStatus() == MQTTSN_MSG_REQUEST || getMsgRequestStatus() == MQTTSN_MSG_RESEND_REQ){
        /*======  Send Message =======*/
        if (_clientStatus.isAvailableToSend()){
			rc = unicast(MQTTSN_TIME_RETRY);
		}else{
			rc = MQTTSN_ERR_NOT_CONNECTED;
		}
    }

	/*======= Receive Message ===========*/
	_network->readPacket();  //  Receive MQTT-S Message

	if (_clientStatus.isPINGREQRequired()){
		/*-------- Send PINGREQ -----------*/
		if(getMsgRequestType() != MQTTSN_TYPE_PINGREQ){
			pingReq(_clientId);
		}
		rc = unicast(MQTTSN_TIME_RETRY);
	}
	return rc;
}

/*------------------------------------
 *   Broad cast the MQTT-S Message
 -------------------------------------*/
int MqttsnClient::broadcast(uint16_t packetReadTimeout){
    int retry = 0;
    while(retry < _nRetry){
        _network->send(_sendQ->getMessage(0)->getMsgBuff(), _sendQ->getMessage(0)->getLength(),BcastReq);

        D_MQTTW("Bcast ");
		D_MQTTLN(_sendQ->getMessage(0)->getMsgTypeName());
		D_MQTTF("%s\r\n", _sendQ->getMessage(0)->getMsgTypeName());

        _respTimer.start(packetReadTimeout * 1000);
        setMsgRequestStatus(MQTTSN_MSG_WAIT_ACK);

        while(!_respTimer.isTimeUp()){
        	if(_network->readPacket() != PACKET_ERROR_NODATA){
				if (getMsgRequestStatus() == MQTTSN_MSG_COMPLETE){
					//clearMsgRequest();
					return MQTTSN_ERR_NO_ERROR;
				}else if(getMsgRequestStatus() == MQTTSN_MSG_REJECTED){
					//clearMsgRequest();
					return MQTTSN_ERR_REBOOT_REQUIRED;
				}
        	}
        }
        setMsgRequestStatus(MQTTSN_MSG_REQUEST);
        retry++;
    }
    //clearMsgRequest();
    return MQTTSN_ERR_RETRY_OVER;
}

/*------------------------------------
 *   Unicast the MQTT-S Message
 -------------------------------------*/
int MqttsnClient::unicast(uint16_t packetReadTimeout){
    int retry = 0;

    while(retry < _nRetry){
    	if (getMsgRequestStatus() == MQTTSN_MSG_RESEND_REQ){
			/* ------  Re send Time delay -------*/
			#ifdef ARDUINO
			  delay(MQTTSN_TIME_WAIT * 1000UL);
			#else
				#ifdef MBED
					wait_ms(MQTTSN_TIME_WAIT * 1000);
				#else
					usleep(MQTTSN_TIME_WAIT * 1000000);
				#endif
			#endif
			setMsgRequestStatus(MQTTSN_MSG_REQUEST);
		}

    	/*------ Send Top message in SendQue -----*/
    	if (getMsgRequestStatus() != MQTTSN_MSG_REQUEST){
    		return MQTTSN_ERR_NO_ERROR;
    	}

        D_MQTTW("Ucast ");
        D_MQTTLN(_sendQ->getMessage(0)->getMsgTypeName());
        D_MQTTF("%s\r\n", _sendQ->getMessage(0)->getMsgTypeName());

        _network->send(_sendQ->getMessage(0)->getMsgBuff(), _sendQ->getMessage(0)->getLength(), UcastReq);
        _clientStatus.setLastSendTime();

        _sendQ->getMessage(0)->setDup();
        _respTimer.start(packetReadTimeout * 1000UL);
        setMsgRequestStatus(MQTTSN_MSG_WAIT_ACK);

        while(!_respTimer.isTimeUp()){
            if ((getMsgRequestType() == MQTTSN_TYPE_PUBLISH &&
            		  _sendQ->getMessage(0)->getQos() == 0 )  ||
            	getMsgRequestType() == MQTTSN_TYPE_PUBACK     ||
				getMsgRequestType() == MQTTSN_TYPE_REGACK     ||
				getMsgRequestType() == MQTTSN_TYPE_PUBCOMP    ||
				getMsgRequestStatus() == MQTTSN_MSG_COMPLETE ){
            	//clearMsgRequest();
                return MQTTSN_ERR_NO_ERROR;
            }else if(getMsgRequestType() == MQTTSN_TYPE_DISCONNECT ){
            	_clientStatus.recvDISCONNECT();
            	//clearMsgRequest();
				return MQTTSN_ERR_NO_ERROR;
        	}else if (getMsgRequestStatus() == MQTTSN_MSG_REJECTED){
        		//clearMsgRequest();
				return MQTTSN_ERR_REJECTED;
            }
            /*----- Read response  ----*/
			int rc = _network->readPacket();
			if(rc == MQTTSN_READ_RESP_ONCE_MORE){
				rc = _network->readPacket();
			}
            if(rc == MQTTSN_ERR_INVALID_TOPICID){
            	//clearMsgRequest();
            	return rc;
            }else if(rc == PACKET_MODEM_STATUS ){
				break;
			}

            /*---- WILLTOPICREQ ,WILLMESSAGEREQ PUBREC are received ---*/
            if (getMsgRequestStatus() == MQTTSN_MSG_REQUEST &&
               (getMsgRequestType() == MQTTSN_TYPE_WILLTOPIC ||
                getMsgRequestType() == MQTTSN_TYPE_WILLMSG  ||
                getMsgRequestType() == MQTTSN_TYPE_PUBREL)){
            	retry = 0;
                break;
            }

        }
		setMsgRequestStatus(MQTTSN_MSG_REQUEST);
        retry++;
    }
    return MQTTSN_ERR_RETRY_OVER;
}

/*========================================
 *   Create & send the MQTT-S Messages
 =========================================*/
#ifdef MQTTSN_DEBUG
const char* gateWayLost = " Gateway lost\r\n";
#endif

/*--------- REGISTER ------*/
int MqttsnClient::registerTopic(MQString* topic){
    MqttsnRegister mqttsMsg = MqttsnRegister();
    mqttsMsg.setTopicName(topic);
    mqttsMsg.setMsgId(getNextMsgId());
    _topics.addTopic(topic);

    D_MQTTW("\nREGISTER SEND Topic = ");
	D_MQTTLN(topic->getConstStr());
	D_MQTTF("%s\r\n", topic->getConstStr());

    requestPrioritySendMsg((MqttsnMessage*)&mqttsMsg);
    return exec();
}

/*--------- PUBLISH ------*/
int MqttsnClient::publish(MQString* topic, const char* data, int dataLength, uint8_t qos){
	if(!_clientStatus.isAvailableToSend()){
		return MQTTSN_ERR_NOT_CONNECTED;
	}

	MqttsnPublish mqttsMsg = MqttsnPublish();
	uint16_t topicId = _topics.getTopicId(topic);

    if (topic->getCharLength() == 2 && topicId == 0){
		_topics.addTopic(topic);
		if(topic->getStr()){
			topicId = getUint16((uint8_t*)topic->getStr());
		}else{
			topicId = getUint16((uint8_t*)topic->getConstStr());
		}
		_topics.setTopicId(topic, topicId);
		mqttsMsg.setFlags(_clientFlg | MQTTSN_TOPIC_TYPE_SHORT);
		mqttsMsg.setTopicId(topicId);
	}else{
		mqttsMsg.setFlags(_clientFlg | MQTTSN_TOPIC_TYPE_NORMAL);
		if(topicId){
			mqttsMsg.setTopicId(topicId);
		}else{
			registerTopic(topic);
			mqttsMsg.setTopicId(_topics.getTopicId(topic));
		}
	}
    mqttsMsg.setQos(qos);
	mqttsMsg.setData((uint8_t*)data, (uint8_t)dataLength);
	if (qos){
		mqttsMsg.setMsgId(getNextMsgId());
	}
	requestSendMsg((MqttsnMessage*)&mqttsMsg);

	D_MQTTW("PUBLISH SEND msgID = ");
	D_MQTTLN(mqttsMsg.getMsgId(),DEC);
	D_MQTTF("%d\r\n", mqttsMsg.getMsgId());

	int rc = exec();
	if (rc == MQTTSN_ERR_INVALID_TOPICID){
		registerTopic(topic);
	}
	return rc;
}

/*--------- PUBLISH ------*/
int MqttsnClient::publish(MQString* topic, MQString* data, uint8_t qos){
	if(data->getStr()){
		return publish(topic, (const char*)data->getStr(),data->getCharLength(), qos);
	}else if(data->getConstStr()){
		return publish(topic, data->getConstStr(),data->getCharLength(), qos);
	}
	return MQTTSN_ERR_NO_DATA;
}

/*--------- PUBLISH ------*/
int MqttsnClient::publish(MQString* topic, Payload* payload, uint8_t qos){
	return publish(topic, (const char*)(payload->getBuf()), payload->getLen(), qos);
}
/*--------- PUBLISH ------*/
int MqttsnClient::publish(uint16_t predefinedId, const char* data, int dataLength, uint8_t qos){
	MqttsnPublish mqttsMsg = MqttsnPublish();
    mqttsMsg.setFlags(_clientFlg | MQTTSN_TOPIC_TYPE_PREDEFINED);
    mqttsMsg.setTopicId(predefinedId);
    mqttsMsg.setQos(qos);
    mqttsMsg.setData((uint8_t*)data, (uint8_t)dataLength);
    if (qos){
        mqttsMsg.setMsgId(getNextMsgId());
    }
    requestSendMsg((MqttsnMessage*)&mqttsMsg);
    return exec();
}

/*--------- SUBSCRIBE ------*/
int MqttsnClient::subscribe(MQString* topic, TopicCallback callback, uint8_t qos){
	MqttsnSubscribe mqttsMsg = MqttsnSubscribe();
    uint16_t topicId = _topics.getTopicId(topic);
    if (topic->getCharLength() == 2 && topicId == 0){
    	if(topic->getStr()){
    		topicId = getUint16((uint8_t*)topic->getStr());
    	}else{
    		topicId = getUint16((uint8_t*)topic->getConstStr());
    	}
		_topics.addTopic(topic);
		_topics.setTopicId(topic, topicId);
		_topics.setCallback(topic, callback);
		mqttsMsg.setTopicName(topic);
		mqttsMsg.setFlags(_clientFlg | MQTTSN_TOPIC_TYPE_SHORT);
    }else{
        mqttsMsg.setTopicName(topic);
        mqttsMsg.setFlags(_clientFlg | MQTTSN_TOPIC_TYPE_NORMAL);
        _topics.addTopic(topic);
        _topics.setCallback(topic, callback);
    }
    mqttsMsg.setMsgId(getNextMsgId());
    mqttsMsg.setQos(qos);
    requestSendMsg((MqttsnMessage*)&mqttsMsg);
    return exec();
}

/*--------- SUBSCRIBE ------*/
int MqttsnClient::subscribe(uint16_t predefinedId, TopicCallback callback, uint8_t qos){
	MqttsnSubscribe mqttsMsg = MqttsnSubscribe();
    mqttsMsg.setTopicId(predefinedId);
    mqttsMsg.setFlags(_clientFlg | MQTTSN_TOPIC_TYPE_PREDEFINED);
    mqttsMsg.setMsgId(getNextMsgId());
    _topics.setCallback(predefinedId, callback);
    requestSendMsg((MqttsnMessage*)&mqttsMsg);
    return exec();
}

/*--------- UNSUBSCRIBE ------*/
int MqttsnClient::unsubscribe(MQString* topic){
    MqttsnUnsubscribe mqttsMsg = MqttsnUnsubscribe();
    uint16_t topicId = _topics.getTopicId(topic);
    if (topic->getCharLength() == 2 && topicId > 0){
        mqttsMsg.setTopicName(topic);
        mqttsMsg.setFlags(_clientFlg | MQTTSN_TOPIC_TYPE_SHORT);
    }else{
        mqttsMsg.setTopicId(topicId);
		mqttsMsg.setFlags(_clientFlg | MQTTSN_TOPIC_TYPE_NORMAL);
    }
    mqttsMsg.setMsgId(getNextMsgId());
    requestSendMsg((MqttsnMessage*)&mqttsMsg);
    return exec();
}

/*--------- UNSUBSCRIBE ------*/
int MqttsnClient::unsubscribe(uint16_t predefinedId){
    MqttsnUnsubscribe mqttsMsg = MqttsnUnsubscribe();
    mqttsMsg.setTopicId(predefinedId);
    mqttsMsg.setMsgId(getNextMsgId());
    mqttsMsg.setFlags(_clientFlg | MQTTSN_TOPIC_TYPE_PREDEFINED);
    requestSendMsg((MqttsnMessage*)&mqttsMsg);
    return exec();
}

/*--------- DISCONNECT ------*/
int MqttsnClient::disconnect(uint16_t duration){
    MqttsnDisconnect mqttsMsg = MqttsnDisconnect();
    if (duration){
        mqttsMsg.setDuration(duration);
    }
    requestSendMsg((MqttsnMessage*)&mqttsMsg);
    return exec();
}

/*====  private Messages ====*/

/*--------- SEARCHGW ------*/
int  MqttsnClient::searchGw(uint8_t radius){
    MqttsnSearchGw mqttsMsg = MqttsnSearchGw();
    mqttsMsg.setRadius(radius);
    delayTime(MQTTSN_TIME_SEARCHGW);
    return requestPrioritySendMsg((MqttsnMessage*)&mqttsMsg);
}

/*--------- CONNECT ------*/
int MqttsnClient::connect(){
    MqttsnConnect mqttsMsg = MqttsnConnect(_clientId);
    mqttsMsg.setDuration(_clientStatus.getKeepAlive());
    mqttsMsg.setFlags(_clientFlg);
    return requestPrioritySendMsg((MqttsnMessage*)&mqttsMsg);
}

/*--------- WILLTOPIC ------*/
int MqttsnClient::willTopic(){
    MqttsnWillTopic mqttsMsg = MqttsnWillTopic();
    mqttsMsg.setWillTopic(_willTopic);
    mqttsMsg.setFlags(_clientFlg & 0x70);
    return requestSendMsg((MqttsnMessage*)&mqttsMsg);
}

/*--------- WILLMSG ------*/
int MqttsnClient::willMsg(){
    MqttsnWillMsg mqttsMsg = MqttsnWillMsg();
    mqttsMsg.setWillMsg(_willMessage);
    return requestSendMsg((MqttsnMessage*)&mqttsMsg);
}

/*--------- PUBACK ------*/
int MqttsnClient::pubAck(uint16_t topicId, uint16_t msgId, uint8_t rc){
    MqttsnPubAck mqttsMsg = MqttsnPubAck();
    mqttsMsg.setTopicId(topicId);
    mqttsMsg.setMsgId(msgId);
    mqttsMsg.setReturnCode(rc);

    D_MQTTW("\nPUBACK SEND MsgId = ");
	D_MQTTLN(msgId,DEC);
	D_MQTTF("%d", msgId);

    return requestPrioritySendMsg((MqttsnMessage*)&mqttsMsg);
    //return requestSendMsg((MqttsnMessage*)&mqttsMsg);
}

/*--------- PUBREC ------*/
int MqttsnClient::pubRec(uint16_t msgId){
	MqttsnPubRec mqttsMsg = MqttsnPubRec();
    mqttsMsg.setMsgId(msgId);
    return requestPrioritySendMsg((MqttsnMessage*)&mqttsMsg);
}

/*--------- PUBREL ------*/
int MqttsnClient::pubRel(uint16_t msgId){
    MqttsnPubRel mqttsMsg = MqttsnPubRel();
    mqttsMsg.setMsgId(msgId);
    return requestPrioritySendMsg((MqttsnMessage*)&mqttsMsg);
}

/*--------- PUBCOMP ------*/
int MqttsnClient::pubComp(uint16_t msgId){
    MqttsnPubComp mqttsMsg = MqttsnPubComp();
    mqttsMsg.setMsgId(msgId);
    return requestPrioritySendMsg((MqttsnMessage*)&mqttsMsg);
}

/*--------- REGACK ------*/
int MqttsnClient::regAck(uint16_t topicId, uint16_t msgId, uint8_t rc){
    MqttsnRegAck mqttsMsg = MqttsnRegAck();
    mqttsMsg.setTopicId(topicId);
    mqttsMsg.setMsgId(msgId);
    mqttsMsg.setReturnCode(rc);
    return requestSendMsg((MqttsnMessage*)&mqttsMsg);
}

/*--------- PINGREQ ------*/
int  MqttsnClient::pingReq(MQString* clientId){
    MqttsnPingReq mqttsMsg = MqttsnPingReq(clientId);
    return requestSendMsg((MqttsnMessage*)&mqttsMsg);
}

/* ===================================================
          Procedures for  Received Messages
 =====================================================*/
void MqttsnClient::recieveMessageHandler(NWResponse* recvMsg, int* returnCode){
	uint8_t msgType = recvMsg->getType();
    if ( (_clientStatus.isSearching() && msgType != MQTTSN_TYPE_GWINFO) ||
    	 (_clientStatus.isSubscribing() && msgType == MQTTSN_TYPE_PUBLISH ) ||
		 (getMsgRequestType() == MQTTSN_TYPE_PUBLISH &&
		  getMsgRequestStatus() == MQTTSN_MSG_WAIT_ACK &&
		  msgType == MQTTSN_TYPE_PUBLISH ) )
    {
    	D_MQTTW("Ignore Received Message\r\n");
/*---------  GWINFO  ----------*/
	}else if (msgType == MQTTSN_TYPE_GWINFO && recvMsg->getPayloadLength() == 3){
		D_MQTTW("GWINFO recv\r\n");
		MqttsnGwInfo mqMsg = MqttsnGwInfo();
		copyMsg(&mqMsg, recvMsg);
		if (getMsgRequestType() == MQTTSN_TYPE_SEARCHGW){
			if(_clientStatus.recvGWINFO(&mqMsg)){
				setMsgRequestStatus(MQTTSN_MSG_COMPLETE);
			}else{
				setMsgRequestStatus(MQTTSN_MSG_REJECTED);
			}
			if(isCleanSession()){
				_topics.clearTopic();
			}
			_network->setGwAddress();
		}
/*---------  REGISTER  ----------*/
	}else if (msgType == MQTTSN_TYPE_REGISTER){

		if(_clientStatus.isAvailableToSend()){
			D_MQTTW("REGISTER recv\r\n");
			MqttsnRegister mqMsg = MqttsnRegister();

			mqMsg.setFrame(recvMsg);
			uint16_t topicId = _topics.getTopicId(mqMsg.getTopicName());
			if (topicId == 0){
				if (_topics.match(mqMsg.getTopicName())){
					MQString* mqStr = mqMsg.getTopicName()->create();
					_topics.addTopic(mqStr);
					_topics.setTopicId(mqStr,mqMsg.getTopicId());
					_topics.setCallback(mqMsg.getTopicId(),_topics.match(mqStr)->getCallback());
				}
			}

			if (mqMsg.getMsgId() != 0){
				regAck(mqMsg.getTopicId(), mqMsg.getMsgId(), MQTTSN_RC_ACCEPTED);
				if(getMsgRequestStatus() == MQTTSN_MSG_WAIT_ACK){
					*returnCode = MQTTSN_READ_RESP_ONCE_MORE;
				}
			}else{
				D_MQTTW("RegAck is not required\r\n");
			}
		}else{
			D_MQTTW("REGISTER recv. Client is not Active\r\n");
		}
/*---------  PUBLISH  --------*/
    }else if (msgType == MQTTSN_TYPE_PUBLISH){

    	if(_clientStatus.isAvailableToSend()){
    		_clientStatus.setSubscribing(true);
			MqttsnPublish mqMsg = MqttsnPublish();
			mqMsg.setFrame(recvMsg);

    		D_MQTTW("PUBLISH RECV msgID = ");
    		D_MQTTLN(mqMsg.getMsgId(),DEC);
    		D_MQTTF("%d\r\n", mqMsg.getMsgId());

			if (mqMsg.getQos() == QOS1){
				pubAck(mqMsg.getTopicId(), mqMsg.getMsgId(), MQTTSN_RC_ACCEPTED);
				unicast(MQTTSN_TIME_RETRY);
			}else if(mqMsg.getQos() == QOS2){
				pubRec(mqMsg.getMsgId());
				unicast(MQTTSN_TIME_RETRY);
			}

			if(getMsgRequestStatus() == MQTTSN_MSG_WAIT_ACK){
				*returnCode = MQTTSN_READ_RESP_ONCE_MORE;
			}
			_pubHdl.exec(&mqMsg,&_topics);   //  Execute Callback routine
    		_clientStatus.setSubscribing(false);
    	}else{
    		D_MQTTW("PUBLISH recv Client is not Active\r\n");
    	}

/*===========  Response  =========*/

/*---------  PUBACK  ----------*/
    }else if (msgType == MQTTSN_TYPE_PUBACK &&
    		(getMsgRequestStatus() == MQTTSN_MSG_WAIT_ACK &&
    		 getMsgRequestType() == MQTTSN_TYPE_PUBLISH)){
        MqttsnPubAck mqMsg = MqttsnPubAck();
        copyMsg(&mqMsg, recvMsg);

        D_MQTTW("\nPUBACK RECV MsgId = ");
		D_MQTTLN(mqMsg.getMsgId(),DEC);
		D_MQTTF("%d", mqMsg.getMsgId());
        D_MQTTW(" RC=");
        D_MQTTLN(mqMsg.getReturnCode(),DEC);
        D_MQTTF("%d\r\n", mqMsg.getReturnCode());

        if (mqMsg.getMsgId() == getUint16(_sendQ->getMessage(0)->getBody() + 3)){
            if (mqMsg.getReturnCode() == MQTTSN_RC_ACCEPTED){
                setMsgRequestStatus(MQTTSN_MSG_COMPLETE);

            }else if (mqMsg.getReturnCode() == MQTTSN_RC_REJECTED_CONGESTION){
                  setMsgRequestStatus(MQTTSN_MSG_RESEND_REQ);

            }else if (mqMsg.getReturnCode() == MQTTSN_RC_REJECTED_INVALID_TOPIC_ID){
                *returnCode = MQTTSN_ERR_INVALID_TOPICID;
                setMsgRequestStatus(MQTTSN_MSG_REJECTED);
            }
        }else{
        	//D_MQTTW("MsgId dosn't match.\r\n");
        }

/*---------  PINGRESP  ----------*/
    }else if (msgType == MQTTSN_TYPE_PINGRESP){
        D_MQTTW("PINGRESP recv\r\n");

        if (getMsgRequestType() == MQTTSN_TYPE_PINGREQ){
        	_clientStatus.recvPINGRESP();
            setMsgRequestStatus(MQTTSN_MSG_COMPLETE);
        }

/*---------  ADVERTISE  ----------*/
    }else if (msgType == MQTTSN_TYPE_ADVERTISE){
        D_MQTTW("ADVERTISE recv\r\n");

        MqttsnAdvertise mqMsg = MqttsnAdvertise();
        copyMsg(&mqMsg, recvMsg);
        _clientStatus.recvADVERTISE(&mqMsg);
        if(getMsgRequestStatus() == MQTTSN_MSG_WAIT_ACK){
        	*returnCode = MQTTSN_READ_RESP_ONCE_MORE;
        }

/*---------  CONNACK  ----------*/
    }else if (msgType == MQTTSN_TYPE_CONNACK){
        if ((getMsgRequestType() == MQTTSN_TYPE_CONNECT || getMsgRequestType() == MQTTSN_TYPE_WILLMSG)){
            MqttsnConnack mqMsg = MqttsnConnack();
            copyMsg(&mqMsg, recvMsg);

            D_MQTTW("CONNACK recv RC=");
			D_MQTTLN(mqMsg.getReturnCode(),DEC);
			D_MQTTF("%d\r\n", mqMsg.getReturnCode());

            if (mqMsg.getReturnCode() == MQTTSN_RC_ACCEPTED){
                setMsgRequestStatus(MQTTSN_MSG_COMPLETE);
                _clientStatus.recvCONNACK();

            }/*else if (mqMsg.getReturnCode() == MQTTSN_RC_REJECTED_CONGESTION){
            	setMsgRequestStatus(MQTTSN_MSG_COMPLETE);
           		*returnCode = MQTTSN_ERR_REJECTED;
           		_clientStatus.recvDISCONNECT();
            }*/else{
               setMsgRequestStatus(MQTTSN_MSG_REJECTED);
               *returnCode = MQTTSN_ERR_REJECTED;
               _clientStatus.recvDISCONNECT();
            }
        }
        D_MQTTW("\r\n");

/*---------  REGACK  ----------*/
    }else if (msgType == MQTTSN_TYPE_REGACK){
         D_MQTTW("REGACK recv\r\n");

        if (getMsgRequestStatus() == MQTTSN_MSG_WAIT_ACK &&
            getMsgRequestType() == MQTTSN_TYPE_REGISTER){
            MqttsnRegAck mqMsg = MqttsnRegAck();
            copyMsg(&mqMsg, recvMsg);
            if (mqMsg.getMsgId() == getUint16(_sendQ->getMessage(0)->getBody() + 2)){
				if (getUint16((uint8_t*)_sendQ->getMessage(0)->getBody()+4)){
					if (mqMsg.getReturnCode() == MQTTSN_RC_ACCEPTED){
						setMsgRequestStatus(MQTTSN_MSG_COMPLETE);
						MQString topic;
						topic.readBuf(_sendQ->getMessage(0)->getBody() + 4);
						_topics.setTopicId(&topic, mqMsg.getTopicId());
					}else if (mqMsg.getReturnCode() == MQTTSN_RC_REJECTED_CONGESTION){
					  setMsgRequestStatus(MQTTSN_MSG_RESEND_REQ);
					}else{
						*returnCode = MQTTSN_ERR_REJECTED;
					}
				}
            }
        }

/*---------  SUBACK  ----------*/
    }else if (msgType == MQTTSN_TYPE_SUBACK && getMsgRequestStatus() == MQTTSN_MSG_WAIT_ACK){
        MqttsnSubAck mqMsg = MqttsnSubAck();
        copyMsg(&mqMsg, recvMsg);

        D_MQTT("\nSUBACK RC=");
        D_MQTTLN(mqMsg.getReturnCode(),HEX);
        D_MQTTF("\nSUBACK RC=%d\r\n", mqMsg.getReturnCode());

        if (mqMsg.getMsgId() == getUint16(_sendQ->getMessage(0)->getBody() + 1)){
            if (mqMsg.getReturnCode() == MQTTSN_RC_ACCEPTED){
                setMsgRequestStatus(MQTTSN_MSG_COMPLETE);
                if (_sendQ->getMessage(0)->getBodyLength() > 5){ // TopicName is not Id
                    MQString topic;
                    topic.copy(_sendQ->getMessage(0)->getBody() + 3,
                    		_sendQ->getMessage(0)->getBodyLength() - 3);
                    _topics.setTopicId(&topic, mqMsg.getTopicId());
                }
            }else if (mqMsg.getReturnCode() == MQTTSN_RC_REJECTED_CONGESTION){
                setMsgRequestStatus(MQTTSN_MSG_RESEND_REQ);
            }else{
                *returnCode = MQTTSN_ERR_REJECTED;       // Return Code
            }
        }

/*---------  UNSUBACK  ----------*/
    }else if (msgType == MQTTSN_TYPE_UNSUBACK && getMsgRequestStatus() == MQTTSN_MSG_WAIT_ACK){
        D_MQTTW("UNSUBACK recv\r\n");
        MqttsnUnSubAck mqMsg = MqttsnUnSubAck();
        copyMsg(&mqMsg, recvMsg);
        if (mqMsg.getMsgId() == getUint16(_sendQ->getMessage(0)->getBody() + 1)){
              setMsgRequestStatus(MQTTSN_MSG_COMPLETE);
        }

/*---------  DISCONNECT  ----------*/
    }else if (msgType == MQTTSN_TYPE_DISCONNECT){
         D_MQTTW("DISCONNECT recv\r\n");
         setMsgRequestStatus(MQTTSN_MSG_COMPLETE);
         _clientStatus.recvDISCONNECT();


/*---------  WILLTOPICREQ  ----------*/
    }else if (msgType == MQTTSN_TYPE_WILLTOPICREQ){
        D_MQTTW("WILLTOPICREQ recv\r\n");
        if (getMsgRequestType() == MQTTSN_TYPE_CONNECT){
            //clearMsgRequest();
        	//setMsgRequestStatus(MQTTSN_MSG_COMPLETE);
            MqttsnWillTopic mqMsg = MqttsnWillTopic();
            mqMsg.setFlags(0);                               // ToDo:  add  WillQoS, WillRetain to appConfig
            mqMsg.setWillTopic(_willTopic);
            requestPrioritySendMsg((MqttsnMessage*)&mqMsg);
        }

/*---------  WILLMSGREQ  -----------*/
    }else if (msgType == MQTTSN_TYPE_WILLMSGREQ){
        D_MQTTW("WILLMSGREQ recv\r\n");
        if (getMsgRequestType() == MQTTSN_TYPE_WILLTOPIC){
            //clearMsgRequest();
        	//setMsgRequestStatus(MQTTSN_MSG_COMPLETE);
            MqttsnWillMsg mqMsg = MqttsnWillMsg();
            mqMsg.setWillMsg(_willMessage);
            requestPrioritySendMsg((MqttsnMessage*)&mqMsg);
        }

/*---------  PUBREC  ----------*/
	}else if (msgType == MQTTSN_TYPE_PUBREC &&
			(getMsgRequestStatus() == MQTTSN_MSG_WAIT_ACK &&
			 getMsgRequestType() == MQTTSN_TYPE_PUBLISH)){
		MqttsnPubRec mqMsg = MqttsnPubRec();
		copyMsg(&mqMsg, recvMsg);

		D_MQTTW("\nPUBREC recv\r\n");

		if (mqMsg.getMsgId() == getUint16(_sendQ->getMessage(0)->getBody() + 3)){
			MqttsnPubRel mqrMsg = MqttsnPubRel();
			mqrMsg.setMsgId(mqMsg.getMsgId());
			requestPrioritySendMsg((MqttsnMessage*)&mqrMsg);
		}

/*---------  PUBREL  ----------*/
	}else if (msgType == MQTTSN_TYPE_PUBREL &&
			(getMsgRequestStatus() == MQTTSN_MSG_WAIT_ACK &&
			 getMsgRequestType() == MQTTSN_TYPE_PUBREC)){
		MqttsnPubRel mqMsg = MqttsnPubRel();
		copyMsg(&mqMsg, recvMsg);

		D_MQTTW("\nPUBREL recv\r\n");

		if (mqMsg.getMsgId() == getUint16(_sendQ->getMessage(0)->getBody())){
			//clearMsgRequest();  // delete PUBREC
			MqttsnPubComp mqrMsg = MqttsnPubComp();
			mqrMsg.setMsgId(mqMsg.getMsgId());
			requestPrioritySendMsg((MqttsnMessage*)&mqMsg);
		}

/*---------  PUBCOMP  ----------*/
	}else if (msgType == MQTTSN_TYPE_PUBCOMP &&
			(getMsgRequestStatus() == MQTTSN_MSG_WAIT_ACK &&
			 getMsgRequestType() == MQTTSN_TYPE_PUBREL)){
		MqttsnPubComp mqMsg = MqttsnPubComp();
		copyMsg(&mqMsg, recvMsg);

		D_MQTTW("\nPUBCOMP recv\r\n");

		if (mqMsg.getMsgId() == getUint16(_sendQ->getMessage(0)->getBody())){
			//clearMsgRequest();    // delete request of PUBREL
			setMsgRequestStatus(MQTTSN_MSG_COMPLETE);  // PUBLISH complete
		}
    }else{
		*returnCode = PACKET_ERROR_NODATA;
	}
}


/*=====================================
        Class SendQue
 ======================================*/
SendQue::SendQue(){
    _queCnt = 0;
    _queSize = SENDQ_SIZE;
}
SendQue::~SendQue(){
    for( int i = 0; i < _queCnt; i++){
        delete _msg[i];
    }
}

int SendQue::addRequest(MqttsnMessage* msg){
    if ( _queCnt < _queSize){

		D_MQTTW("\nAdd SendQue size = ");
		D_MQTT(_queCnt + 1, DEC);
		D_MQTT(" Msg = 0x");
		D_MQTTLN(msg->getType(), HEX);
		D_MQTTF("%d  Msg = 0x%x\r\n", _queCnt + 1, msg->getType());

        _msg[_queCnt] =new MqttsnMessage();
        _msg[_queCnt++]->copy(msg);
        return _queCnt - 1;
    }
    return MQTTSN_ERR_CANNOT_ADD_REQUEST; // Over Que size
}

int SendQue::addPriorityRequest(MqttsnMessage* msg){
    if ( _queCnt < _queSize){
	_queCnt++;
    }

	D_MQTTW("\nAdd SendQue Top Size = ");
	D_MQTT(_queCnt, DEC);
	D_MQTT("  Msg = 0x");
	D_MQTTLN(msg->getType(), HEX);
	D_MQTTF("%d  Msg = 0x%x", _queCnt, msg->getType());

    for(int i = _queCnt-1; i > 0; i--){
        _msg[i] = _msg[i - 1];
    }
    _msg[0] = new MqttsnMessage();
    _msg[0]->copy(msg);


        for(int i = 1; i < _queCnt; i++){
     	    D_MQTT( "  Msg = 0x");
			D_MQTT(_msg[i]->getType(), HEX);
			D_MQTTF("  Msg = 0x%x ", _msg[i]->getType());
        }
        D_MQTTW("\r\n");

    return 0;
}

int SendQue::deleteRequest(uint8_t index){

	if ( index < _queCnt){

        delete _msg[index];
        _queCnt--;

    	D_MQTTW("\nDelete SendQue  Size = ");
    	D_MQTT(_queCnt, DEC);
    	D_MQTTF("%d", _queCnt);

        for(int i = index; i < _queCnt; i++){
            _msg[i] = _msg[i + 1];

            D_MQTT( "  Msg = 0x");
			D_MQTT(_msg[i]->getType(), HEX);
			D_MQTTF("  Msg = 0x%x ", _msg[i]->getType());

        }

        D_MQTTW("\r\n");
        for(int i = _queCnt; i < _queSize; i++){
            _msg[i] = 0;
        }

        return 0;
    }
    return -2;
}

void   SendQue::deleteAllRequest(){
    while ( _queCnt > 0){
        deleteRequest(0);
    }
}

void SendQue::setStatus(uint8_t index, uint8_t status){
    if ( index < _queCnt){
        _msg[index]->setStatus(status);
    }
}

void SendQue::setQueSize(uint8_t sz){
  _queSize = sz;
}

MqttsnMessage* SendQue::getMessage(uint8_t index){
  if ( index < _queCnt){
      return _msg[index];
  }
  return 0;
}

int SendQue::getStatus(uint8_t index){
  if ( index < _queCnt){
      return _msg[index]->getStatus();
  }
  return -1;
}

uint8_t SendQue::getCount(){
   return _queCnt;
}


/*=====================================
        Class SendQue
 ======================================*/
ClientStatus::ClientStatus(){
	init();
	_gwId = 0;
	_sleepModeFlg = false;
	_onConnectFlg = false;
	_subscribingFlg = false;
}

ClientStatus::~ClientStatus(){
}

void ClientStatus::init(){
	_gwStat = GW_LOST;
	_clStat = CL_DISCONNECTED;

	_keepAliveDuration = MQTTSN_DEFAULT_DURATION;
	_advertiseDuration = MQTTSN_DEFAULT_KEEPALIVE;
	_advertiseTimer.stop();
	_keepAliveTimer.stop();
	_onConnectFlg = false;
	_subscribingFlg = false;
}


void ClientStatus::changeUTC(){
	_keepAliveTimer.changeUTC();   // changeUTC() is static
}

bool ClientStatus::isLost(){
	if(_gwStat == GW_LOST){
		return true;
	}else{
		return false;
	}
}

bool ClientStatus::isSearching(){
	if(_gwStat == GW_SEARCHING){
		return true;
	}else{
		return false;
	}
}


bool ClientStatus::isConnected(){
	if(_gwStat == GW_FIND && _clStat != CL_DISCONNECTED){
		return true;
	}else{
		return false;
	}
}

bool ClientStatus::isOnConnected(){
	return _onConnectFlg;
}

bool ClientStatus::isSubscribing(){
	return _subscribingFlg;
}

bool ClientStatus::isAvailableToSend(){
	if((_gwStat == GW_FIND) &&
		((_clStat == CL_ACTIVE) || (_clStat == CL_ASLEEP))){
		return true;
	}else{
		return false;
	}
}

bool ClientStatus::isPINGREQRequired(){
	if(_sleepModeFlg){
		_keepAliveTimer.changeUTC();
	}
	return (_keepAliveTimer.isTimeUp() && isConnected());
}

bool ClientStatus::isGatewayAlive(){
	if(_sleepModeFlg){
		_advertiseTimer.changeUTC();
	}
	return (_advertiseTimer.isTimeUp() ? false : true);
}

uint16_t ClientStatus::getKeepAlive(){
	return _keepAliveDuration;
}

void ClientStatus::setKeepAlive(uint16_t sec){
	_keepAliveDuration = sec;
}

void ClientStatus::sendSEARCHGW(){
	_gwStat = GW_SEARCHING;
}

bool ClientStatus::recvGWINFO(MqttsnGwInfo* gwi){
	bool rc = true;
	if (_gwStat == GW_SEARCHING){
		_gwStat = GW_FIND;
		if(_gwId != gwi->getGwId() && _gwId != 0){
			rc = false;
		}
		_gwId = gwi->getGwId();
	}
	return rc;
}

void ClientStatus::recvADVERTISE(MqttsnAdvertise* adv){
	if ( adv->getGwId() == _gwId ){
		_advertiseDuration = (adv->getDuration() > 60 ?
				adv->getDuration() + adv->getDuration() / 10 :
				adv->getDuration() + adv->getDuration() / 2);
		_advertiseTimer.start((uint32_t)(_advertiseDuration * 1000UL));
	}
}

void ClientStatus::recvCONNACK(){
	_onConnectFlg = true;
}

void ClientStatus::recvDISCONNECT(){
	_clStat = CL_DISCONNECTED;
	_advertiseTimer.stop();
}

void ClientStatus::setLastSendTime(){
	_keepAliveTimer.start((uint32_t)(_keepAliveDuration * 1000UL));
}


void ClientStatus::recvPINGRESP(){
	setLastSendTime();
}

void ClientStatus::setAvailableToSend(){
	_clStat = CL_ACTIVE;
	_onConnectFlg = false;
}

void ClientStatus::setSubscribing(bool stat){
	_subscribingFlg = stat;
}

void ClientStatus::setModeSleep(){
	_sleepModeFlg = true;
}


/*===================  End of file ====================*/
