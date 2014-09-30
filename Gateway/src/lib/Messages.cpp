/*
 * Messages.cpp
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

#include "Defines.h"
#include "ZBStack.h"
#include "UDPStack.h"
#include "ProcessFramework.h"
#include "Messages.h"
//#include "ErrorMessage.h"

#include <string>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

extern uint16_t getUint16(uint8_t* pos);
extern void setUint16(uint8_t* pos, uint16_t val);
extern void setUint32(uint8_t* pos, uint32_t val);
extern uint8_t* mqcalloc(uint8_t length);
extern void utfSerialize(uint8_t* pos, string str);

using namespace tomyGateway;


/*=====================================
        Class MQTTSnMessage
 ======================================*/
MQTTSnMessage::MQTTSnMessage(){
    _message = 0;
    _length = 0;
    _type = 0;
}

MQTTSnMessage::~MQTTSnMessage(){
    if (_message){
        free(_message);
    }
}

void MQTTSnMessage::setType(uint8_t type){
    _type = type;
}

void MQTTSnMessage::setMessageLength(uint16_t length){
    _length = length;
}

void MQTTSnMessage::setBody(uint8_t* body, uint16_t bodyLength){
	if(_length > 255){
		_length = bodyLength + 4;
	}else{
		_length = bodyLength + MQTTSN_HEADER_SIZE;
	}
    allocate();
	memcpy(getBodyPtr(), body, bodyLength);
}

void MQTTSnMessage::allocate(){
		if ( _length ) {
			if (_message){
				  free(_message);
			}
			_message = mqcalloc(_length);
			if(_length > 255){
				*_message = 0x01;
				setUint32(_message + 1, _length);
				*(_message + 3) = _type;
			}else{
				*_message = _length;
				*(_message + 1) = _type;
			}
		}
}

uint16_t MQTTSnMessage::getMessageLength(){
    return _length;
}

uint8_t MQTTSnMessage::getType(){
    return _type;
}

uint8_t* MQTTSnMessage::getBodyPtr(){
	if( _length > 255){
		return _message + 4;
	}else{
		return _message + MQTTSN_HEADER_SIZE;
	}
}

uint16_t MQTTSnMessage::getBodyLength(){
	if(_length > 255){
		return _length - 4;
	}else{
		return _length - MQTTSN_HEADER_SIZE;
	}
}

bool MQTTSnMessage::getMessage(uint16_t pos, uint8_t& val){
	if(pos < 0 || pos >= _length){
		return false;
	}else{
		val = *(getMessagePtr() + pos);
		return true;
	}
}

uint8_t* MQTTSnMessage::getMessagePtr(){
	return _message;
}

void MQTTSnMessage::absorb(MQTTSnMessage* src){
    setMessageLength(src->getMessageLength());
    setType(src->getType());
    allocate();
    memcpy(getBodyPtr(), src->getBodyPtr(), (size_t)src->getBodyLength());
}

void MQTTSnMessage::absorb(NWResponse* src){
	setMessageLength(src->getPayloadLength());
	setType(src->getMsgType());
	allocate();
	memcpy(_message,src->getPayloadPtr(), (size_t)src->getPayloadLength());
}


/*=====================================
        Class MQTTSnAdvertise
 ======================================*/
MQTTSnAdvertise::MQTTSnAdvertise():MQTTSnMessage(){
    setMessageLength(5);
    setType(MQTTSN_TYPE_ADVERTISE);
    allocate();
}

MQTTSnAdvertise::~MQTTSnAdvertise(){

}

void MQTTSnAdvertise::setGwId(uint8_t id){
    getBodyPtr()[0] = id;
}

void MQTTSnAdvertise::setDuration(uint16_t sec){
    uint8_t* pos = getBodyPtr() + 1;
    setUint16(pos, sec);
}

uint8_t MQTTSnAdvertise::getGwId(){
    return getBodyPtr()[0];
}

uint16_t MQTTSnAdvertise::getDuration(){
  uint8_t* pos = getBodyPtr() + 1;
    return getUint16(pos);
}

/*=====================================
        Class MQTTSnSearchGw
 ======================================*/
MQTTSnSearchGw::MQTTSnSearchGw(){
    setMessageLength(3);
    setType(MQTTSN_TYPE_SEARCHGW);
    allocate();
}

MQTTSnSearchGw::~MQTTSnSearchGw(){

}

void MQTTSnSearchGw::setRadius(uint8_t radius){
  getBodyPtr()[0] = radius;
}

uint8_t MQTTSnSearchGw::getRadius(){
  return getBodyPtr()[0];
}

void MQTTSnSearchGw::absorb(NWResponse* src){
	MQTTSnMessage::absorb(src);
}

/*=====================================
        Class MQTTSnGwInfo
 ======================================*/
MQTTSnGwInfo::MQTTSnGwInfo(){
  setMessageLength(3);
  setType(MQTTSN_TYPE_GWINFO);
  allocate();
}

MQTTSnGwInfo::~MQTTSnGwInfo(){

}

uint8_t MQTTSnGwInfo::getGwId(){
    return getBodyPtr()[0];
}

void MQTTSnGwInfo::setGwId(uint8_t id){
    getBodyPtr()[0] = id;
}

/*=====================================
         Class MQTTSnConnect
  ======================================*/
MQTTSnConnect::MQTTSnConnect(){
    setMessageLength(6);
    allocate();
    setType(MQTTSN_TYPE_CONNECT);
    getBodyPtr()[1] = MQTTSN_PROTOCOL_ID;
}

MQTTSnConnect::MQTTSnConnect(string* id){
    setMessageLength(id->size() + 6);
    allocate();
    setType(MQTTSN_TYPE_CONNECT);
    getBodyPtr()[1] = MQTTSN_PROTOCOL_ID;
    id->copy((char*)getBodyPtr() + 4,id->size(),0);
}

MQTTSnConnect::~MQTTSnConnect(){

}

void MQTTSnConnect::setFlags(uint8_t flg){
    getBodyPtr()[0] = flg & 0x0c;
}

uint8_t MQTTSnConnect::getFlags(){
    return getBodyPtr()[0];
}

bool MQTTSnConnect::isCleanSession(){
	return getBodyPtr()[0] & MQTTSN_FLAG_CLEAN;
}

bool MQTTSnConnect::isWillRequired(){
	return getBodyPtr()[0] & MQTTSN_FLAG_WILL;
}

void MQTTSnConnect::setDuration(uint16_t sec){
    setUint16((uint8_t*)getBodyPtr() + 2, sec);
}

uint16_t MQTTSnConnect::getDuration(){
    return getUint16((uint8_t*)getBodyPtr() + 2);
}

void MQTTSnConnect::setClientId(string id){
	_clientId = id;
}

string* MQTTSnConnect::getClientId(){
	_clientId = string((char*)(getBodyPtr() + 4), getMessageLength() - 6 );
    return &_clientId;
}

void MQTTSnConnect::absorb(NWResponse* src){
	MQTTSnMessage::absorb(src);
}

void MQTTSnConnect::absorb(MQTTSnMessage* src){
	setMessageLength(src->getMessageLength());
	setClientId(*getClientId());
	setFlags(src->getBodyPtr()[0]);
	setDuration(getUint16(src->getBodyPtr() + 2));
	MQTTSnMessage::absorb(src);
}

/*=====================================
        Class MQTTSnConnack
 ======================================*/
MQTTSnConnack::MQTTSnConnack(){
    setMessageLength(3);
    setType(MQTTSN_TYPE_CONNACK);
    allocate();
}

MQTTSnConnack::~MQTTSnConnack(){

}

void MQTTSnConnack::setReturnCode(uint8_t rc){
    getBodyPtr()[0] = rc;
}

uint8_t MQTTSnConnack::getReturnCode(){
    return getBodyPtr()[0];
}

/*=====================================
       Class MQTTSnWillTopicReq
======================================*/
MQTTSnWillTopicReq::MQTTSnWillTopicReq(){
    setMessageLength(2);
    setType(MQTTSN_TYPE_WILLTOPICREQ);
    allocate();
}

MQTTSnWillTopicReq::~MQTTSnWillTopicReq(){

}

/*=====================================
         Class MQTTSnWillTopic
  ======================================*/
MQTTSnWillTopic::MQTTSnWillTopic(){
    setMessageLength(3);
    setType(MQTTSN_TYPE_WILLTOPIC);
    allocate();
    _flags = 0;
}

MQTTSnWillTopic::~MQTTSnWillTopic(){

}

void MQTTSnWillTopic::setFlags(uint8_t flags){
    flags &= 0x70;
    if (_message){
		getBodyPtr()[0] = flags;
    }
    _flags = flags;
}

void MQTTSnWillTopic::setWillTopic(string* topic){
    setMessageLength(topic->size() + 3);
    allocate();
    topic->copy((char*)getBodyPtr() + 1,topic->size() ,0);
    _message[2] = _flags;
    _topicName = *topic;
}

string* MQTTSnWillTopic::getWillTopic(){
	if (_message){
		return &_topicName;
	}else{
		return 0;
	}
}

bool MQTTSnWillTopic::isWillRequired(){
    return getBodyPtr()[0] && MQTTSN_FLAG_WILL;
}

uint8_t MQTTSnWillTopic::getQos(){
    return (_flags && (MQTTSN_FLAG_QOS_1 | MQTTSN_FLAG_QOS_2)) >> 5;
}

void MQTTSnWillTopic::absorb(NWResponse* src){
	MQTTSnMessage::absorb(src);
}

void MQTTSnWillTopic::absorb(MQTTSnMessage* src){
	setFlags(src->getBodyPtr()[0]);
	_topicName = string((char*)src->getBodyPtr() + 1, src->getMessageLength() - 3);
	MQTTSnMessage::absorb(src);
}

/*=====================================
         Class MQTTSnWillMsgReq
  ======================================*/
MQTTSnWillMsgReq::MQTTSnWillMsgReq(){
    setMessageLength(2);
    setType(MQTTSN_TYPE_WILLMSGREQ);
    allocate();

}

MQTTSnWillMsgReq::~MQTTSnWillMsgReq(){

}

/*=====================================
         Class MQTTSnWillMsg
  ======================================*/
MQTTSnWillMsg::MQTTSnWillMsg(){
    setMessageLength(2);
    setType(MQTTSN_TYPE_WILLMSG);
    allocate();
}

MQTTSnWillMsg::~MQTTSnWillMsg(){

}

void MQTTSnWillMsg::setWillMsg(string* msg){
    setMessageLength(2 + msg->size());
    allocate();
    msg->copy((char*)getBodyPtr(),msg->size(),0);
    _willMsg = *msg;
}

string* MQTTSnWillMsg::getWillMsg(){
	return &_willMsg;
}

void MQTTSnWillMsg::absorb(NWResponse* src){
	MQTTSnMessage::absorb(src);
}

void MQTTSnWillMsg::absorb(MQTTSnMessage* src){
	_willMsg = string((char*)src->getBodyPtr(), src->getMessageLength() - 2);
	MQTTSnMessage::absorb(src);
}
/*=====================================
         Class MQTTSnRegister
  ======================================*/
MQTTSnRegister::MQTTSnRegister(){
    setMessageLength(6);
    setType(MQTTSN_TYPE_REGISTER);
    allocate();
    _topicId = 0;
    _msgId = 0;
}

MQTTSnRegister::~MQTTSnRegister(){

}

void MQTTSnRegister::setTopicId(uint16_t topicId){
  if (_message){
            setUint16(getBodyPtr(), topicId);
    }
    _topicId = topicId;
}
uint16_t MQTTSnRegister::getTopicId(){
    return _topicId;
}
void MQTTSnRegister::setMsgId(uint16_t msgId){
    if (_message){
            setUint16(getBodyPtr() + 2, msgId);
    }
    _msgId = msgId;
}
uint16_t MQTTSnRegister::getMsgId(){
    return _msgId;

}
void MQTTSnRegister::setTopicName(string* topicName){
    setMessageLength(6 + topicName->size());
    allocate();
    topicName->copy((char*)getBodyPtr() + 4, topicName->size(),0);
    setTopicId(_topicId);
    setMsgId(_msgId);
}
/*
void MQTTSnRegister::setFrame(uint8_t* data, uint8_t len){
    //setMessageLength(len + MQTTSN_HEADER_SIZE);
    //allocate();
    //memcpy(getBodyPtr(), data, len);
    _topicId = getUint16(data);
    _msgId = getUint16(data + 2);
    _topicName = string((char*)getBodyPtr() + 4, len - 4);
}

void MQTTSnRegister::setFrame(NWResponse* resp){
    setFrame(resp->getPayloadPtr() + MQTTSN_HEADER_SIZE, resp->getPayloadPtr()[0] - MQTTSN_HEADER_SIZE);
}
*/
string* MQTTSnRegister::getTopicName(){
    return &_topicName;
}

void MQTTSnRegister::absorb(NWResponse* src){
	MQTTSnMessage::absorb(src);
}

void MQTTSnRegister::absorb(MQTTSnMessage* src){
	_topicId = getUint16((uint8_t*)(src->getBodyPtr()));
	_msgId = getUint16((uint8_t*)(src->getBodyPtr() +2));
	_topicName = string((char*)src->getBodyPtr() + 4, src->getMessageLength() - 6);
	MQTTSnMessage::absorb(src);
}

/*=====================================
         Class MQTTSnRegAck
  ======================================*/
MQTTSnRegAck::MQTTSnRegAck(){
    setMessageLength(7);
    setType(MQTTSN_TYPE_REGACK);
    allocate();
}
MQTTSnRegAck::~MQTTSnRegAck(){

}
void MQTTSnRegAck::setTopicId(uint16_t topicId){
    setUint16((uint8_t*)getBodyPtr(), topicId);
}
uint16_t MQTTSnRegAck::getTopicId(){
    return getUint16((unsigned char*)getBodyPtr());
}
void MQTTSnRegAck::setMsgId(uint16_t msgId){
    setUint16(getBodyPtr()+ 2,msgId);
}
uint16_t MQTTSnRegAck::getMsgId(){
    return getUint16((unsigned char*)getBodyPtr()+ 2);
}
void MQTTSnRegAck::setReturnCode(uint8_t rc){
    getBodyPtr()[4] = rc;
}
uint8_t MQTTSnRegAck::getReturnCode(){
    return (uint8_t)getBodyPtr()[4];
}

void MQTTSnRegAck::absorb(NWResponse* src){
	MQTTSnMessage::absorb(src);
}

/*=====================================
         Class MQTTSnPublish
  ======================================*/
MQTTSnPublish::MQTTSnPublish(){
    setMessageLength(7);
    setType(MQTTSN_TYPE_PUBLISH);
    allocate();
    _topicId = 0;
    _msgId = 0;
    _flags = 0;
}

MQTTSnPublish::~MQTTSnPublish(){

}

void MQTTSnPublish::setFlags(uint8_t flags){
    _flags = flags & 0xf3;
    getBodyPtr()[0] = _flags ;
}

uint8_t MQTTSnPublish::getFlags(){
    return _flags;
}

uint8_t MQTTSnPublish::getTopicType(){
    return _flags & MQTTSN_TOPIC_TYPE;
}

uint8_t MQTTSnPublish::getQos(){
    return ((_flags >> 5) & 0x03);
}

void MQTTSnPublish::setQos(uint8_t qos){
	_flags &= 0x9f;
	if( qos == 1){
		_flags |= MQTTSN_FLAG_QOS_1;
	}else if( qos == 2){
		_flags |= MQTTSN_FLAG_QOS_2;
	}
	setFlags(_flags);
}

void MQTTSnPublish::setDup(){
	_flags |= 0x80;
	setFlags(_flags);
}

void MQTTSnPublish::setRetain(){
	_flags |= 0x10;
	setFlags(_flags);
}

void MQTTSnPublish::setTopicIdType(uint8_t type){
	switch(type){
	case 0:
		_flags &= 0x9f;
		break;
	case 1:
		_flags &= 0x9f;
		_flags |= 0x01;
		break;
	default:
		break;
	}
	setFlags(_flags);
}

void MQTTSnPublish::setTopicId(uint16_t id){
    setUint16((uint8_t*)(getBodyPtr() + 1), id);
    _topicId = id;
}

void MQTTSnPublish::setTopic(string* topic){
   topic->copy((char*)(getBodyPtr() + 1), 2);
   _topicId = getUint16((uint8_t*)(getBodyPtr() + 1));
}

string* MQTTSnPublish::getTopic(string* str){
	char tp[3];
	tp[0] = (char)*(getBodyPtr() + 1);
	tp[1] = (char)*(getBodyPtr() + 2);
	tp[2] = 0;
	str->copy(tp,2);
	return str;
}

uint16_t MQTTSnPublish::getTopicId(){
    return _topicId;
}

void MQTTSnPublish::setMsgId(uint16_t msgId){
    setUint16((uint8_t*)(getBodyPtr() + 3), msgId);
    _msgId = msgId;
}

uint16_t MQTTSnPublish::getMsgId(){
    return _msgId;
}


void MQTTSnPublish::setData(uint8_t* data, uint8_t len){
    setMessageLength(7 + len);
    allocate();
    memcpy(getBodyPtr() + 5, data, len);
    setTopicId(_topicId);
    setMsgId(_msgId);
    setFlags(_flags);
}

uint8_t*  MQTTSnPublish::getData(){
    return (uint8_t*)(getBodyPtr() + 5);
}

uint16_t  MQTTSnPublish::getDataLength(){
	return getBodyLength() -5;
}

/*
void MQTTSnPublish::setFrame(uint8_t* data, uint8_t len){
    setMessageLength(len + MQTTSN_HEADER_SIZE);
    allocate();
    memcpy(getBodyPtr(), data, len);
    _topicId = getUint16(data + 1);
    _msgId = getUint16(data + 3);
    _flags = *data;
}


void MQTTSnPublish::setFrame(NWResponse* resp){
    setFrame(resp->getPayloadPtr() + MQTTSN_HEADER_SIZE, resp->getPayloadPtr()[0] - MQTTSN_HEADER_SIZE);
}
*/
void MQTTSnPublish::absorb(NWResponse* src){
	MQTTSnMessage::absorb(src);
}

void MQTTSnPublish::absorb(MQTTSnMessage* src){
	_msgId = getUint16((uint8_t*)(src->getBodyPtr() + 3));
	_flags = src->getBodyPtr()[0];
	_topicId = getUint16((uint8_t*)(src->getBodyPtr() + 1));
	MQTTSnMessage::absorb(src);
}

/*=====================================
         Class MQTTSnPubAck
 ======================================*/
MQTTSnPubAck::MQTTSnPubAck(){
	_topicId = 0;
	_msgId = 0;
	_returnCode = 0;
    setMessageLength(7);
    setType(MQTTSN_TYPE_PUBACK);
    allocate();
}
MQTTSnPubAck::~MQTTSnPubAck(){

}
void MQTTSnPubAck::setTopicId(uint16_t topicId){
    setUint16((uint8_t*)getBodyPtr(), topicId);
}
uint16_t MQTTSnPubAck::getTopicId(){
    return getUint16((unsigned char*)getBodyPtr());
}
void MQTTSnPubAck::setMsgId(uint16_t msgId){
    setUint16(getBodyPtr()+ 2,msgId);
}
uint16_t MQTTSnPubAck::getMsgId(){
    return getUint16((unsigned char*)getBodyPtr()+ 2);
}
void MQTTSnPubAck::setReturnCode(uint8_t rc){
    getBodyPtr()[4] = rc;
}
uint8_t MQTTSnPubAck::getReturnCode(){
    return (uint8_t)getBodyPtr()[4];
}

void MQTTSnPubAck::absorb(MQTTSnMessage* src){
	MQTTSnMessage::absorb(src);
}

void MQTTSnPubAck::absorb(NWResponse* resp){
	MQTTSnMessage::absorb(resp);
}

/*=====================================
         Class MQTTSnPubRec
 ======================================*/
MQTTSnPubRec::MQTTSnPubRec(){
	_msgId = 0;
    setMessageLength(4);
    setType(MQTTSN_TYPE_PUBREC);
    allocate();
}

MQTTSnPubRec::~MQTTSnPubRec(){

}

void MQTTSnPubRec::setMsgId(uint16_t msgId){
    setUint16(getBodyPtr(),msgId);
}

uint16_t MQTTSnPubRec::getMsgId(){
    return getUint16((unsigned char*)getBodyPtr());
}
/*
void MQTTSnPubRec::absorb(NWResponse* resp){
	MQTTSnMessage::absorb(resp);
}*/

/*=====================================
         Class MQTTSnPubRel
 ======================================*/
MQTTSnPubRel::MQTTSnPubRel(){
	_msgId = 0;
    setMessageLength(4);
    setType(MQTTSN_TYPE_PUBREL);
    allocate();
}

MQTTSnPubRel::~MQTTSnPubRel(){

}

/*=====================================
         Class MQTTSnPubComp
 ======================================*/
MQTTSnPubComp::MQTTSnPubComp(){
	_msgId = 0;
    setMessageLength(4);
    setType(MQTTSN_TYPE_PUBCOMP);
    allocate();
}

MQTTSnPubComp::~MQTTSnPubComp(){

}

 /*=====================================
         Class MQTTSnSubscribe
  ======================================*/
MQTTSnSubscribe::MQTTSnSubscribe(){
    setMessageLength(5);
    setType(MQTTSN_TYPE_SUBSCRIBE);
    allocate();
    _topicId = 0;
    _msgId = 0;
    _flags = 0;
}

MQTTSnSubscribe::~MQTTSnSubscribe(){

}

void MQTTSnSubscribe::setFlags(uint8_t flags){
    _flags = flags & 0xe3;
    if (_message){
		  getBodyPtr()[0] = _flags;
    }
}

uint8_t MQTTSnSubscribe::getFlags(){
    return _flags;
}

uint8_t MQTTSnSubscribe::getQos(){
    return ((_flags >> 5) & 0x03);
}

void MQTTSnSubscribe::setQos(uint8_t qos){
	_flags &= 0x9f;
	if(qos == 1){
		_flags |= MQTTSN_FLAG_QOS_1;
	}
	setFlags(_flags);
}

void MQTTSnSubscribe::setTopicId(uint16_t predefinedId){
    setMessageLength(7);
    allocate();
    setMsgId(_msgId);
    setUint16((uint8_t*)(getBodyPtr() + 3), predefinedId);
    setFlags(_flags | MQTTSN_TOPIC_TYPE_PREDEFINED);
    _topicId = predefinedId;
}

uint16_t MQTTSnSubscribe::getTopicId(){
    if (_message){
        _topicId = getUint16(getBodyPtr() +3);
    }
    return _topicId;
}
void MQTTSnSubscribe::setMsgId(uint16_t msgId){
    _msgId = msgId;
    if (_message){
       setUint16((uint8_t*)(getBodyPtr() + 1), msgId);
    }
}

uint16_t MQTTSnSubscribe::getMsgId(){
    if (_message){
        _msgId = getUint16(getBodyPtr() + 1);
    }
    return _msgId;
}
void MQTTSnSubscribe::setTopicName(string* data){
    setMessageLength(5 + data->size());
    allocate();
    data->copy((char*)getBodyPtr() + 3, data->size(),0);
    setMsgId(_msgId);
    setFlags(_flags | MQTTSN_TOPIC_TYPE_NORMAL);
    _topicName = *data;
}


string*  MQTTSnSubscribe::getTopicName(){
    return &_topicName;
}

/*
void MQTTSnSubscribe::setFrame(uint8_t* data, uint8_t len){
    setMessageLength(len + MQTTSN_HEADER_SIZE);
    allocate();
    memcpy(getBodyPtr(), data, len);
    _msgId = getUint16(data + 1);
    _flags = *data;
    if ((_flags & MQTTSN_TOPIC_TYPE) == MQTTSN_TOPIC_TYPE_NORMAL){
        _topicId = 0;
        _topicName = string((char*)data + 3, len);
    }else{
        _topicId = getUint16(data + 3);
    }
}

void MQTTSnSubscribe::setFrame(NWResponse* resp){
    setFrame(resp->getPayloadPtr() + MQTTSN_HEADER_SIZE, resp->getPayloadPtr()[0] - MQTTSN_HEADER_SIZE);
}
*/
void MQTTSnSubscribe::absorb(NWResponse* src){
	MQTTSnMessage::absorb(src);
}

void MQTTSnSubscribe::absorb(MQTTSnMessage* src){
	_msgId = getUint16((uint8_t*)(src->getBodyPtr() +1));
	_flags = src->getBodyPtr()[0];
	if((_flags & MQTTSN_TOPIC_TYPE) == MQTTSN_TOPIC_TYPE_SHORT ){
		_topicName = string((char*)src->getBodyPtr() + 3, src->getMessageLength() - 5);
	}else if((_flags & MQTTSN_TOPIC_TYPE) == MQTTSN_TOPIC_TYPE_NORMAL){
		_topicName = string((char*)src->getBodyPtr() + 3, src->getMessageLength() - 5);
	}else if((_flags & MQTTSN_TOPIC_TYPE) == MQTTSN_TOPIC_TYPE_PREDEFINED){
		 _topicId = getUint16(getBodyPtr() +3);
	}
	MQTTSnMessage::absorb(src);
}
/*=====================================
         Class MQTTSnSubAck
  ======================================*/
MQTTSnSubAck::MQTTSnSubAck(){
    setMessageLength(8);
    setType(MQTTSN_TYPE_SUBACK);
    _flags = 0;
    _topicId = 0;
    _msgId = 0;
    _returnCode = 0;
    allocate();
}

MQTTSnSubAck::~MQTTSnSubAck(){

}

void MQTTSnSubAck::setFlags(uint8_t flags){
    _flags = flags & 0x60;
    getBodyPtr()[0] = _flags;
}

void MQTTSnSubAck::setQos(uint8_t qos){
	_flags &= 0x9f;
	if(qos == 1){
		_flags |= MQTTSN_FLAG_QOS_1;
	}
	setFlags(_flags);
}

uint8_t MQTTSnSubAck::getFlags(){
    return _flags;
}

uint8_t MQTTSnSubAck::getQos(){
    return (_flags >> 5) & 0x03;
}

void MQTTSnSubAck::setTopicId(uint16_t id){
    _topicId = id;
    setUint16(getBodyPtr() + 1, id);
}

uint16_t MQTTSnSubAck::getTopicId(){
    return _topicId;
}
void MQTTSnSubAck::setMsgId(uint16_t msgId){
   _msgId = msgId;
   setUint16(getBodyPtr() + 3, msgId);
}

uint16_t MQTTSnSubAck::getMsgId(){
    return _msgId;
}
void MQTTSnSubAck::setReturnCode(uint8_t rc){
    _returnCode = rc;
    getBodyPtr()[5] = rc;
}
uint8_t  MQTTSnSubAck::getReturnCode(){
    return _returnCode;
}

 /*=====================================
         Class MQTTSnUnsubscribe
  ======================================*/
MQTTSnUnsubscribe::MQTTSnUnsubscribe() : MQTTSnSubscribe(){
    setType(MQTTSN_TYPE_UNSUBSCRIBE);
}

MQTTSnUnsubscribe::~MQTTSnUnsubscribe(){

}

void MQTTSnUnsubscribe::setFlags(uint8_t flags){
    if (_message){
		  getBodyPtr()[0] = flags & 0x03;
    }
}

string* MQTTSnUnsubscribe::getTopicName(){
	return &_topicName;
}

uint16_t MQTTSnUnsubscribe::getTopicId(){
	return _topicId;
}

void MQTTSnUnsubscribe::absorb(NWResponse* src){
	MQTTSnMessage::absorb(src);
}

void MQTTSnUnsubscribe::absorb(MQTTSnMessage* src){
	MQTTSnSubscribe::absorb(src);
}

/*=====================================
         Class MQTTSnUnSubAck
  ======================================*/
MQTTSnUnsubAck::MQTTSnUnsubAck(){
	_msgId = 0;
    setMessageLength(4);
    setType(MQTTSN_TYPE_UNSUBACK);
    allocate();
}

MQTTSnUnsubAck::~MQTTSnUnsubAck(){

}

void MQTTSnUnsubAck::setMsgId(uint16_t msgId){
    setUint16((uint8_t*)getBodyPtr(), msgId);
}

uint16_t MQTTSnUnsubAck::getMsgId(){
    return getUint16(getBodyPtr());
}

/*=====================================
        Class MQTTSnPingReq
 ======================================*/
MQTTSnPingReq::MQTTSnPingReq(){
	setMessageLength(2);
	setType(MQTTSN_TYPE_PINGREQ);
	allocate();
}

MQTTSnPingReq::MQTTSnPingReq(string* id){
  setMessageLength(id->size() + 2);
  setType(MQTTSN_TYPE_PINGREQ);
  allocate();
  id->copy((char*)getBodyPtr(),id->size(),0);

}
MQTTSnPingReq::~MQTTSnPingReq(){

}

char* MQTTSnPingReq::getClientId(){
    return (char*)getBodyPtr();
}

void MQTTSnPingReq::absorb(NWResponse* src){
	MQTTSnMessage::absorb(src);
}

/*=====================================
        Class MQTTSnPingResp
 ======================================*/
MQTTSnPingResp::MQTTSnPingResp(){
    setMessageLength(2);
    setType(MQTTSN_TYPE_PINGRESP);
    allocate();
}
MQTTSnPingResp::~MQTTSnPingResp(){

}

 /*=====================================
         Class MQTTSnDisconnect
  ======================================*/
MQTTSnDisconnect::MQTTSnDisconnect(){
    setMessageLength(4);
    setType(MQTTSN_TYPE_DISCONNECT);
    allocate();

}
MQTTSnDisconnect::~MQTTSnDisconnect(){

}
void MQTTSnDisconnect::setDuration(uint16_t duration){
    setUint16((uint8_t*)getBodyPtr(), duration);
}
uint16_t MQTTSnDisconnect::getDuration(){
    return getUint16((uint8_t*)getBodyPtr());
}

void MQTTSnDisconnect::absorb(NWResponse* src){
	MQTTSnMessage::absorb(src);
}

void MQTTSnDisconnect::absorb(MQTTSnMessage* src){
	MQTTSnMessage::absorb(src);
}

/*=====================================
        Class RemaingLength
 ======================================*/
RemainingLength::RemainingLength(){
	_size = _digit[0] = _digit[1] = _digit[2] = _digit[3] = 0;
}

RemainingLength::~RemainingLength(){

}

void RemainingLength::encode(uint16_t len){
	_size = 0;
	do{
		uint8_t digit = len % 128;
		len = len / 128;
		if(len > 0){
			digit |= 0x80;
		}
		_digit[_size++] = digit;
	}while(len > 0);
}

uint16_t RemainingLength::decode(){
	uint16_t multiplier = 1;
	uint16_t value = 0;
	uint8_t digit;
	uint8_t pos = 0;
	do{
		digit = _digit[pos++];
		value += (digit & 0x7f) * multiplier;
		multiplier *= 128;
	}while((digit & 0x80) != 0);
	return value;
}

uint16_t RemainingLength::serialize(uint8_t* pos){
	memcpy(pos, _digit, _size);
	return decode();
}

void RemainingLength::deserialize(uint8_t* pos){
	uint8_t i = 0;
	_size = 0;
	do{
		_digit[i++] = *pos;
		_size++;
	}while((*pos++ & 0x80) != 0 );
}

uint8_t RemainingLength::getSize(){
	return _size;
}

/*=====================================
         Class MQTTMessage
  ======================================*/
MQTTMessage::MQTTMessage(){
	_type = 0;
	_flags = 0;
	_messageId = 0;
	_remainLength = 0;
	_payload = 0;
	_userName = string("");
	_password = string("");
	_willTopic = string("");
	_willMessage = string("");
	_clientId = string("");
	_topic = string("");
}

MQTTMessage::~MQTTMessage(){
	if(_payload){
		free(_payload);
	}
}

uint8_t MQTTMessage::getType(){
	return _type;
}

uint8_t MQTTMessage::getQos(){
	return (_flags & 0x06) >> 1;
}

uint16_t MQTTMessage::getRemainLength(){
	return _remainLength;
}

uint8_t MQTTMessage::getRemainLengthSize(){
	RemainingLength remLen;
	remLen.encode(_remainLength);
	return remLen.getSize();
}

bool MQTTMessage::isDup(){
	return _flags & 0x80;
}

bool MQTTMessage::isRetain(){
	return _flags & 0x01;
}

uint16_t MQTTMessage::serialize(uint8_t* buf){
	RemainingLength remLen;
	remLen.encode(_remainLength);

	*buf++ = _type | _flags;
	remLen.serialize(buf);
	buf += remLen.getSize();

	if(remLen.decode() > 0){
		setUint16(buf, _messageId);
	}
	return 1 + remLen.getSize() + remLen.decode();
}

bool MQTTMessage::deserialize(uint8_t* buf){
	RemainingLength remLen;

	_type = *buf & 0xf0;
	_flags = *buf & 0x0f;
	remLen.deserialize(buf + 1);
	_remainLength = remLen.decode();

	switch(_type){
		case MQTT_TYPE_PINGRESP:
			return true;
			break;
		case MQTT_TYPE_UNSUBACK:
		case MQTT_TYPE_PUBACK:
		case MQTT_TYPE_PUBREC:
		case MQTT_TYPE_PUBREL:
		case MQTT_TYPE_PUBCOMP:
			_messageId = getUint16(buf + 2);
			break;
		default:
			return false;
	}
	return true;
}

void MQTTMessage::absorb(MQTTMessage* msg){
	_type = msg->_type;
	_flags = msg->_flags;
	_messageId = msg->_messageId;
	_remainLength = msg->_remainLength;
}

void MQTTMessage::setType(uint8_t type){
	_type = type;
}
void MQTTMessage::setQos(uint8_t qos){
	_flags &= 0xf9;
	if(qos == 1){
		_flags |= 0x02;
	}else if(qos == 2){
		_flags |= 0x04;
	}
}

void MQTTMessage::setDup(){
	_flags |= 0x08;
}

void MQTTMessage::setRetain(){
	_flags |= 0x01;
}

/*=====================================
         Class MQTTDisconnect
  ======================================*/
MQTTDisconnect::MQTTDisconnect(){
	_type = MQTT_TYPE_DISCONNECT;
}

MQTTDisconnect::~MQTTDisconnect(){

}

/*=====================================
         Class MQTTPingReq
  ======================================*/
MQTTPingReq::MQTTPingReq(){
	_type = MQTT_TYPE_PINGREQ;
}

MQTTPingReq::~MQTTPingReq(){

}

/*=====================================
         Class MQTTPingResp
  ======================================*/
MQTTPingResp::MQTTPingResp(){
	_type = MQTT_TYPE_PINGRESP;
}

MQTTPingResp::~MQTTPingResp(){

}

/*=====================================
         Class MQTTConnAck
  ======================================*/
MQTTConnAck::MQTTConnAck(){
	_type = MQTT_TYPE_CONNACK;
	_remainLength = 2;
	_returnCd = 0;
}

MQTTConnAck::~MQTTConnAck(){

}


uint8_t MQTTConnAck::getReturnCd(){
	return _returnCd;
}

bool MQTTConnAck::deserialize(uint8_t* buf){
	MQTTMessage::deserialize(buf);

	if(_type != MQTT_TYPE_CONNACK){
		return false;
	}
	buf += 3;
	_returnCd = *buf;
	return true;
}

/*=====================================
         Class MQTTUnsubAck
  ======================================*/
MQTTUnsubAck::MQTTUnsubAck(){
	_type = MQTT_TYPE_CONNACK;
	_remainLength = 2;
}

MQTTUnsubAck::~MQTTUnsubAck(){

}

void MQTTUnsubAck::setMessageId(uint16_t id){
	_messageId = id;
}

uint16_t MQTTUnsubAck::getMessageId(){
	return _messageId;
}

/*=====================================
         Class MQTTPubAck
  ======================================*/
MQTTPubAck::MQTTPubAck(){
	_type = MQTT_TYPE_PUBACK;
	_remainLength = 2;
}

MQTTPubAck::~MQTTPubAck(){

}

void MQTTPubAck::setMessageId(uint16_t id){
	_messageId = id;
}

uint16_t MQTTPubAck::getMessageId(){
	return _messageId;
}

/*=====================================
         Class MQTTPubRec
  ======================================*/
MQTTPubRec::MQTTPubRec(){
	_type = MQTT_TYPE_PUBREC;
	_remainLength = 2;
}

MQTTPubRec::~MQTTPubRec(){

}
void MQTTPubRec::setMessageId(uint16_t id){
	_messageId = id;
}

uint16_t MQTTPubRec::getMessageId(){
	return _messageId;
}

/*=====================================
         Class MQTTPubRel
  ======================================*/
MQTTPubRel::MQTTPubRel(){
	_type = MQTT_TYPE_PUBREL;
	_remainLength = 2;
}

MQTTPubRel::~MQTTPubRel(){

}

/*=====================================
         Class MQTTPubComp
  ======================================*/
MQTTPubComp::MQTTPubComp(){
	_type = MQTT_TYPE_PUBCOMP;
	_remainLength = 2;
}

MQTTPubComp::~MQTTPubComp(){

}

/*=====================================
         Class MQTTSubAck
  ======================================*/
MQTTSubAck::MQTTSubAck(){
	_type = MQTT_TYPE_SUBACK;
	_remainLength = 3;
	_qos = 0;
}

MQTTSubAck::~MQTTSubAck(){

}

void MQTTSubAck::setMessageId(uint16_t id){
	_messageId = id;
}

uint16_t MQTTSubAck::getMessageId(){
	return _messageId;
}

uint8_t MQTTSubAck::getGrantedQos(){
	return _qos;
}

void MQTTSubAck::setGrantedQos0(){
	_qos = 0;
}

void MQTTSubAck::setGrantedQos1(){
	_qos = 1;
}

bool MQTTSubAck::deserialize(uint8_t* buf){
	RemainingLength remLen;
	remLen.encode(_remainLength);

	_type = *buf & 0xf0;
	if(_type != MQTT_TYPE_SUBACK){
		return false;
	}
	_flags = *buf++ & 0x0f;
	remLen.deserialize(buf);
	_messageId = getUint16(buf + remLen.getSize());
	_qos = *(buf + remLen.getSize() + 2);
	return true;
}

/*=====================================
         Class MQTTUnsubscribe
  ======================================*/
MQTTUnsubscribe::MQTTUnsubscribe(){
	_type = MQTT_TYPE_UNSUBSCRIBE;
	_remainLength = 0;
}

MQTTUnsubscribe::~MQTTUnsubscribe(){

}

void MQTTUnsubscribe::setMessageId(uint16_t id){
	_messageId = id;
}

uint16_t MQTTUnsubscribe::getMessageId(){
	return _messageId;
}

void MQTTUnsubscribe::setTopicName(string* topic){
	_topic = *topic;
}

uint16_t MQTTUnsubscribe::serialize(uint8_t* buf){
	RemainingLength remLen;
	_remainLength = _topic.size() + 4;
	remLen.encode(_remainLength);

	*buf++ = _type | 0x02;
	remLen.serialize(buf);
	buf += remLen.getSize();
	setUint16(buf, _messageId);
	buf += 2;
	utfSerialize(buf,_topic);
	return 1 + remLen.getSize() + remLen.decode();
}


/*=====================================
         Class MQTTSubscribe
  ======================================*/
MQTTSubscribe::MQTTSubscribe(){
	_type = MQTT_TYPE_SUBSCRIBE;
	_remainLength = 2;
	_qos = 0;
	setQos(1);
}

MQTTSubscribe::~MQTTSubscribe(){

}

void MQTTSubscribe::setMessageId(uint16_t id){
	_messageId = id;
}

uint16_t MQTTSubscribe::getMessageId(){
	return _messageId;
}

void MQTTSubscribe::setTopic(string* topic, uint8_t qos){
	_topic = *topic;
	_qos = qos;
}

uint16_t MQTTSubscribe::serialize(uint8_t* buf){
	RemainingLength remLen;
	_remainLength = _topic.size() + 5; //length:2, Topic:n, QoS:1
	remLen.encode(_remainLength);

	*buf++ = _type | 0x02;   //SUBSCRIBE QoS1
	remLen.serialize(buf);
	buf += remLen.getSize();
	setUint16(buf, _messageId);
	buf += 2;
	utfSerialize(buf,_topic);
	buf += 2 + _topic.size();
	*buf = _qos;
	return 1 + remLen.getSize() + remLen.decode();
}
/*=====================================
         Class MQTTConnect
  ======================================*/
MQTTConnect::MQTTConnect() : MQTTMessage(){
	_type = MQTT_TYPE_CONNECT;
	_connectFlags = 0;;
	_keepAliveTime = 0;
	_userName = string("");
	_password = string("");
	_willTopic = string("");
	_willMessage = string("");
	_clientId = string("");
	_protocol = 0;
}


MQTTConnect::~MQTTConnect(){

}

void MQTTConnect::setKeepAliveTime(uint16_t sec){
	_keepAliveTime = sec;
}

void MQTTConnect::setUserName(string* userName){
	_userName = *userName;
	_connectFlags &= 0x7e;
	_connectFlags |= 0x80;
}

void MQTTConnect::setPassword(string* pw){
	_password = *pw;
	_connectFlags &= 0xbe;
	_connectFlags |= 0x40;
}

void MQTTConnect::setWillMessage(string* wm){
	_willMessage = *wm;
	_connectFlags &= 0xfa;
	_connectFlags |= 0x04;
}

void MQTTConnect::setWillTopic(string* tp){
	_willTopic = *tp;
	_connectFlags &= 0xfa;
	_connectFlags |= 0x04;
}

void MQTTConnect::setWillQos(uint8_t qos){
	_connectFlags &= 0xe6;
	_connectFlags |= (qos << 3);
}

void MQTTConnect::setProtocol(uint8_t protocol){
	_protocol = protocol;
}

void MQTTConnect::setClientId(string* cid){
	_clientId = *cid;
}

void MQTTConnect::setCleanSessionFlg(){
	_connectFlags &= 0xfc;
	_connectFlags |= 0x02;
}


uint16_t MQTTConnect::serialize(uint8_t* buf){
	uint16_t len;
	string protocol;
	if(_protocol == MQTT_PROTOCOL_VER4){
		protocol = MQTT_PROTOCOL_NAME4;
		len = 10;
	}else{
		protocol = MQTT_PROTOCOL_NAME3;
		len = 12;
	}

	len += _clientId.size() + 2;

	if(_connectFlags & 0x0c){  // Will Topic & Message
		len += _willTopic.size() + 2;
		len += _willMessage.size() + 2;
	}
	if(_connectFlags & 0x80){  // User Name
		len += _userName.size() + 2;
	}
	if(_connectFlags & 0x40){  // Password
		len += _password.size() + 2;
	}
	RemainingLength remLen;
	remLen.encode(len);
	_remainLength = len;

	*buf++ = _type | _flags;
	remLen.serialize(buf);
	buf += remLen.getSize();

	utfSerialize(buf, protocol);
	buf += protocol.size() + 2;
	*buf++ = _protocol;
	*buf++ = _connectFlags;
	setUint16(buf++, _keepAliveTime);

	utfSerialize(++buf,_clientId); // copy clienId
	buf += _clientId.size() + 2;

	if(_connectFlags & 0x0c){  // Will Topic & Message
		utfSerialize(buf,_willTopic);
		buf += _willTopic.size() + 2;
		utfSerialize(buf,_willMessage);
		buf += _willMessage.size() + 2;
	}
	if(_connectFlags & 0x80){  // User Name
		utfSerialize(buf,_userName);
		buf += _userName.size() + 2;
	}
	if(_connectFlags & 0x40){  // Password
		utfSerialize(buf,_password);
	}

	return 1 + remLen.getSize() + remLen.decode();

}

/*=====================================
         Class MQTTPublish
  ======================================*/
MQTTPublish::MQTTPublish(){
	_type = MQTT_TYPE_PUBLISH;
	//_topic = string("");
	//_payload = 0;
	_len = 0;
	_messageId = 0;
}

MQTTPublish::~MQTTPublish(){

}

void MQTTPublish::setMessageId(uint16_t id){
	_messageId = id;
}

uint16_t MQTTPublish::getMessageId(){
	return _messageId;
}

string* MQTTPublish::getTopic(){
	return &_topic;
}

uint8_t* MQTTPublish::getPayload(){
	return _payload;
}


uint8_t  MQTTPublish::getPayloadLength(){
	return _len;
}

void MQTTPublish::setTopic(string* topic){
	_topic = *topic;
}

void MQTTPublish::setPayload(uint8_t* payload, uint8_t length){
	if(_payload){
		_remainLength -= _len;
		free(_payload);
	}
	_payload = mqcalloc(length);
	memcpy(_payload, payload, length);
	_len = length;
	_remainLength += length;
}

uint16_t MQTTPublish::serialize(uint8_t* buf){
	RemainingLength remLen;
	_remainLength = _topic.size() + 2 + _len;
	if(getQos()){
		_remainLength += 2;
	}
	remLen.encode(_remainLength);

	*buf++ = (_type & 0xf0) | (_flags & 0x0f);
	remLen.serialize(buf);
	buf += remLen.getSize();
	utfSerialize(buf, _topic);
	buf += _topic.size() + 2;
	if(getQos()){
		setUint16(buf, _messageId);
		buf += 2;
	}
	memcpy(buf, _payload, _len);

	return 1 + remLen.getSize() + remLen.decode();
}

bool MQTTPublish::deserialize(uint8_t* buf){
	uint8_t pos = 2;

	_type = *buf & 0xf0;
	if(_type != MQTT_TYPE_PUBLISH){
		return false;
	}
	RemainingLength remLen;

	_flags = *buf & 0x0f;
	remLen.deserialize(buf + 1);
	_remainLength = remLen.decode();

	buf += 1 + remLen.getSize();
	_topic = string((char*)buf + 2, getUint16(buf));

	buf += _topic.size() + 2;
	if(getQos()){
		_messageId = getUint16(buf);
		buf += 2;
		pos += 2;
	}
	_len = _remainLength - _topic.size() - pos;
	_payload = mqcalloc(_len);
	memcpy(_payload, buf, _len);
	return true;
}


