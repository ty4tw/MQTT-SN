/*
 * mqttsn.cpp
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


#ifndef ARDUINO
  	  #include "MQTTSN_Application.h"
#else
  	  #include <MQTTSN_Application.h>
#endif


#ifdef ARDUINO
  #include <mqttsn.h>
  #include <Network.h>

  #if defined(MQTTSN_DEBUG) || defined(ZBEE_DEBUG)
    #include <SoftwareSerial.h>
    extern SoftwareSerial debug;
  #endif

#endif  /* ARDUINO */

#ifdef MBED
  #include "mbed.h"
  #include "Network.h"
  #include "mqttsn.h"
#endif  /* MBED */

#ifdef LINUX
  #include "Network.h"
  #include "mqttsn.h"
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

extern uint16_t getUint16(uint8_t* pos);
extern uint32_t getUint32(uint8_t* pos);
extern float    getFloat32(uint8_t* pos);

extern void setUint16(uint8_t* pos, uint16_t val);
extern void setUint32(uint8_t* pos, uint32_t val);
extern void setFloat32(uint8_t* pos, float val);


/*=====================================
        Class MQString
  =====================================*/
MQString::MQString(){
    //_length = 0;
    _constStr = 0;
    _str = 0;
}

MQString::MQString(const char* str){
    if (strlen(str)){
        _constStr = str;
    }else{
        _constStr = 0;
    }
    _str = 0;
}

MQString::MQString(char* str){
    if (strlen(str)){
        _str = str;
    }else{
        _str = 0;
    }
    _constStr = 0;
}

MQString::~MQString(){
    if (_str){
        free(_str);
    }
}

uint16_t MQString::getCharLength(){
    if(_str){
    	return strlen(_str);
    }
    return strlen(_constStr);
}

int MQString::comp(MQString* str){
    if (_str){
    	if(str->getStr()){
    		return (strcmp(_str, str->getStr()));
    	}else if(str->getConstStr()){
    		return (strcmp(_str, str->getConstStr()));
    	}
    }else if(_constStr){
    	if(str->getStr()){
    		return (strcmp(_constStr, str->getStr()));
    	}else if(str->getConstStr()){
    		return (strcmp(_constStr, str->getConstStr()));
    	}
    }
    return 1;
}


int MQString::comp(const char* str){
    if(_str){
        return (strcmp(_str, str));
    }else if (_constStr){
        return (strcmp(_constStr, str));
    }
    return 1;
}


int MQString::ncomp(MQString* str, uint16_t len){
    if (_str){
        return strncmp(_str, str->getStr(), (long)len);
    }else if (_constStr){
        return strncmp(_constStr, str->getConstStr(), (long)len);
    }
    return 1;
}


bool MQString::operator==(MQString &str){
	return (comp(&str) == 0);
}

bool MQString::operator!=(MQString &str){
	return (comp(&str) != 0);
}


void MQString::copy(MQString* str){
    if (str->isConst()){
        _constStr = getConstStr();
    }else{
        _str = str->getStr();
    }
}

MQString* MQString::create(){
    char* newStr = (char*)calloc(getCharLength()+ 1, sizeof(char));
    memcpy(newStr, getConstStr(), getCharLength());
    MQString* newPtr = new MQString((const char*)newStr);
    return newPtr;
}

void MQString::copy(const char* str){
    _constStr = str;
    _str = 0;
    freeStr();
}

void MQString::copy(char* str){
    freeStr();
    _str = (char*)calloc(strlen(str) + 1, sizeof(char));
    _constStr = 0;
    strcpy(_str, str);
}

void MQString::copy(uint8_t* str, uint8_t len){
    freeStr();
    _str = (char*)calloc(len + 1, sizeof(char));
    _constStr = 0;
    memcpy(_str, str, len);
}

void MQString::writeBuf(uint8_t* buf){
    if (_str){
        memcpy(buf, _str, strlen(_str));
    }else if (_constStr){
        memcpy(buf, _constStr, strlen(_constStr));
    }
}

void MQString::readBuf(uint8_t* buf){
    _str = 0;
    _constStr = (const char*)buf;
}

uint8_t MQString::getChar(uint8_t index){
    if (_str){
        return (index < strlen(_str) ? _str[index]: 0);
    }else if (_constStr){
        return (index < strlen(_constStr) ? _constStr[index]: 0);
    }
    return 0;
}

char* MQString::getStr(){
    return (_str ? _str : 0);
}

const char* MQString::getConstStr(){
    return (_constStr ? _constStr : 0);
}


bool MQString::isConst(){
  return (_constStr ? true : false);
}

void MQString::freeStr(){
    if (_str){
        free(_str);
        _str = 0;
    }
}


/*=====================================
        Class MqttsnMessage
 ======================================*/
MqttsnMessage::MqttsnMessage(){
    _msgBuff = 0;    _flags = 0;
    _flags = 0;
    _length = 0;
    _status = 0;
    _type = 0;
}
MqttsnMessage::~MqttsnMessage(){
    if (_msgBuff != 0){
        delete(_msgBuff);
    }
}

void MqttsnMessage::reset(){
    _msgBuff = 0;
    _length = 0;
    _status = 0;
    _type = 0;
}

void MqttsnMessage::setType(uint8_t type){
    _type = type;
    if ( _msgBuff != 0){
    	if(_length > 255){
    		_msgBuff[3] = type;
    	}else{
    		_msgBuff[1] = type;
    	}
    }
}

void MqttsnMessage::setFlags(uint8_t flg){
    _flags = flg;
    if ( _msgBuff != 0){
    	if(_length > 255){
    		_msgBuff[4] = flg;
    	}else{
    		_msgBuff[2] = flg;
    	}
    }
}

void MqttsnMessage::setLength(uint16_t length){
    _length = length;
    if ( _msgBuff != 0){
    	if(_length > 255){
    		_msgBuff[0] = 0x01;
			setUint16(_msgBuff + 1, _length);
		}else{
			 _msgBuff[0] = length;
		}
    }
}

void MqttsnMessage::setQos(uint8_t qos){
	_flags &= 0x9f;
	if(qos == QOS1){
		_flags |= 0x20;
	}else if(qos == QOS2){
		_flags |= 0x40;
	}
	setFlags(_flags);
}

bool MqttsnMessage::setBody(uint8_t* body){
    if (allocateBody()) {
    	if(_length > 255){
    		_msgBuff[0] = 0x01;
			setUint16(_msgBuff + 1, _length);
			_msgBuff[3] = _type;
			memcpy(_msgBuff + 4, body, _length - 4);
    	}else{
			_msgBuff[0] = _length;
			_msgBuff[1] = _type;
			memcpy(_msgBuff + 2, body, _length - MQTTSN_HEADER_SIZE);
    	}
        return true;
    }else{
        return false;
    }
}

bool MqttsnMessage::allocateBody(){
    if ( _length > 0 ) {
        if (_msgBuff){
              free(_msgBuff);
        }

        _msgBuff = (uint8_t*)calloc(_length + 1, sizeof(uint8_t));
        if ( _msgBuff){
        	if(_length > 255){
        		_msgBuff[0] = 0x01;
        		setUint16(_msgBuff + 1, _length);
        		_msgBuff[3] = _type;
                _msgBuff[4] = _flags;
        	}else{
				_msgBuff[0] = _length;
				_msgBuff[1] = _type;
		        _msgBuff[2] = _flags;
        	}
            return true;
        }else{
            return false;
        }
    }else{
        return false;
    }
}

void MqttsnMessage::setDup(){
	if(_msgBuff && (_type == MQTTSN_TYPE_PUBLISH || _type == MQTTSN_TYPE_SUBSCRIBE)){
		_flags |= 0x80;
		getBody()[0] = _flags ;
	}
}

void MqttsnMessage::setStatus(uint8_t stat){
    _status = stat;
}

uint8_t MqttsnMessage::getQos(){
    return (_flags & (MQTTSN_FLAG_QOS_1 | MQTTSN_FLAG_QOS_2)) >> 5;
}

uint8_t MqttsnMessage::getLength(){
    return _length;
}

uint8_t MqttsnMessage::getType(){
    return _type;
}

uint8_t MqttsnMessage::getFlags(){
    return _flags;
}

uint8_t MqttsnMessage::getStatus(){
    return _status;
}

uint8_t* MqttsnMessage::getBody(){
	if(_length > 255){
		return _msgBuff + 4;
	}else{
		return _msgBuff + MQTTSN_HEADER_SIZE;
	}
}

uint16_t MqttsnMessage::getBodyLength(){
    if(_length > 255){
    	return _length - 4;
    }else{
    	return _length - MQTTSN_HEADER_SIZE;
    }
}

uint16_t MqttsnMessage::getFrameLength(){
    return _length;
}

uint8_t* MqttsnMessage::getMsgBuff(){
    return _msgBuff;
}

void MqttsnMessage::setMsgBuff(uint8_t* msgBuff){
    _msgBuff = msgBuff;
}

bool MqttsnMessage::copy(MqttsnMessage* src){
    setLength(src->getLength());
    setType(src->getType());
    setFlags(src->getFlags());
    setStatus(src->getStatus());
    _msgBuff = src->_msgBuff;
    src->setMsgBuff(0);
    if (_msgBuff == 0){
        return false;
    }
    return true;
}

const char* MqttsnMessage::getMsgTypeName(){
	switch(_type){
	case 0x00:
		return "ADVERTIZE";
	case 0x01:
		return "SEARCGW";
	case 0x02:
		return "GWINFO";
	case 0x04:
		return "CONNECT";
	case 0x05:
		return "CONNACK";
	case 0x06:
		return "WILLTOPICREQ";
	case 0x07:
		return "WILLTOPIC";
	case 0x08:
		return "WILLMSGREQ";
	case 0x09:
		return "WILLMSG";
	case 0x0a:
		return "REGISTER";
	case 0x0b:
		return "REGACK";
	case 0x0c:
		return "PUBLISH";
	case 0x0d:
		return "PUBACK";
	case 0x0e:
		return "PUBCOMP";
	case 0x0f:
		return "PUBREC";
	case 0x10:
		return "PUBREL";
	case 0x12:
		return "SUBSCRIBE";
	case 0x13:
		return "SUBACK";
	case 0x14:
		return "UNSUBSCRIBE";
	case 0x15:
		return "UNSUBACK";
	case 0x16:
		return "PINGREQ";
	case 0x17:
		return "PINGRESP";
	case 0x18:
		return "DISCONNECT";
	default:
		return "";
	}
}


/*=====================================
        Class MqttsnAdvrtise
 ======================================*/
MqttsnAdvertise::MqttsnAdvertise():MqttsnMessage(){
    setLength(5);
    setType(MQTTSN_TYPE_ADVERTISE);
    allocateBody();
}

MqttsnAdvertise::~MqttsnAdvertise(){

}

void MqttsnAdvertise::setGwId(uint8_t id){
    getBody()[0] = id;
}

void MqttsnAdvertise::MqttsnAdvertise::setDuration(uint16_t duration){
    uint8_t* pos = getBody() + 1;
    setUint16(pos, duration);
}

uint8_t MqttsnAdvertise::getGwId(){
    return getBody()[0];
}

uint16_t MqttsnAdvertise::getDuration(){
  uint8_t* pos = getBody() + 1;
    return getUint16(pos);
}

/*=====================================
        Class MqttsnSearchgw
 ======================================*/
MqttsnSearchGw::MqttsnSearchGw():MqttsnMessage(){
    setLength(3);
    setType(MQTTSN_TYPE_SEARCHGW);
    allocateBody();
}

MqttsnSearchGw::~MqttsnSearchGw(){

}

void MqttsnSearchGw::setRadius(uint8_t radius){
  getBody()[0] = radius;
}

uint8_t MqttsnSearchGw::getRadius(){
  return getBody()[0];
}

/*=====================================
        Class MqttsnGwinfo
 ======================================*/
MqttsnGwInfo::MqttsnGwInfo():MqttsnMessage(){
  setLength(3);
  setType(MQTTSN_TYPE_GWINFO);
  allocateBody();
}

MqttsnGwInfo::~MqttsnGwInfo(){

}

uint8_t MqttsnGwInfo::getGwId(){
    return getBody()[0];
}

void MqttsnGwInfo::setGwId(uint8_t id){
    getBody()[0] = id;
}

/*=====================================
         Class MqttsnConnect
  ======================================*/
MqttsnConnect::MqttsnConnect(MQString* id):MqttsnMessage(){
    setLength(id->getCharLength() + 6);
    allocateBody();
    setType(MQTTSN_TYPE_CONNECT);
    getBody()[1] = MQTTSN_PROTOCOL_ID;
    id->writeBuf(getBody() + 4);
}

MqttsnConnect::~MqttsnConnect(){

}

void MqttsnConnect::setDuration(uint16_t msec){
    setUint16((uint8_t*)getBody() + 2, msec);
}

uint16_t MqttsnConnect::getDuration(){
    return getUint16((uint8_t*)getBody() + 2);
}

void MqttsnConnect::setClientId(MQString* id){
    id->writeBuf(getBody() + 4);
    setLength(id->getCharLength() + 6);
}

uint8_t* MqttsnConnect::getClientId(){
    return getBody() + 6;
}

void MqttsnConnect::setFrame(uint8_t* data, uint8_t len){
    setLength(len + MQTTSN_HEADER_SIZE);
    allocateBody();
    memcpy(getBody(), data, len);
    getBody()[2] = getUint16(data + 4);
    getBody()[0] = *(data + 2);
}

/*=====================================
        Class MqttsnConnack
 ======================================*/
MqttsnConnack::MqttsnConnack():MqttsnMessage(){
    setLength(3);
    setType(MQTTSN_TYPE_CONNACK);
    allocateBody();
}

MqttsnConnack::~MqttsnConnack(){

}

void MqttsnConnack::setReturnCode(uint8_t rc){
    getBody()[0] = rc;
}

uint8_t MqttsnConnack::getReturnCode(){
    return getBody()[0];
}

/*=====================================
       Class MqttsnWillTopicReq
======================================*/
MqttsnWillTopicReq::MqttsnWillTopicReq():MqttsnMessage(){
    setLength(2);
    setType(MQTTSN_TYPE_WILLTOPICREQ);
    allocateBody();
}

MqttsnWillTopicReq::~MqttsnWillTopicReq(){

}

/*=====================================
         Class MqttsnWillTopic
  ======================================*/
MqttsnWillTopic::MqttsnWillTopic():MqttsnMessage(){
    setLength(3);
    setType(MQTTSN_TYPE_WILLTOPIC);
    allocateBody();
    _flags = 0;
}

MqttsnWillTopic::~MqttsnWillTopic(){

}


void MqttsnWillTopic::setWillTopic(MQString* topic){
    setLength(topic->getCharLength() + 3);
    allocateBody();
    topic->writeBuf(getBody() + 1);
    _msgBuff[2] = _flags;
    _ustring.copy(topic);
}

MQString* MqttsnWillTopic::getWillTopic(){
  if (_msgBuff){
        return &_ustring;
    }else{
        return 0;
    }
}

bool MqttsnWillTopic::isWillRequired(){
    return getBody()[0] && MQTTSN_FLAG_WILL;
}

/*=====================================
         Class MqttsnWillMsgReq
  ======================================*/
MqttsnWillMsgReq::MqttsnWillMsgReq():MqttsnMessage(){
    setLength(2);
    setType(MQTTSN_TYPE_WILLMSGREQ);
    allocateBody();

}

MqttsnWillMsgReq::~MqttsnWillMsgReq(){

}

/*=====================================
         Class MqttsnWillMsg
  ======================================*/
MqttsnWillMsg::MqttsnWillMsg():MqttsnMessage(){
    setLength(2);
    setType(MQTTSN_TYPE_WILLMSG);
    allocateBody();
}

MqttsnWillMsg::~MqttsnWillMsg(){

}

void MqttsnWillMsg::setWillMsg(MQString* msg){
    setLength(2 + msg->getCharLength());
    allocateBody();
    msg->writeBuf(getBody());
}

char* MqttsnWillMsg::getWillMsg(){
    if (_msgBuff){
            return (char*)getBody();
    }else{
            return 0;
    }
}

/*=====================================
         Class MqttsnRegister
  ======================================*/
MqttsnRegister::MqttsnRegister():MqttsnMessage(){
    setLength(6);
    setType(MQTTSN_TYPE_REGISTER);
    allocateBody();
    _topicId = 0;
    _msgId = 0;
}

MqttsnRegister::~MqttsnRegister(){

}

void MqttsnRegister::setTopicId(uint16_t topicId){
  if (_msgBuff){
            setUint16(getBody(), topicId);
    }
    _topicId = topicId;
}
uint16_t MqttsnRegister::getTopicId(){
    return _topicId;
}
void MqttsnRegister::setMsgId(uint16_t msgId){
    if (_msgBuff){
            setUint16(getBody() + 2, msgId);
    }
    _msgId = msgId;
}
uint16_t MqttsnRegister::getMsgId(){
    return _msgId;

}
void MqttsnRegister::setTopicName(MQString* topicName){
    setLength(6 + topicName->getCharLength());
    allocateBody();
    topicName->writeBuf(getBody() + 4);
    setTopicId(_topicId);
    setMsgId(_msgId);
}

void MqttsnRegister::setFrame(uint8_t* data, uint8_t len){
    setLength(len + MQTTSN_HEADER_SIZE);
    allocateBody();
    memcpy(getBody(), data, len);
    _topicId = getUint16(data);
    _msgId = getUint16(data + 2);
    _ustring.readBuf(getBody() + 4);
}

void MqttsnRegister::setFrame(NWResponse* resp){
    setFrame(resp->getPayload() + MQTTSN_HEADER_SIZE, resp->getPayload(0) - MQTTSN_HEADER_SIZE);
}

MQString* MqttsnRegister::getTopicName(){
    return &_ustring;
}

/*=====================================
         Class MqttsnRegAck
  ======================================*/
MqttsnRegAck::MqttsnRegAck():MqttsnMessage(){
    setLength(7);
    setType(MQTTSN_TYPE_REGACK);
    allocateBody();
}
MqttsnRegAck::~MqttsnRegAck(){

}
void MqttsnRegAck::setTopicId(uint16_t topicId){
    setUint16((uint8_t*)getBody(), topicId);
}
uint16_t MqttsnRegAck::getTopicId(){
    return getUint16((unsigned char*)getBody());
}
void MqttsnRegAck::setMsgId(uint16_t msgId){
    setUint16(getBody()+ 2,msgId);
}
uint16_t MqttsnRegAck::getMsgId(){
    return getUint16((unsigned char*)getBody()+ 2);
}
void MqttsnRegAck::setReturnCode(uint8_t rc){
    getBody()[4] = rc;
}
uint8_t MqttsnRegAck::getReturnCode(){
    return (uint8_t)getBody()[4];
}

/*=====================================
         Class MqttsnPublish
  ======================================*/
MqttsnPublish::MqttsnPublish():MqttsnMessage(){
    setLength(7);
    setType(MQTTSN_TYPE_PUBLISH);
    allocateBody();
    _topicId = 0;
    _topic = 0;
    _msgId = 0;
}

MqttsnPublish::~MqttsnPublish(){

}

void MqttsnPublish::setTopicId(uint16_t id){
    setUint16((uint8_t*)(getBody() + 1), id);
    _topicId = id;
}

uint16_t MqttsnPublish::getTopicId(){
    return _topicId;
}

MQString* MqttsnPublish::getTopic(MQString* topic){
	char tp[3];
	tp[0] = (char)*(getBody() + 1);
	tp[1] = (char)*(getBody() + 2);
	tp[2] = 0;
	topic->copy(tp);
    return topic;
}

void MqttsnPublish::setTopic(MQString* topic){
	if(topic->getCharLength() == 2){
		topic->writeBuf((uint8_t*)(getBody() + 1));
		memcpy((void*)&_topicId,(getBody() + 1), 2);
		_flags &= 0xfc;
		_flags |= MQTTSN_TOPIC_TYPE_SHORT;
	}
}

void MqttsnPublish::setMsgId(uint16_t msgId){
    setUint16((uint8_t*)(getBody() + 3), msgId);
    _msgId = msgId;
}

uint16_t MqttsnPublish::getMsgId(){
    return _msgId;
}


void MqttsnPublish::setData(uint8_t* data, uint16_t len){
    setLength(7 + len);
    allocateBody();
    memcpy(getBody() + 5, data, len);
    setTopicId(_topicId);
    setMsgId(_msgId);
}

void MqttsnPublish::setData(MQString* str){
	setLength(7 + str->getCharLength());
	allocateBody();
	setTopicId(_topicId);
	setMsgId(_msgId);
	str->writeBuf(getBody() + 5);
}

uint8_t*  MqttsnPublish::getData(){
    return (uint8_t*)(getBody() + 5);
}

uint16_t  MqttsnPublish::getDataLength(){
    return  this->getBodyLength() -  5;
}

void MqttsnPublish::setFrame(uint8_t* data, uint16_t len){
	if(len > 255){
		setLength(len + MQTTSN_HEADER_SIZE + 2);
	}else{
		setLength(len + MQTTSN_HEADER_SIZE);
	}
    allocateBody();
    memcpy(getBody(), data, len);
    _topicId = getUint16(data + 1);
    _msgId = getUint16(data + 3);
    _flags = *data;
}

void MqttsnPublish::setFrame(NWResponse* resp){
	setFrame(resp->getBody(), resp->getBodyLength());
}

void MqttsnPublish::setPayload(Payload* payload){

}
/*=====================================
         Class MqttsnPubAck
 ======================================*/
MqttsnPubAck::MqttsnPubAck():MqttsnMessage(){
    setLength(7);
    setType(MQTTSN_TYPE_PUBACK);
    allocateBody();
}
MqttsnPubAck::~MqttsnPubAck(){

}
void MqttsnPubAck::setTopicId(uint16_t topicId){
    setUint16((uint8_t*)getBody(), topicId);
}
uint16_t MqttsnPubAck::getTopicId(){
    return getUint16((unsigned char*)getBody());
}
void MqttsnPubAck::setMsgId(uint16_t msgId){
    setUint16(getBody()+ 2,msgId);
}
uint16_t MqttsnPubAck::getMsgId(){
    return getUint16((unsigned char*)getBody()+ 2);
}
void MqttsnPubAck::setReturnCode(uint8_t rc){
    getBody()[4] = rc;
}
uint8_t MqttsnPubAck::getReturnCode(){
    return (uint8_t)getBody()[4];
}

/*=====================================
         Class MqttsnPubRec
 ======================================*/
MqttsnPubRec::MqttsnPubRec(){
    setLength(4);
    setType(MQTTSN_TYPE_PUBREC);
    allocateBody();
}
MqttsnPubRec::~MqttsnPubRec(){

}

uint16_t MqttsnPubRec::getMsgId(){
    return getUint16(getBody());
}

void MqttsnPubRec::setMsgId(uint16_t msgId){
    setUint16(getBody(),msgId);
}

/*=====================================
         Class MqttsnPubRel
 ======================================*/
MqttsnPubRel::MqttsnPubRel(){
    setLength(4);
    setType(MQTTSN_TYPE_PUBREL);
    allocateBody();
}

MqttsnPubRel::~MqttsnPubRel(){

}

/*=====================================
         Class MqttsnPubComp
 ======================================*/
MqttsnPubComp::MqttsnPubComp(){
    setLength(4);
    setType(MQTTSN_TYPE_PUBCOMP);
    allocateBody();
}

MqttsnPubComp::~MqttsnPubComp(){

}

 /*=====================================
         Class MqttsnSubscribe
  ======================================*/
MqttsnSubscribe::MqttsnSubscribe(){
    setLength(5);
    setType(MQTTSN_TYPE_SUBSCRIBE);
    allocateBody();
    _topicId = 0;
    _msgId = 0;
    _flags = 0;
}

MqttsnSubscribe::~MqttsnSubscribe(){

}

void MqttsnSubscribe::setTopicId(uint16_t predefinedId){
    setLength(7);
    allocateBody();
    setMsgId(_msgId);
    setUint16((uint8_t*)(getBody() + 3), predefinedId);
    setFlags(_flags | MQTTSN_TOPIC_TYPE_PREDEFINED);
    _topicId = predefinedId;
}

uint16_t MqttsnSubscribe::getTopicId(){
    if (_msgBuff){
        _topicId = getUint16(getBody() +3);
    }
    return _topicId;
}
void MqttsnSubscribe::setMsgId(uint16_t msgId){
    _msgId = msgId;
    if (_msgBuff){
       setUint16((uint8_t*)(getBody() + 1), msgId);
    }
}

uint16_t MqttsnSubscribe::getMsgId(){
    if (_msgBuff){
        _msgId = getUint16(getBody() + 1);
    }
    return _msgId;
}

void MqttsnSubscribe::setTopicName(MQString* data){
    setLength(5 + data->getCharLength());
    allocateBody();
    data->writeBuf(getBody() + 3);
    setMsgId(_msgId);
    setFlags((_flags & 0xe0) | MQTTSN_TOPIC_TYPE_NORMAL);
    _ustring.copy(data);
}


MQString*  MqttsnSubscribe::getTopicName(){
    return &_ustring;
}

void MqttsnSubscribe::setFrame(uint8_t* data, uint8_t len){
    setLength(len + MQTTSN_HEADER_SIZE);
    allocateBody();
    memcpy(getBody(), data, len);
    _msgId = getUint16(data + 1);
    _flags = *data;
    if ((_flags & MQTTSN_TOPIC_TYPE) == MQTTSN_TOPIC_TYPE_PREDEFINED){
    	_topicId = getUint16(data + 3);
    }else{
    	_topicId = 0;
		_ustring.readBuf(data + 3);
    }
}

void MqttsnSubscribe::setFrame(NWResponse* resp){
	setFrame(resp->getPayload() + MQTTSN_HEADER_SIZE, resp->getPayload(0) - MQTTSN_HEADER_SIZE);
}

/*=====================================
         Class MqttsnSubAck
  ======================================*/
MqttsnSubAck::MqttsnSubAck():MqttsnMessage(){
    setLength(8);
    setType(MQTTSN_TYPE_SUBACK);
    allocateBody();
}

MqttsnSubAck::~MqttsnSubAck(){

}

void MqttsnSubAck::setTopicId(uint16_t id){
    setUint16((uint8_t*)(getBody() + 1), id);
}

uint16_t MqttsnSubAck::getTopicId(){
    return getUint16(getBody() + 1);
}
void MqttsnSubAck::setMsgId(uint16_t msgId){
   setUint16((uint8_t*)(getBody() + 3), msgId);
}

uint16_t MqttsnSubAck::getMsgId(){
    return getUint16(getBody() + 3);
}
void MqttsnSubAck::setReturnCode(uint8_t rc){
    getBody()[6] = rc;
}
uint8_t  MqttsnSubAck::getReturnCode(){
    return getBody()[5];
}


 /*=====================================
         Class MqttsnUnsubscribe
  ======================================*/
MqttsnUnsubscribe::MqttsnUnsubscribe() : MqttsnSubscribe(){
    setType(MQTTSN_TYPE_UNSUBSCRIBE);
}
MqttsnUnsubscribe::~MqttsnUnsubscribe(){

}

/*=====================================
         Class MqttsnUnSubAck
  ======================================*/
MqttsnUnSubAck::MqttsnUnSubAck():MqttsnMessage(){
    setLength(4);
    setType(MQTTSN_TYPE_UNSUBACK);
    allocateBody();
}

MqttsnUnSubAck::~MqttsnUnSubAck(){

}

void MqttsnUnSubAck::setMsgId(uint16_t msgId){
    setUint16((uint8_t*)getBody(), msgId);
}

uint16_t MqttsnUnSubAck::getMsgId(){
    return getUint16(getBody());
}

/*=====================================
        Class MqttsnPingReq
 ======================================*/
MqttsnPingReq::MqttsnPingReq(MQString* id):MqttsnMessage(){
  setLength(id->getCharLength() + 2);
  setType(MQTTSN_TYPE_PINGREQ);
  allocateBody();
  id->writeBuf(getBody());

}
MqttsnPingReq::~MqttsnPingReq(){

}

char* MqttsnPingReq::getClientId(){
    return (char*)getBody();
}

/*=====================================
        Class MqttsnPingResp
 ======================================*/
MqttsnPingResp::MqttsnPingResp():MqttsnMessage(){
    setLength(2);
    setType(MQTTSN_TYPE_PINGRESP);
    allocateBody();
}
MqttsnPingResp::~MqttsnPingResp(){

}

 /*=====================================
         Class MqttsnDisconnect
  ======================================*/
MqttsnDisconnect::MqttsnDisconnect():MqttsnMessage(){
    setLength(4);
    setType(MQTTSN_TYPE_DISCONNECT);
    allocateBody();

}
MqttsnDisconnect::~MqttsnDisconnect(){

}
void MqttsnDisconnect::setDuration(uint16_t duration){
    setUint16((uint8_t*)getBody(), duration);
}
uint16_t MqttsnDisconnect::getDuration(){
    return getUint16((uint8_t*)getBody());
}


/*=====================================
        Class Topic
 ======================================*/
Topic::Topic(){
    _topicStr = 0;
    _callback = 0;
    _topicId = 0;
    _status = 0;
}

Topic::~Topic(){
    if (_topicStr){
        delete _topicStr;
    }
}

uint8_t Topic::getStatus(){
    return _status;
}

uint16_t Topic::getTopicId(){
    return _topicId;
}

MQString* Topic::getTopicName(){
    return _topicStr;
}

uint8_t Topic::getTopicLength(){
    return _topicStr->getCharLength();
}

TopicCallback Topic::getCallback(){
    return _callback;
}

void Topic::setTopicId(uint16_t id){
    _topicId = id;
}

void Topic::setStatus(uint8_t stat){
    _topicId = stat;
}


void Topic::setTopicName(MQString* topic){
    _topicStr = topic;
}

void Topic::setCallback(TopicCallback callback){
    _callback = callback;
}

int Topic::execCallback(MqttsnPublish* msg){
    if(_callback != 0){
        return _callback(msg);
    }
    return 0;
}

void Topic::copy(Topic* src){
    setTopicId(src->getTopicId());
    setStatus(src->getStatus());
    setCallback(src->getCallback());
    setCallback(_callback);
    _topicStr = src->getTopicName();
}

uint8_t Topic::isWildCard(){
    if (getTopicName()->getChar(getTopicName()->getCharLength() - 1) == MQTTSN_TOPIC_SINGLE_WILDCARD){
        return MQTTSN_TOPIC_SINGLE_WILDCARD;
    }else if (getTopicName()->getChar(getTopicName()->getCharLength() - 1) == MQTTSN_TOPIC_MULTI_WILDCARD){
        return MQTTSN_TOPIC_MULTI_WILDCARD;
    }
    return 0;
}

bool Topic::isMatch(Topic* wildCard){
    uint8_t pos = wildCard->getTopicName()->getCharLength() - 1;
    if (wildCard->isWildCard() == MQTTSN_TOPIC_SINGLE_WILDCARD &&
        getTopicName()->ncomp(wildCard->getTopicName(), (long)pos) == 0 ){
        for(; pos < getTopicName()->getCharLength(); pos++){
            if (getTopicName()->getChar(pos) == '/'){
                return false;
            }
        }
        return true;
    }else if(wildCard->isWildCard() == MQTTSN_TOPIC_MULTI_WILDCARD &&
        getTopicName()->ncomp(wildCard->getTopicName(), (long)pos) == 0 ){
        return true;
    }
    return false;
}

/*=====================================
        Class Topics
 ======================================*/
Topics::Topics(){
    _sizeMax = 0;
    _elmCnt = 0;
    _topics = 0;
}

Topics::~Topics() {

}

bool Topics::allocate(uint8_t size){
    _elmCnt = 0;
      _topics = (Topic*)calloc(size, sizeof(Topic));
    if (_topics == 0){
        _sizeMax = 0;
        return false;
    }else{
        _sizeMax = size;
        return true;
    }
}


uint16_t Topics::getTopicId(MQString* topic){
    Topic *p = getTopic(topic);
    if ( p != 0) {
        return p->getTopicId();
    }
    return 0;
}


Topic* Topics::getTopic(MQString* topic) {
    for (int i = 0; i < _elmCnt; i++) {
		if ( topic->comp(_topics[i].getTopicName()) == 0) {
            return &_topics[i];
        }
    }
    return 0;
}

Topic* Topics::getTopic(uint16_t id) {
    for (int i = 0; i < _elmCnt; i++) {
        if ( _topics[i].getTopicId() == id) {
            return &_topics[i];
        }
    }
      return 0;
}

bool Topics::setTopicId(MQString* topic, uint16_t id){
    Topic* p = getTopic(topic);
    if ( p != 0) {
        p->setTopicId(id);
        return true;
    }else{
        return false;
    }
}

bool Topics::setCallback(MQString* topic, TopicCallback callback){
    Topic* p = getTopic(topic);
    if ( p != 0) {
        p->setCallback(callback);
        return true;
    }else{
        return false;
    }
}
bool Topics::setCallback(uint16_t topicId, TopicCallback callback){
    Topic* p = getTopic(topicId);
    if ( p != 0) {
        p->setCallback(callback);
        return true;
    }else{
        return false;
    }
}
int Topics::execCallback(uint16_t topicId, MqttsnPublish* msg){
    Topic* p = getTopic(topicId);
    if ( p != 0) {
        return p->execCallback(msg);
    }
    return 0;
}

int Topics::execCallback(MQString* topic, MqttsnPublish* msg){
    Topic* p = getTopic(topic);
    if ( p != 0) {
        return p->execCallback(msg);
    }
    return 0;
}

void Topics::addTopic(MQString* topic){
    if (getTopic(topic) == 0){
        if ( _elmCnt < _sizeMax){
            _topics[_elmCnt].setTopicName(topic);
            _elmCnt++;
        }else{
            Topic* saveTopics = _topics;
            Topic* newTopics = (Topic*)calloc(_sizeMax += MQTTSN_MAX_TOPICS + 1, sizeof(Topic));
            if (newTopics != 0){
                _topics = newTopics;
                for(int i = 0; i < _elmCnt; i++){
                    _topics[i].copy(&saveTopics[i]);
                    saveTopics[i].setTopicName((MQString*)0);
                }

                _topics[_elmCnt].setTopicName(topic);
                _elmCnt++;
                if (saveTopics){
                    delete saveTopics;
                }
            }
        }
    }
}

Topic* Topics::match(MQString* topic){
    for ( int i = 0; i< _elmCnt; i++){
        if (_topics[i].isWildCard()){
            if (getTopic(topic)->isMatch(&_topics[i])){
               return &_topics[i];
            }
        }
    }
    return 0;
}

void Topics::setSize(uint8_t size){
    _sizeMax = size;
}

void Topics::clearTopic(void){
	for( int i = 0; i < _elmCnt; i++){
		if ( _topics[i].getTopicId() >= MQTTSN_TOPICID_NORMAL) {
			_elmCnt = i + 1;
			break;
		}
	}
}


/*=====================================
        Class Payload
  =====================================*/
Payload::Payload(){
	_buff = _pos = 0;
	_len = 0;
	_elmCnt = 0;
	_memDlt = 0;
}

Payload::Payload(uint16_t len){
	_buff = (uint8_t*)calloc(len, sizeof(uint8_t));
	if(_buff == 0){
		exit(-1);
	}
	_pos = _buff;
	_elmCnt = 0;
	_len = len;
	_memDlt = 1;
}

Payload::~Payload(){
	if(_memDlt){
		free(_buff);
	}
}

void Payload::init(){
	_pos = _buff;
	_elmCnt = 0;
}


void Payload::getPayload(MqttsnPublish* msg){
	if(_memDlt){
			free(_buff);
			_memDlt = 0;
	}
	_buff = msg->getData();
	_len = msg->getDataLength();
	_pos = _buff + _len;
	/*
	for(uint8_t i; i < MSGPACK_MAX_ELEMENTS; i++){
		if(getBufferPos(i) != 0){
			_elmCnt++;
		}else{
			break;
		}
	}
	*/
	_elmCnt = 6;
}

uint16_t Payload::getAvailableLength(){
	return _len - (_pos - _buff);
}

uint16_t Payload::getLen(){
	return _pos - _buff;
}

uint8_t* Payload::getBuf(){
	return _buff;
}

/*======================
 *     setter
 ======================*/
int8_t Payload::set_uint32(uint32_t val){
	if(getAvailableLength() < 6){
		return -1;
	}
	if(val < 128){
		*_pos++ = (uint8_t)val;
	}else if(val < 256){
		*_pos++ = MSGPACK_UINT8;
		*_pos++ = (uint8_t)val;
	}else if(val < 65536){
		*_pos++ = MSGPACK_UINT16;
		setUint16(_pos,(uint16_t) val);
		_pos += 2;
	}else{
		*_pos++ = MSGPACK_UINT32;
		setUint32(_pos, val);
		_pos += 4;
	}
	_elmCnt++;
	return 0;
}

int8_t Payload::set_int32(int32_t val){
	if(getAvailableLength() < 6){
			return -1;
	}
	if((val > -32) && (val < 0)){
		*_pos++ = val | MSGPACK_NEGINT;
	}else if((val >= 0) && (val < 128)){
		*_pos++ = val;
	}else if(val > -128 && val < 128){
		*_pos++ = MSGPACK_INT8;
		*_pos++ = (uint8_t)val;
	}else if(val > -32768 && val < 32768){
		*_pos++ = MSGPACK_INT16;
		setUint16(_pos, (uint16_t)val);
		_pos += 2;
	}else{
		*_pos++ = MSGPACK_INT32;
		setUint32(_pos, (uint32_t)val);
		_pos += 4;
	}
	_elmCnt++;
	return 0;
}

int8_t Payload::set_float(float val){
	if(getAvailableLength() < 6){
			return -1;
	}
	*_pos++ = MSGPACK_FLOAT32;
	setFloat32(_pos, val);
	_pos += 4;
	_elmCnt++;
	return 0;
}

int8_t Payload::set_str(char* val){
	return set_str((const char*) val);
}

int8_t Payload::set_str(const char* val){
	if(getAvailableLength() < strlen(val) + 3){
		return -1;
	}else if(strlen(val) < 32){
		*_pos++ = (uint8_t)strlen(val) | MSGPACK_FIXSTR;
	}else if(strlen(val) < 256){
		*_pos++ = MSGPACK_STR8;
		*_pos++ = (uint8_t)strlen(val);
	}else if(strlen(val) < 65536){
		*_pos++ = MSGPACK_STR16;
		setUint16(_pos, (uint16_t)strlen(val));
		_pos += 2;
	}
	memcpy(_pos, val, strlen(val));
	_pos += strlen(val);
	return 0;
}

int8_t Payload::set_array(uint8_t val){
	if(getAvailableLength() < (uint16_t)val+ 1){
		return -1;
	}
	if(val < 16){
		*_pos++ = MSGPACK_ARRAY15 | val;
	}else{
		*_pos++ = MSGPACK_ARRAY16;
		setUint16(_pos,(uint16_t)val);
		_pos += 2;
	}
	_elmCnt++;
	return 0;
}

/*======================
 *     getter
 ======================*/
uint8_t Payload::getArray(uint8_t index){
	uint8_t rc = 0;
	uint8_t* val = getBufferPos(index);
	if(val != 0){
		if(*val == MSGPACK_ARRAY15){
			rc = *val & 0x0F;
		}else if(*val == MSGPACK_ARRAY16){
			rc = (uint8_t)getUint16(val + 1);
		}
	}
	return rc;
}

uint32_t Payload::get_uint32(uint8_t index){
	uint32_t rc = 0;
	uint8_t* val = getBufferPos(index);
	if(val != 0){
		if(*val == MSGPACK_UINT32){
			rc = getUint32(val + 1);
		}else if(*val == MSGPACK_UINT16){
			rc = (uint32_t)getUint16(val + 1);
		}else if(*val == MSGPACK_UINT8){
			rc = (uint32_t)*(val + 1);
		}else if(*val < 128){
			rc = (uint32_t)*val;
		}
	}
	return rc;
}

int32_t Payload::get_int32(uint8_t index){
	int32_t rc = 0;
	uint8_t* val = getBufferPos(index);
	if(val != 0){
		if(*val == MSGPACK_INT32){
			rc = (int32_t) getUint32(val + 1);
		}else if(*val == MSGPACK_INT16){
			uint16_t d16 = getUint16(val + 1);
			if(d16 >= 32768){
				rc = d16 - 65536;
			}else{
				rc = (int32_t)d16;
			}
		}else if(*val == MSGPACK_INT8){
			rc = (int32_t)*(val + 1);
		}else if((*val & MSGPACK_NEGINT) == MSGPACK_NEGINT){
			*val &= ~MSGPACK_NEGINT;
			rc = ((int32_t)*val) * -1;
		}else{
			rc = (int32_t) *val;
		}
	}
	return rc;
}

float Payload::get_float(uint8_t index){
	uint8_t* val = getBufferPos(index);
	if(val != 0){
		if(*val == MSGPACK_FLOAT32){
			return getFloat32(val + 1);
		}
	}
	return 0;
}

const char* Payload::get_str(uint8_t index, uint16_t* len){
	uint8_t* val = getBufferPos(index);
	if(val != 0){
		if(*val == MSGPACK_STR16){
			*len = getUint16(val + 1);
			return (const char*)(val + 3);
		}else if(*val == MSGPACK_STR8){
			*len = *(val + 1);
			return (const char*)(val + 2);
		}else if(*val & MSGPACK_FIXSTR){
			*len = *val & (~MSGPACK_FIXSTR);
			return (const char*)(val + 1);
		}
	}
	*len = 0;
	return (const char*) 0;

}


uint8_t* Payload::getBufferPos(uint8_t index){
	uint8_t* bpos = 0;
	uint8_t* pos = _buff;

	for(uint8_t i = 0; i <= index; i++){
		bpos = pos;
		switch(*pos){
		case MSGPACK_FALSE:
		case MSGPACK_TRUE:
			pos++;
			break;
		case MSGPACK_UINT8:
		case MSGPACK_INT8:
			pos += 2;
			break;
		case MSGPACK_UINT16:
		case MSGPACK_INT16:
		case MSGPACK_ARRAY16:
			pos += 3;
			break;
		case MSGPACK_UINT32:
		case MSGPACK_INT32:
		case MSGPACK_FLOAT32:
			pos += 5;
			break;
		case MSGPACK_STR8:
			pos += *(pos + 1) + 2;
			break;
		case MSGPACK_STR16:
			pos += getUint16(pos + 1) + 3;
			break;
		default:
			if((*pos < MSGPACK_POSINT) ||
				((*pos & MSGPACK_NEGINT) == MSGPACK_NEGINT) ||
				((*pos & MSGPACK_ARRAY15) == MSGPACK_ARRAY15)) {
				pos++;
			}else if((*pos & MSGPACK_FIXSTR) == MSGPACK_FIXSTR){
				pos += *pos & (~MSGPACK_FIXSTR);
			}
		}
		/*
		if((pos - _buff) >= _len){
			return 0;
		}
		*/
	}
	return bpos;
}

void Payload::print(){
	for(uint8_t* pos = _buff; pos < _pos; pos++){
		printf(" 0x%x", *pos);
	}
	printf("\n");
}

/*=====================================
        Class PublishHandller
 ======================================*/
PublishHandller::PublishHandller(){

}
PublishHandller::~PublishHandller(){

}
int PublishHandller::exec(MqttsnPublish* msg, Topics* topics){
	MQString tp = MQString();
	if((msg->getFlags() && MQTTSN_TOPIC_TYPE) == MQTTSN_TOPIC_TYPE_SHORT){
		if (topics->getTopic(msg->getTopic(&tp))){
			return topics->getTopic(msg->getTopic(&tp))->execCallback(msg);
		}
	}else{
		if (topics->getTopic(msg->getTopicId())){
			return topics->getTopic(msg->getTopicId())->execCallback(msg);
		}
	}
    return 0;
}



/////////////////// End of File ///////////////
