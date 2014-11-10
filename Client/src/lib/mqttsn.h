/*
 * mqttsn.h
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

#ifndef MQTTS_H_
#define MQTTS_H_

#ifdef ARDUINO
        #include <MQTTSN_Application.h>
#else
        #include "MQTTSN_Application.h"
#endif

#if defined(ARDUINO) && ARDUINO >= 100
        #include "Arduino.h"
        #include <inttypes.h>
        #include <Network.h>
#endif

#if defined(ARDUINO) && ARDUINO < 100
        #include "WProgram.h"
        #include <inttypes.h>
        #include <Network.h>
#endif

#ifdef LINUX
        #include <sys/time.h>
        #include <iostream>
        #include "Network.h"
#endif

#ifdef MBED
        #include "mbed.h"
        #include "Network.h"
#endif


                              /* [sec] */
#define MQTTSN_DEFAULT_KEEPALIVE 3600     // 1H
#define MQTTSN_DEFAULT_DURATION   900     // 15min
#define MQTTSN_TIME_SEARCHGW        3
#define MQTTSN_TIME_RETRY          10
#define MQTTSN_TIME_WAIT          300     //  5min
#define MQTTSN_RETRY_COUNT          5

#define MQTTSN_MAX_TOPICS         15
#define MQTTSN_MAX_PACKET_LENGTH  60

#define MQTTSN_TYPE_ADVERTISE     0x00
#define MQTTSN_TYPE_SEARCHGW      0x01
#define MQTTSN_TYPE_GWINFO        0x02
#define MQTTSN_TYPE_CONNECT       0x04
#define MQTTSN_TYPE_CONNACK       0x05
#define MQTTSN_TYPE_WILLTOPICREQ  0x06
#define MQTTSN_TYPE_WILLTOPIC     0x07
#define MQTTSN_TYPE_WILLMSGREQ    0x08
#define MQTTSN_TYPE_WILLMSG       0x09
#define MQTTSN_TYPE_REGISTER      0x0A
#define MQTTSN_TYPE_REGACK        0x0B
#define MQTTSN_TYPE_PUBLISH       0x0C
#define MQTTSN_TYPE_PUBACK        0x0D
#define MQTTSN_TYPE_PUBCOMP       0x0E
#define MQTTSN_TYPE_PUBREC        0x0F
#define MQTTSN_TYPE_PUBREL        0x10
#define MQTTSN_TYPE_SUBSCRIBE     0x12
#define MQTTSN_TYPE_SUBACK        0x13
#define MQTTSN_TYPE_UNSUBSCRIBE   0x14
#define MQTTSN_TYPE_UNSUBACK      0x15
#define MQTTSN_TYPE_PINGREQ       0x16
#define MQTTSN_TYPE_PINGRESP      0x17
#define MQTTSN_TYPE_DISCONNECT    0x18
#define MQTTSN_TYPE_WILLTOPICUPD  0x1A
#define MQTTSN_TYPE_WILLTOPICRESP 0x1B
#define MQTTSN_TYPE_WILLMSGUPD    0x1C
#define MQTTSN_TYPE_WILLMSGRESP   0x1D

#define MQTTSN_TOPIC_TYPE_NORMAL     0x00
#define MQTTSN_TOPIC_TYPE_PREDEFINED 0x01
#define MQTTSN_TOPIC_TYPE_SHORT      0x02
#define MQTTSN_TOPIC_TYPE            0x03

#define MQTTSN_FLAG_DUP     0x80
#define MQTTSN_FLAG_QOS_0   0x0
#define MQTTSN_FLAG_QOS_1   0x20
#define MQTTSN_FLAG_QOS_2   0x40
#define MQTTSN_FLAG_QOS_N1  0xc0
#define MQTTSN_FLAG_RETAIN  0x10
#define MQTTSN_FLAG_WILL    0x08
#define MQTTSN_FLAG_CLEAN   0x04

#define MQTTSN_PROTOCOL_ID  0x03
#define MQTTSN_HEADER_SIZE  2

#define MQTTSN_RC_ACCEPTED                  0x00
#define MQTTSN_RC_REJECTED_CONGESTION       0x01
#define MQTTSN_RC_REJECTED_INVALID_TOPIC_ID 0x02
#define MQTTSN_RC_REJECTED_NOT_SUPPORTED    0x03

#define MQTTSN_MSG_REQUEST     1
#define MQTTSN_MSG_RESEND_REQ  2
#define MQTTSN_MSG_WAIT_ACK    3
#define MQTTSN_MSG_COMPLETE    4
#define MQTTSN_MSG_REJECTED    5


#define MQTTSN_GW_INIT         0
#define MQTTSN_GW_SEARCHING    1
#define MQTTSN_GW_FOUND        2
#define MQTTSN_GW_CONNECTED    3
#define MQTTSN_GW_DISCONNECTED 4
#define MQTTSN_GW_LOST         5

#define MQTTSN_ERR_NO_ERROR            0
#define MQTTSN_ERR_NOT_CONNECTED      -11
#define MQTTSN_ERR_RETRY_OVER         -12
#define MQTTSN_ERR_GATEWAY_LOST       -13
#define MQTTSN_ERR_CANNOT_ADD_REQUEST -14
#define MQTTSN_ERR_NO_TOPICID         -15
#define MQTTSN_ERR_REJECTED           -16
#define MQTTSN_ERR_WAIT_GWINFO        -17
#define MQTTSN_ERR_OUT_OF_MEMORY      -18
#define MQTTSN_ERR_PING_REQUIRED      -19
#define MQTTSN_ERR_ACK_TIMEOUT        -20
#define MQTTSN_ERR_PINGRESP_TIMEOUT   -21
#define MQTTSN_ERR_INVALID_TOPICID    -22
#define MQTTSN_ERR_NO_DATA            -23
#define MQTTSN_ERR_REBOOT_REQUIRED    -24
#define MQTTSN_READ_RESP_ONCE_MORE    -25

#define MQTTSN_TOPIC_MULTI_WILDCARD   '#'
#define MQTTSN_TOPIC_SINGLE_WILDCARD  '+'

#define MQTTSN_TOPICID_NORMAL 256
#define MQTTSN_TOPICID_PREDEFINED_TIME   0x0001
#define MQTTSN_TOPIC_PREDEFINED_TIME     ("$GW/01")

#define QOS0  0
#define QOS1  1
#define QOS2  2

#define MSGPACK_FALSE    0xc2
#define MSGPACK_TRUE     0xc3
#define MSGPACK_POSINT   0x80
#define MSGPACK_NEGINT   0xe0
#define MSGPACK_UINT8    0xcc
#define MSGPACK_UINT16   0xcd
#define MSGPACK_UINT32   0xce
#define MSGPACK_INT8     0xd0
#define MSGPACK_INT16    0xd1
#define MSGPACK_INT32    0xd2
#define MSGPACK_FLOAT32  0xca
#define MSGPACK_FIXSTR   0xa0
#define MSGPACK_STR8     0xd9
#define MSGPACK_STR16    0xda
#define MSGPACK_ARRAY15  0x90
#define MSGPACK_ARRAY16  0xdc
#define MSGPACK_MAX_ELEMENTS   50   // Less than 256

using namespace tomyClient;

/*=====================================
        Class MQString
 =====================================*/
class MQString{
public:
    MQString();
    MQString(const char*);
    MQString(char*);
    ~MQString();
    uint16_t getCharLength();
    int     comp(MQString* str);
    int     comp(const char* str);
    int     ncomp(MQString* str, uint16_t n);
    void    copy(MQString* str);
    void    copy(const char* str);
    void    copy(char* str);
    void    copy(uint8_t* str, uint8_t len);
    MQString* create();
    void    writeBuf(uint8_t* buf);
    void    readBuf(uint8_t* buf);
    uint8_t getChar(uint8_t index);
    char*  getStr();
    const char* getConstStr();
    bool    isConst();
    bool operator==(MQString&);
    bool operator!=(MQString&);
private:
    void    freeStr();
    char*    _str;
    const char* _constStr;
};

/*=====================================
        Class MqttsnMessage
  =====================================*/
class MqttsnMessage {
public:
    MqttsnMessage();
    ~MqttsnMessage();
    void   setLength(uint16_t length);
    void   setType(uint8_t type);
    bool   setBody(uint8_t* body);
    bool   allocateBody();
    void   setStatus(uint8_t stat);
    void   setDup();
    void   setQos(uint8_t qos);
    void   setRetain(bool flg);
    void   setFlags(uint8_t flag);
    uint8_t getLength();
    uint8_t getType();
    uint8_t getFlags();
    uint8_t getStatus();
    uint8_t getQos();
    uint8_t* getBody();
    uint8_t* getMsgBuff();
    uint16_t getFrameLength();
    uint16_t getBodyLength();
    bool    isDup();
    bool    copy(MqttsnMessage* src);
    void    reset();
    void    setMsgBuff(uint8_t* buff);
    const char* getMsgTypeName();
protected:
    uint8_t* _msgBuff;
    uint8_t  _flags;
private:
    uint8_t  _status; // 1:request 2:sending 3:resending 4:waitingAck  5:complite
    uint8_t  _type;
    uint16_t  _length;
};

/*=====================================
        Class MqttsnAdvertize
 ======================================*/
class MqttsnAdvertise : public MqttsnMessage {
public:
  MqttsnAdvertise();
  ~MqttsnAdvertise();
  void setGwId(uint8_t id);
  void setDuration(uint16_t duration);
  uint8_t getGwId();
  uint16_t getDuration();

private:
};

/*=====================================
        Class MqttsnSearchGw
 ======================================*/
class MqttsnSearchGw : public MqttsnMessage {
public:
    MqttsnSearchGw();
  ~MqttsnSearchGw();
  void setRadius(uint8_t radius);
  uint8_t getRadius();

private:
};

/*=====================================
        Class MqttsnGwinfo
 ======================================*/
class MqttsnGwInfo : public MqttsnMessage {
public:
  MqttsnGwInfo();
  ~MqttsnGwInfo();
  void setGwId(uint8_t id);
  uint8_t getGwId();

private:
};

/*=====================================
         Class MqttsnConnect
  ======================================*/
class MqttsnConnect : public MqttsnMessage {
public:
    MqttsnConnect(MQString* id);
    ~MqttsnConnect();
    void setDuration(uint16_t msec);
    void setClientId(MQString* id);
    uint8_t* getClientId();
    uint16_t getDuration();
    void setFrame(uint8_t* data, uint8_t len);
private:
 };

/*=====================================
        Class MqttsnConnack
 ======================================*/
class MqttsnConnack : public MqttsnMessage  {
public:
    MqttsnConnack();
    ~MqttsnConnack();
    void setReturnCode(uint8_t rc);
    uint8_t getReturnCode();

private:

};

/*=====================================
         Class MqttsnWillTopicReq
  ======================================*/
class MqttsnWillTopicReq : public MqttsnMessage  {
public:
    MqttsnWillTopicReq();
    ~MqttsnWillTopicReq();

private:

 };

/*=====================================
         Class MqttsnWillTopic
  ======================================*/
class MqttsnWillTopic : public MqttsnMessage  {
public:
    MqttsnWillTopic();
    ~MqttsnWillTopic();
    void setWillTopic(MQString* topic);
    MQString* getWillTopic();
    bool isWillRequired();

private:
    MQString _ustring;
 };

/*=====================================
         Class MqttsnWillMsgReq
  ======================================*/
class MqttsnWillMsgReq : public MqttsnMessage  {
public:
    MqttsnWillMsgReq();
    ~MqttsnWillMsgReq();

private:

 };

/*=====================================
         Class MqttsnWillMsg
  ======================================*/
class MqttsnWillMsg : public MqttsnMessage  {
public:
    MqttsnWillMsg();
    ~MqttsnWillMsg();
    void setWillMsg(MQString* msg);
    char* getWillMsg();

private:

 };

/*=====================================
         Class MqttsnRegister
  ======================================*/
class MqttsnRegister : public MqttsnMessage  {
public:
    MqttsnRegister();
    ~MqttsnRegister();
    void setTopicId(uint16_t topicId);
    uint16_t getTopicId();
    void setMsgId(uint16_t msgId);
    uint16_t getMsgId();
    void setTopicName(MQString* topicName);
    void setFrame(uint8_t* data, uint8_t len);
    void setFrame(NWResponse* resp);
    MQString* getTopicName();

private:
    uint16_t _topicId;
    uint16_t _msgId;
    MQString _ustring;

 };

/*=====================================
         Class MqttsnRegAck
  ======================================*/
class MqttsnRegAck : public MqttsnMessage  {
public:
    MqttsnRegAck();
    ~MqttsnRegAck();
    void setTopicId(uint16_t topicId);
    uint16_t getTopicId();
    void setMsgId(uint16_t msgId);
    uint16_t getMsgId();
    void setReturnCode(uint8_t rc);
    uint8_t getReturnCode();

private:

 };

 /*=====================================
         Class MqttsnPublish
  ======================================*/
class Payload;

class MqttsnPublish : public MqttsnMessage  {
public:
    MqttsnPublish();
    ~MqttsnPublish();
    void setTopicId(uint16_t id);
    void setTopic(MQString* topic);
    void setMsgId(uint16_t msgId);
    void setData(uint8_t* data, uint16_t len);
    void setData(MQString* str);
    void setFrame(uint8_t* data, uint16_t len);
    void setFrame(NWResponse* resp);
    void setPayload(Payload* payload);
    MQString* getTopic(MQString* topic);
    uint16_t getMsgId();
    uint16_t getTopicId();

    uint8_t* getData();
    uint16_t getDataLength();

private:
    uint16_t _topicId;
    uint16_t _msgId;
    MQString* _topic;
 };

/*=====================================
         Class MqttsnPubAck
  ======================================*/
class MqttsnPubAck : public MqttsnMessage  {
public:
    MqttsnPubAck();
    ~MqttsnPubAck();
    void setTopicId(uint16_t id);
    uint16_t getTopicId();
    void setMsgId(uint16_t msgId);
    uint16_t getMsgId();
    void setReturnCode(uint8_t rc);
    uint8_t getReturnCode();


private:

 };

/*=====================================
         Class MqttsnPubRec
  ======================================*/
class MqttsnPubRec : public MqttsnMessage  {
public:
	MqttsnPubRec();
    ~MqttsnPubRec();
    void setMsgId(uint16_t msgId);
    uint16_t getMsgId();

private:

 };

/*=====================================
         Class MqttsnPubRel
  ======================================*/
class MqttsnPubRel : public MqttsnPubRec  {
public:
	MqttsnPubRel();
    ~MqttsnPubRel();
private:

 };

/*=====================================
         Class MqttsnPubComp
  ======================================*/
class MqttsnPubComp : public MqttsnPubRec  {
public:
	MqttsnPubComp();
    ~MqttsnPubComp();
private:

 };

 /*=====================================
         Class MqttsnSubscribe
  ======================================*/
class MqttsnSubscribe : public MqttsnMessage  {
public:
    MqttsnSubscribe();
    ~MqttsnSubscribe();
    void setMsgId(uint16_t msgId);
    uint16_t getMsgId();
    uint16_t getTopicId();
    MQString* getTopicName();
    void setTopicId(uint16_t predefinedId);
	void setFrame(uint8_t* data, uint8_t len);
	void setFrame(NWResponse* resp);
    void setTopicName(MQString* topicName);
protected:
    uint16_t _topicId;
    uint16_t _msgId;
    MQString _ustring;
 };

/*=====================================
         Class MqttsnSubAck
  ======================================*/
class MqttsnSubAck : public MqttsnMessage  {
public:
    MqttsnSubAck();
    ~MqttsnSubAck();
    void setMsgId(uint16_t msgId);
    uint16_t getMsgId();
    void setTopicId(uint16_t topicId);
    uint16_t getTopicId();
    void setReturnCode(uint8_t rc);
    uint8_t getReturnCode();

private:

 };

 /*=====================================
         Class MqttsnUnsubscribe
  ======================================*/
class MqttsnUnsubscribe : public MqttsnSubscribe  {
public:
    MqttsnUnsubscribe();
    ~MqttsnUnsubscribe();
private:

 };

/*=====================================
         Class MqttsnUnSubAck
  ======================================*/
class MqttsnUnSubAck : public MqttsnMessage  {
public:
    MqttsnUnSubAck();
    ~MqttsnUnSubAck();
    void setMsgId(uint16_t msgId);
    uint16_t getMsgId();

private:

 };

/*=====================================
        Class MqttsnPingReq
 ======================================*/
class MqttsnPingReq : public MqttsnMessage  {
public:
    MqttsnPingReq(MQString* id);
    ~MqttsnPingReq();
    char* getClientId();
private:

};

/*=====================================
        Class MqttsnPingResp
 ======================================*/
class MqttsnPingResp : public MqttsnMessage  {
public:
    MqttsnPingResp();
    ~MqttsnPingResp();
private:

};

 /*=====================================
         Class MqttsnDisconnect
  ======================================*/
class MqttsnDisconnect : public MqttsnMessage  {
public:
    MqttsnDisconnect();
    ~MqttsnDisconnect();
    void setDuration(uint16_t duration);
    uint16_t getDuration();
private:

 };

/*=====================================
        Class Topic
 ======================================*/
typedef int (*TopicCallback)(MqttsnPublish*);

class Topic {
public:
    Topic();
    ~Topic();
    uint8_t   getStatus();
    uint16_t  getTopicId();
    MQString*  getTopicName();
    uint8_t   getTopicLength();
    uint8_t   getTopicType();
    TopicCallback getCallback();
    void     setTopicId(uint16_t id);
    void     setTopicName(MQString* topic);
    void     setStatus(uint8_t stat);
    int      execCallback(MqttsnPublish* msg);
    void     copy(Topic* src);
    void     setCallback(TopicCallback callback);
    uint8_t   isWildCard();
    bool     isMatch(Topic* wildCard);
private:
    uint16_t  _topicId;
    uint8_t   _status;
    MQString*  _topicStr;
    TopicCallback  _callback;
};

/*=====================================
        Class Topics
 ======================================*/
class Topics {
public:
      Topics();
      ~Topics();
      bool     allocate(uint8_t topicsSize);
      uint16_t  getTopicId(MQString* topic);
      Topic*    getTopic(MQString* topic);
      Topic*    getTopic(uint16_t topicId);
      bool     setTopicId(MQString* topic, uint16_t id);
      bool     setCallback(MQString* topic, TopicCallback callback);
      bool     setCallback(uint16_t topicId, TopicCallback callback);
      int      execCallback(uint16_t  topicId, MqttsnPublish* msg);
      int      execCallback(MQString* topic, MqttsnPublish* msg);
      void     addTopic(MQString* topic);
      Topic*    match(MQString* topic);
      void     setSize(uint8_t size);
      void     clearTopic(void);

private:

    uint8_t   _sizeMax;
    uint8_t   _elmCnt;
    Topic*  _topics;

};

/*=====================================
        Class Payload
  =====================================*/
class Payload{
public:
	Payload();
	Payload(uint16_t len);
	~Payload();
	void init(void);
	int8_t set_uint32(uint32_t val);
	int8_t set_int32(int32_t val);
	int8_t set_float(float val);
	int8_t set_str(char* val);
	int8_t set_str(const char* val);
	int8_t set_array(uint8_t val);

	uint8_t getArray(uint8_t index);
	uint32_t get_uint32(uint8_t index);
	int32_t  get_int32(uint8_t index);
    float    get_float(uint8_t index);
    const char* get_str(uint8_t index, uint16_t* len);

	void 	 getPayload(MqttsnPublish* msg);
	uint16_t getAvailableLength();
	uint16_t getLen();
	uint8_t* getBuf();

	void print();
private:
	uint8_t* getBufferPos(uint8_t index);
	uint8_t* _buff;
	uint16_t _len;
	uint8_t  _elmCnt;
	uint8_t* _pos;
	uint8_t  _memDlt;
};

/*=====================================
        Class Publish Handler
 ======================================*/
class PublishHandller {
public:
    PublishHandller();
    ~PublishHandller();
    int exec(MqttsnPublish* msg, Topics* topics);

};


#endif  /* MQTTS_H_ */
