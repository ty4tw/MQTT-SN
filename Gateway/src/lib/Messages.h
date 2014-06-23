/*
 * Messages.h
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

#ifndef MESSAGES_H_
#define MESSAGES_H_


/*
 *     MQTT-SN Message Type
 */
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

/*
 *     MQTT Message types
 */
#define MQTT_PROTOCOL_NAME3      "MQIsdp"
#define MQTT_PROTOCOL_NAME4      "MQTT"
#define MQTT_PROTOCOL_VER4        4
#define MQTT_PROTOCOL_VER3        3


#define MQTT_TYPE_CONNECT         0x10
#define MQTT_TYPE_CONNACK         0x20
#define MQTT_TYPE_PUBLISH         0x30
#define MQTT_TYPE_PUBACK          0x40
#define MQTT_TYPE_PUBREC          0x50
#define MQTT_TYPE_PUBREL          0x60
#define MQTT_TYPE_PUBCOMP         0x70
#define MQTT_TYPE_SUBSCRIBE       0x80
#define MQTT_TYPE_SUBACK          0x90
#define MQTT_TYPE_UNSUBSCRIBE     0xA0
#define MQTT_TYPE_UNSUBACK        0xB0
#define MQTT_TYPE_PINGREQ         0xC0
#define MQTT_TYPE_PINGRESP        0xD0
#define MQTT_TYPE_DISCONNECT      0xE0

#define MQTT_RC_ACCEPTED                      0
#define MQTT_RC_REFUSED_PROTOCOL_VERSION      1
#define MQTT_RC_REFUSED_IDENTIFIER_REJECTED   2
#define MQTT_RC_REFUSED_SERVER_UNAVAILABLE    3
#define MQTT_RC_REFUSED_BAD_USERNAME_PASSWORD 4
#define MQTT_RC_REFUSED_NOT_AUTHORIZED        5

/*
 *    Flags
 */
#define MQTTSN_FLAG_DUP                     0x80
#define MQTTSN_FLAG_QOS_0                   0x0
#define MQTTSN_FLAG_QOS_1                   0x20
#define MQTTSN_FLAG_QOS_2                   0x40
#define MQTTSN_FLAG_QOS_N1                  0xc0
#define MQTTSN_FLAG_RETAIN                  0x10
#define MQTTSN_FLAG_WILL                    0x08
#define MQTTSN_FLAG_CLEAN                   0x04
#define MQTTSN_FLAG_TOPICID_TYPE_NORMAL     0x00
#define MQTTSN_FLAG_TOPICID_TYPE_PREDEFINED 0x01
#define MQTTSN_FLAG_TOPICID_TYPE_SHORT      0x02
#define MQTTSN_FLAG_TOPICID_TYPE_RESV       0x03

#define MQTTSN_TOPIC_TYPE_NORMAL            0x00
#define MQTTSN_TOPIC_TYPE_PREDEFINED        0x01
#define MQTTSN_TOPIC_TYPE_SHORT             0x02
#define MQTTSN_TOPIC_TYPE                   0x03

#define MQTTSN_PROTOCOL_ID  0x01
#define MQTTSN_HEADER_SIZE  2

/*
 *     Return Code
 */
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
#define MQTTSN_ERR_NOT_CONNECTED      -1
#define MQTTSN_ERR_RETRY_OVER         -2
#define MQTTSN_ERR_GATEWAY_LOST       -3
#define MQTTSN_ERR_CANNOT_ADD_REQUEST -4
#define MQTTSN_ERR_NO_TOPICID         -5
#define MQTTSN_ERR_REJECTED           -6
#define MQTTSN_ERR_WAIT_GWINFO        -7
#define MQTTSN_ERR_OUT_OF_MEMORY      -8
#define MQTTSN_ERR_PING_REQUIRED      -9
#define MQTTSN_ERR_ACK_TIMEOUT       -10
#define MQTTSN_ERR_PINGRESP_TIMEOUT  -11


#include "ZBStack.h"
#include "UDPStack.h"
#include "ProcessFramework.h"
#include <string>

using namespace std;
using namespace tomyGateway;

/*=====================================
        Class MQTTSnMessage
  =====================================*/
class MQTTSnMessage{
public:
	MQTTSnMessage();
    ~MQTTSnMessage();
    void setMessageLength(uint16_t length);
    void setType(uint8_t type);
    void setBody(uint8_t* body, uint16_t bodyLength);
    void setMessage(uint8_t* msg);

    uint8_t  getType();
    uint16_t getBodyLength();
    uint16_t getMessageLength();
    uint8_t* getMessagePtr();
    uint8_t* getBodyPtr();
    bool    getMessage(uint16_t pos, uint8_t& val);

    void absorb(MQTTSnMessage* src);
    void absorb(NWResponse* src);
protected:
    void allocate();
    uint8_t* _message;
private:
    uint16_t  _length;
	uint8_t   _type;
};

/*=====================================
        Class MQTTSnAdvertize
 ======================================*/
class MQTTSnAdvertise : public MQTTSnMessage {
public:
	MQTTSnAdvertise();
	~MQTTSnAdvertise();
	void setGwId(uint8_t id);
	void setDuration(uint16_t duration);
	uint8_t getGwId();
	uint16_t getDuration();
private:
};

/*=====================================
        Class MQTTSnSearchGw
 ======================================*/
class MQTTSnSearchGw : public MQTTSnMessage {
public:
	MQTTSnSearchGw();
	~MQTTSnSearchGw();
	void setRadius(uint8_t radius);
	uint8_t getRadius();
	void absorb(NWResponse* src);
private:
};

/*=====================================
        Class MQTTSnGwinfo
 ======================================*/
class MQTTSnGwInfo : public MQTTSnMessage {
public:
  MQTTSnGwInfo();
  ~MQTTSnGwInfo();
  void setGwId(uint8_t id);
  uint8_t getGwId();

private:
};

/*=====================================
         Class MQTTSnConnect
  ======================================*/
class MQTTSnConnect : public MQTTSnMessage {
public:
	MQTTSnConnect();
    MQTTSnConnect(string* id);
    ~MQTTSnConnect();

    void setFlags(uint8_t flg);
    void setDuration(uint16_t msec);
    void setClientId(string id);

    uint8_t getFlags();
    uint16_t getDuration();
    string* getClientId();
    bool isCleanSession();
    bool isWillRequired();

    void absorb(MQTTSnMessage* src);
    void absorb(NWResponse* src);

private:
    string _clientId;
 };

/*=====================================
        Class MQTTSnConnack
 ======================================*/
class MQTTSnConnack : public MQTTSnMessage  {
public:
    MQTTSnConnack();
    ~MQTTSnConnack();
    void setReturnCode(uint8_t rc);
    uint8_t getReturnCode();

private:

};

/*=====================================
         Class MQTTSnWillTopicReq
  ======================================*/
class MQTTSnWillTopicReq : public MQTTSnMessage  {
public:
    MQTTSnWillTopicReq();
    ~MQTTSnWillTopicReq();

private:

 };

/*=====================================
         Class MQTTSnWillTopic
  ======================================*/
class MQTTSnWillTopic : public MQTTSnMessage  {
public:
    MQTTSnWillTopic();
    ~MQTTSnWillTopic();
    void setFlags(uint8_t flags);
    void setWillTopic(string* topic);
    string* getWillTopic();
    uint8_t getQos();
    bool isWillRequired();

    void absorb(MQTTSnMessage* src);
    void absorb(NWResponse* src);

private:
    uint8_t _flags;
    string _topicName;
 };

/*=====================================
         Class MQTTSnWillMsgReq
  ======================================*/
class MQTTSnWillMsgReq : public MQTTSnMessage  {
public:
    MQTTSnWillMsgReq();
    ~MQTTSnWillMsgReq();

private:

 };

/*=====================================
         Class MQTTSnWillMsg
  ======================================*/
class MQTTSnWillMsg : public MQTTSnMessage  {
public:
    MQTTSnWillMsg();
    ~MQTTSnWillMsg();
    void setWillMsg(string* msg);
    string* getWillMsg();

    void absorb(MQTTSnMessage* src);
    void absorb(NWResponse* src);

private:
    string _willMsg;
 };

/*=====================================
         Class MQTTSnRegister
  ======================================*/
class MQTTSnRegister : public MQTTSnMessage  {
public:
    MQTTSnRegister();
    ~MQTTSnRegister();
    void setTopicId(uint16_t topicId);
    void setMsgId(uint16_t msgId);
    void setTopicName(string* topicName);
    //void setFrame(uint8_t* data, uint8_t len);
    //void setFrame(NWResponse* resp);
    uint16_t getMsgId();
    uint16_t getTopicId();
    string*  getTopicName();

    void absorb(MQTTSnMessage* src);
    void absorb(NWResponse* src);
private:
    uint16_t _topicId;
    uint16_t _msgId;
    string _topicName;

 };

/*=====================================
         Class MQTTSnRegAck
  ======================================*/
class MQTTSnRegAck : public MQTTSnMessage  {
public:
    MQTTSnRegAck();
    ~MQTTSnRegAck();
    void setTopicId(uint16_t topicId);
    void setMsgId(uint16_t msgId);
    void setReturnCode(uint8_t rc);
    uint16_t getMsgId();
    uint16_t getTopicId();
    uint8_t  getReturnCode();

    void absorb(NWResponse* src);
private:

 };

 /*=====================================
         Class MQTTSnPublish
  ======================================*/
class MQTTSnPublish : public MQTTSnMessage  {
public:
    MQTTSnPublish();
    ~MQTTSnPublish();
    void setFlags(uint8_t flags);

    void setTopicId(uint16_t id);
    void setTopic(string* topic);
    void setMsgId(uint16_t msgId);
    void setData(uint8_t* data, uint8_t len);
    //void setFrame(uint8_t* data, uint8_t len);
    //void setFrame(NWResponse* resp);
	void setQos(uint8_t);
	void setDup();
	void setRetain();
	void setTopicIdType(uint8_t);

    uint16_t getTopicId();
    string*  getTopic(string* str);
    uint8_t  getTopicType();
    uint8_t  getQos();
    uint8_t  getFlags();
    uint8_t* getData();
    uint16_t getDataLength();
    uint16_t getMsgId();

    void absorb(MQTTSnMessage* src);
    void absorb(NWResponse* src);

private:
    uint8_t _flags;
    uint16_t _topicId;
    uint16_t _msgId;
 };

/*=====================================
         Class MQTTSnPubAck
  ======================================*/
class MQTTSnPubAck : public MQTTSnMessage  {
public:
    MQTTSnPubAck();
    ~MQTTSnPubAck();
    void setTopicId(uint16_t id);
    void setMsgId(uint16_t msgId);
    void setReturnCode(uint8_t rc);

    uint8_t  getReturnCode();
    uint16_t getTopicId();
    uint16_t getMsgId();

    void absorb(MQTTSnMessage* src);
    void absorb(NWResponse* src);

private:
    uint16_t _topicId;
	uint16_t _msgId;
	uint8_t  _returnCode;
 };

/*=====================================
         Class MQTTSnPubRec
  ======================================*/
class MQTTSnPubRec : public MQTTSnMessage  {
public:
	MQTTSnPubRec();
    ~MQTTSnPubRec();
    void setMsgId(uint16_t msgId);
    uint16_t getMsgId();

    //void absorb(MQTTSnMessage* src);
    //void absorb(NWResponse* src);
protected:
	uint16_t _msgId;
 };

/*=====================================
         Class MQTTSnPubRel
  ======================================*/
class MQTTSnPubRel : public MQTTSnPubRec  {
public:
	MQTTSnPubRel();
    ~MQTTSnPubRel();
 };

/*=====================================
         Class MQTTSnPubRel
  ======================================*/
class MQTTSnPubComp : public MQTTSnPubRec  {
public:
	MQTTSnPubComp();
    ~MQTTSnPubComp();
private:

 };

 /*=====================================
         Class MQTTSnSubscribe
  ======================================*/
class MQTTSnSubscribe : public MQTTSnMessage  {
public:
    MQTTSnSubscribe();
    ~MQTTSnSubscribe();
    void setFlags(uint8_t flags);
    void setMsgId(uint16_t msgId);
	void setQos(uint8_t);
    void setTopicName(string* topicName);
    void setTopicId(uint16_t predefinedId);
	//void setFrame(uint8_t* data, uint8_t len);
	//void setFrame(NWResponse* resp);uint16_t getMsgId();
    uint8_t  getQos();
    uint8_t  getFlags();
    uint16_t getTopicId();
    uint16_t getMsgId();
    string*  getTopicName();

    void absorb(MQTTSnMessage* src);
    void absorb(NWResponse* src);

protected:
    uint16_t _topicId;
    uint16_t _msgId;
    uint8_t  _flags;
    string _topicName;
 };

/*=====================================
         Class MQTTSnSubAck
  ======================================*/
class MQTTSnSubAck : public MQTTSnMessage  {
public:
    MQTTSnSubAck();
    ~MQTTSnSubAck();
    void setFlags(uint8_t flags);
    void setQos(uint8_t qos);
    void setMsgId(uint16_t msgId);
    void setTopicId(uint16_t topicId);
    void setReturnCode(uint8_t rc);

    uint8_t  getFlags();
    uint8_t  getQos();
    uint8_t  getReturnCode();
    uint16_t getMsgId();
    uint16_t getTopicId();

private:
    uint16_t _topicId;
	uint16_t _msgId;
	uint8_t  _flags;
	uint8_t  _returnCode;
	string   _topicName;
 };

 /*=====================================
         Class MQTTSnUnsubscribe
  ======================================*/
class MQTTSnUnsubscribe : public MQTTSnSubscribe  {
public:
    MQTTSnUnsubscribe();
    ~MQTTSnUnsubscribe();
    void setFlags(uint8_t flags);

    string* getTopicName();
    uint16_t  getTopicId();

    void absorb(NWResponse* src);
    void absorb(MQTTSnMessage* src);
private:

 };

/*=====================================
         Class MQTTSnUnSubAck
  ======================================*/
class MQTTSnUnsubAck : public MQTTSnMessage  {
public:
    MQTTSnUnsubAck();
    ~MQTTSnUnsubAck();
    void setMsgId(uint16_t msgId);
    uint16_t getMsgId();

private:
    uint16_t _msgId;
 };

/*=====================================
        Class MQTTSnPingReq
 ======================================*/
class MQTTSnPingReq : public MQTTSnMessage  {
public:
	MQTTSnPingReq();
    MQTTSnPingReq(string* id);
    ~MQTTSnPingReq();
    void setClientId(string* id);
    char* getClientId();

    void absorb(NWResponse* src);
private:

};

/*=====================================
        Class MQTTSnPingResp
 ======================================*/
class MQTTSnPingResp : public MQTTSnMessage  {
public:
    MQTTSnPingResp();
    ~MQTTSnPingResp();
private:

};

 /*=====================================
         Class MQTTSnDisconnect
  ======================================*/
class MQTTSnDisconnect : public MQTTSnMessage  {
public:
    MQTTSnDisconnect();
    ~MQTTSnDisconnect();
    void setDuration(uint16_t duration);
    uint16_t getDuration();

    void absorb(MQTTSnMessage* src);
    void absorb(NWResponse* src);
private:

 };





/*=====================================
        Class RemaingLength
 =====================================*/
class RemainingLength{
public:
	RemainingLength();
	~RemainingLength();
	void encode(uint16_t);
	uint16_t decode();
	uint16_t serialize(uint8_t* pos);
	void deserialize(uint8_t* pos);
	uint8_t getSize();

private:
	uint8_t _digit[4];
	uint8_t _size;
};

/*=====================================
         Class MQTTMessage
  ======================================*/
class MQTTMessage{
public:
	MQTTMessage();
	~MQTTMessage();
	uint8_t getType();
	uint8_t getQos();   // QOS0 = 0, QOS1 = 1
	uint16_t getRemainLength();
	uint8_t getRemainLengthSize();
	bool isDup();
	bool isRetain();
	uint16_t serialize(uint8_t* buf);
	bool deserialize(uint8_t* buf);

	void setType(uint8_t);
	void setQos(uint8_t);
	void setDup();
	void setRetain();
	void absorb(MQTTMessage* msg);

protected:
	uint8_t _type;
	uint8_t _flags;
	uint16_t _remainLength;
	uint16_t _messageId;

	uint8_t* _payload;

	string _userName;
	string _password;
	string _willTopic;
	string _willMessage;
	string _clientId;

	string _topic;


};


/*=====================================
         Class MQTTPingReq
  ======================================*/
class MQTTPingReq : public MQTTMessage{
public:
	MQTTPingReq();
	~MQTTPingReq();
};

/*=====================================
         Class MQTTPingResp
  ======================================*/
class MQTTPingResp : public MQTTMessage{
public:
	MQTTPingResp();
	~MQTTPingResp();
};

/*=====================================
         Class MQTTDisconnect
  ======================================*/
class MQTTDisconnect : public MQTTMessage{
public:
	MQTTDisconnect();
	~MQTTDisconnect();
};

/*=====================================
         Class MQTTPubAck
  ======================================*/
class MQTTPubAck : public MQTTMessage{
public:
	MQTTPubAck();
	~MQTTPubAck();
	void setMessageId(uint16_t);
	uint16_t getMessageId();

};

/*=====================================
         Class MQTTPubRec
  ======================================*/
class MQTTPubRec : public MQTTMessage{
public:
	MQTTPubRec();
	~MQTTPubRec();
	void setMessageId(uint16_t);
	uint16_t getMessageId();

};

/*=====================================
         Class MQTTPubRel
  ======================================*/
class MQTTPubRel : public MQTTPubRec{
public:
	MQTTPubRel();
	~MQTTPubRel();
	//void setMessageId(uint16_t);
	//uint16_t getMessageId();
};

/*=====================================
         Class MQTTPubComp
  ======================================*/
class MQTTPubComp : public MQTTPubRec{
public:
	MQTTPubComp();
	~MQTTPubComp();
	//void setMessageId(uint16_t);
	//uint16_t getMessageId();
};
/*=====================================
         Class MQTTConnAck
  ======================================*/
class MQTTConnAck : public MQTTMessage{
public:
	MQTTConnAck();
	~MQTTConnAck();
	uint8_t getReturnCd();
	bool deserialize(uint8_t* buf);
private:
	uint8_t _returnCd;
};

/*=====================================
         Class MQTTUnsubAck
  ======================================*/
class MQTTUnsubAck : public MQTTMessage{
public:
	MQTTUnsubAck();
	~MQTTUnsubAck();
	void setMessageId(uint16_t);
	uint16_t getMessageId();
};

/*=====================================
         Class MQTTSubAck
  ======================================*/
class MQTTSubAck : public MQTTMessage{
public:
	MQTTSubAck();
	~MQTTSubAck();
	void setMessageId(uint16_t);
	uint16_t getMessageId();
	uint8_t getGrantedQos();
	void setGrantedQos0();
	void setGrantedQos1();
	bool deserialize(uint8_t* buf);
private:
	uint8_t _qos;
};

/*=====================================
         Class MQTTUnSubscribe
  ======================================*/
class MQTTUnsubscribe : public MQTTMessage{
public:
	MQTTUnsubscribe();
	~MQTTUnsubscribe();
	void setMessageId(uint16_t);
	uint16_t getMessageId();
	uint16_t serialize(uint8_t* buf);
	void setTopicName(string*);
private:
	//string _topic;
};

/*=====================================
         Class MQTTSubscribe
  ======================================*/
class MQTTSubscribe : public MQTTMessage{
public:
	MQTTSubscribe();
	~MQTTSubscribe();
	void setMessageId(uint16_t);
	uint16_t getMessageId();
	uint16_t serialize(uint8_t* buf);
	void setTopic(string* topic, uint8_t qos);
private:
	//string _topic;
	uint8_t _qos;
};

/*=====================================
         Class MQTTConnect
  ======================================*/
class MQTTConnect : public MQTTMessage{
public:
	MQTTConnect();
	~MQTTConnect();
	void setProtocol(uint8_t);
	void setUserName(string*);
	void setPassword(string*);
	void setKeepAliveTime(uint16_t);
	void setWillMessage(string*);
	void setWillTopic(string*);
	void setWillQos(uint8_t);
	void setClientId(string*);
	void setCleanSessionFlg();
	uint16_t serialize(uint8_t* buf);
private:
	uint8_t _connectFlags;
	uint16_t _keepAliveTime;
	string _userName;
	string _password;
	string _willTopic;
	string _willMessage;
	string _clientId;
	uint8_t _protocol;
};

/*=====================================
         Class MQTTPublish
  ======================================*/
class MQTTPublish : public MQTTMessage{
public:
	MQTTPublish();
	~MQTTPublish();
	void setMessageId(uint16_t);
	void setPayload(uint8_t*, uint8_t);
	void setTopic(string*);
	string* getTopic();
	uint16_t getMessageId();
	uint8_t* getPayload();
	uint8_t  getPayloadLength();

	uint16_t serialize(uint8_t* buf);
	bool deserialize(uint8_t* buf);

private:
	//string _topic;
	uint16_t _len;
};

#endif /* MESSAGES_H_ */
