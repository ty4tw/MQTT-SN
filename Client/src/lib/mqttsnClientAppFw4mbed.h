/*
 * mqttsnClientAppFw4mbed.h
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

#ifdef MBED

#ifndef MQTTSNCLIENTAPPLICATION_H_
#define MQTTSNCLIENTAPPLICATION_H_

#include "mqttsnClient.h"


extern uint16_t getUint16(uint8_t* pos);
extern uint32_t getUint32(uint8_t* pos);
extern void setUint16(uint8_t* pos, uint16_t val);
extern void setUint32(uint8_t* pos, uint32_t val);

/*======================================
               Class WdTimer
========================================*/
class WdTimer:public XTimer {
public:
	WdTimer(void);
	int registerCallback(uint32_t sec, int (*proc)(void));
	void refleshRegisterTable();
	void start(void);
	void stop(void);
	bool wakeUp(void);

private:
	MQ_TimerTbl *_timerTbls;
	uint8_t _timerCnt;
	bool _initFlg;
};


/*======================================
       Class MqttsnClientApplication
========================================*/
class MqttsnClientApplication{
public:
	MqttsnClientApplication();
	~MqttsnClientApplication();
	void refleshWdtCallbackTable();
	void initialize(APP_CONFIG config);
	void setSubscribe();
	void setKeepAlive(uint16_t msec);
	void setWillTopic(MQString* willTopic);
	void setWillMessage(MQString* willMsg);
	void setRetain(bool retain);
	void setClean(bool clean);
	void setClientId(MQString* id);

	int publish(MQString* topic, const char* data, int dataLength, uint8_t qos = 1);
	int publish(uint16_t predefinedId, const char* data, int dataLength, uint8_t qos = 1);
	int publish(MQString* topic, Payload* payload, uint8_t qos);
	int subscribe(MQString* topic, TopicCallback callback, uint8_t qos = 1);
	int subscribe(uint16_t predefinedId, TopicCallback callback, uint8_t qos = 1);
	int unsubscribe(MQString* topic);
	int disconnect(uint16_t duration);

	void addTask();
	void startWdt();
	void stopWdt();
	int  run();


private:
	MqttsnClient _mqttsn;
	WdTimer _wdTimer;
	XTimer _keepAliveTimer;
	XTimer _advertiseTimer;

};

extern MqttsnClientApplication* theApplication;



#else

#endif /*LINUX*/




#endif /* MQTTSCLIENTAPPLICATION_H_ */


