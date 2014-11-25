/*
 * mqttsnClientAppFw4Arduino.h
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

#ifdef ARDUINO

#ifndef MQTTSNCLIENTAPPLICATION_H_
#define MQTTSNCLIENTAPPLICATION_H_

#include <MQTTSN_Application.h>
#include <mqttsnClient.h>
#include <mqUtil.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>

#define MQ_LED_PIN  13
#define MQ_INT0_PIN 2
#define MQ_SLEEP_PIN 4  // Connect to XBee DTR for hibernation mode
#define MQ_ERROR_RECOVERY_DURATION_ON 8
#define MQ_ON      1
#define MQ_OFF     0

#define ZB_ROUTER_DEVICE  0
#define ZB_PIN_HIBERNATE 1


#define MQ_WDT_ERR   (B01100000)  // Error Indication time

//#define MQ_WDT_TIME (B01000111)   // 2 Sec
//#define MQ_WDT_TIME_MSEC   2000

//#define MQ_WDT_TIME (B01100000)   // 4 Sec
//#define MQ_WDT_TIME_MSEC   4000
 
#define MQ_WDT_TIME (B01100001)   // 8 Sec
#define MQ_WDT_TIME_MSEC   8000


enum MQ_INT_STATUS{ WAIT, INT0_LL, INT0_WAIT_HL, INT_WDT};


#define INDICATOR_ON()   theApplication->indicatorOn()
#define INDICATOR_OFF()  theApplication->indicatorOff()
#define BLINK_INDICATOR(...) theApplication->blinkIndicator(__VA_ARGS__)
#define GETUTC()         theApplication->getUnixTime()
//#define GET_DATETIME(...) theApplication->getDateTime(__VA_ARGS__)


extern void setUint32(uint8_t*, uint32_t);
extern uint32_t getUint32(uint8_t*);
extern void setUint16(uint8_t*, uint16_t);
extern uint16_t getUint16(uint8_t*);


/*======================================
               Class WdTimer
========================================*/
class WdTimer:public XTimer {
public:
	WdTimer(void);
	int  registerCallback(uint32_t sec, int (*proc)(void));
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
	void registerInt0Callback(void (*callback)());
	void registerWdtCallback(long sec, int (*callback)(void));
	void refleshWdtCallbackTable();
	void initialize(APP_CONFIG config);
	void setSubscribe();
	void setKeepAlive(uint16_t msec);
	void setWillTopic(MQString* willTopic);
	void setWillMessage(MQString* willMsg);
	void setRetain(bool retain);
	void setClean(bool clean);
	void setClientId(MQString* id);
	void setZBPinHibernate();
	void blinkIndicator(int msec);
	void startSleepMode();
	void subscribe(void);
	void addTask(void);
	
	int registerTopic(MQString* topic);
	int publish(MQString* topic, const char* data, int dataLength, uint8_t qos = 1);
	int publish(uint16_t predefinedId, const char* data, int dataLength, uint8_t qos = 1);
	int publish(MQString* topic, Payload* payload, uint8_t qos);
	int subscribe(MQString* topic, TopicCallback callback, uint8_t qos = 1);
	int subscribe(uint16_t predefinedId, TopicCallback callback, uint8_t qos = 1);
	int unsubscribe(MQString* topic);
	int disconnect(uint16_t duration);

	void startWdt();
	void stopWdt();
	int  exec();
	void setUnixTime(MqttsnPublish* msg);
	uint32_t getUnixTime();
	void getDateTime(char* buf);
	void reboot();
	void indicatorOn();
	void indicatorOff();

private:
	void checkInterupt();
	void interruptHandler();
    void setInterrupt();
	void sleepXB();
	void wakeupXB();
	void sleepApp();
	
	MqttsnClient _mqttsn;
	bool _txFlag;
	bool  _sleepFlg;
	uint8_t _deviceType;

	WdTimer _wdTimer;
	XTimer _keepAliveTimer;
	XTimer _advertiseTimer;

	void (*_intHandler)(void);

};

extern MqttsnClientApplication* theApplication;

#else

#endif /*ARDUINO*/




#endif /* MQTTSNCLIENTAPPLICATION_H_ */
