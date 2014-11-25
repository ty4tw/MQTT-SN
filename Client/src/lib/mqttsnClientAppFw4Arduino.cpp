/*
 * mqttsnClientAppAppFw4Arduino.cpp
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
#include <MQTTSN_Application.h>
#include <mqttsnClientAppFw4Arduino.h>
#include <avr/power.h>
#include <avr/sleep.h>

#if defined(MQTTSN_DEBUG) || defined(ZBEE_DEBUG)
	#include <SoftwareSerial.h>
	extern SoftwareSerial debug;
#endif

using namespace std;
using namespace tomyClient;

volatile uint8_t MQ_ErrFlag = MQ_OFF;
volatile uint8_t MQ_cnt = 0;

enum MQ_INT_STATUS MQ_intStat;
enum MQ_INT_STATUS MQ_wdtStat;

extern uint32_t getUint32(uint8_t* pos);

extern APP_CONFIG theAppConfig;
extern TaskList theTaskList[];
extern void interruptCallback(void);

MqttsnClientApplication* theApplication = new MqttsnClientApplication();

void loop(){
	theApplication->registerInt0Callback(interruptCallback);
	theApplication->addTask();

	theApplication->setKeepAlive(theAppConfig.mqttsnCfg.keepAlive);
	theApplication->setClean(theAppConfig.mqttsnCfg.cleanSession);
	if(theAppConfig.mqttsnCfg.willTopic){
		MQString* willTpc = new MQString(theAppConfig.mqttsnCfg.willTopic);
		theApplication->setWillTopic(willTpc);
	}
	if(theAppConfig.mqttsnCfg.willMsg){
		MQString* willMsg = new MQString(theAppConfig.mqttsnCfg.willMsg);
		theApplication->setWillMessage(willMsg);
	}
	if(theAppConfig.mqttsnCfg.endDevice){
		theApplication->setZBPinHibernate();
	}
	theApplication->initialize(theAppConfig);

	if(theAppConfig.mqttsnCfg.endDevice){
		theApplication->startSleepMode();
	}else{
		theApplication->startWdt();
	}

	while(true){
		theApplication->exec();
	}
}

/*-------------------------------
 * Set WDT Interrupt procedure
 --------------------------------*/
ISR (WDT_vect) {
	cli();
	MCUSR = 0;
	wdt_reset();
	WDTCSR |= B00011000;   // WDCE:on, WDE:on
	WDTCSR = 0x00;
	MQ_wdtStat = INT_WDT;
	sei();
}

/*-------------------------------
 * INT0 Interrupt procedure
 --------------------------------*/
void MQInt0(){
	detachInterrupt(0);
	MQ_intStat = INT0_LL;
}

/*-------------------------------
 * Set WDT
 --------------------------------*/
void MQwatchdogEnable(){  // Turn on WDT
	cli();
	MCUSR = 0;
	wdt_reset();
	WDTCSR |= B00011000;   // WDCE:on, WDE:on
	WDTCSR = MQ_WDT_TIME;
	MQ_wdtStat = WAIT;
	sei();
}

/*--------------------------------
   Dummy function
---------------------------------*/
void IntHandleDummy(){
}

/*--------------------------------
        reset Arduino
---------------------------------*/
//void (*resetArduino)(void) = 0;
void resetArduino(){
	asm volatile("jmp 0000");
}

/*--------------------------------
        set UnixTime
---------------------------------*/
int setUTC(MqttsnPublish* msg){
	XTimer::setUnixTime(getUint32(msg->getData()));
	return 0;
}

/*========================================
		Class MqttsnClientApplication
=========================================*/
MqttsnClientApplication::MqttsnClientApplication(){
    _txFlag = false;
    _intHandler = IntHandleDummy;
    _sleepFlg = false;
    _deviceType = ZB_ROUTER_DEVICE;

}

MqttsnClientApplication::~MqttsnClientApplication(){

}

void MqttsnClientApplication::addTask(){
	for(int i = 0; theTaskList[i].callback; i++){
		_wdTimer.registerCallback(theTaskList[i].sec, theTaskList[i].callback);
	}
}

void MqttsnClientApplication::setSubscribe(){
	_mqttsn.setSubscribing(true);  // re-entrant control
	_mqttsn.subscribe();
    _mqttsn.subscribe(MQTTSN_TOPICID_PREDEFINED_TIME, setUTC,1);
    _mqttsn.setSubscribing(false);
}

void MqttsnClientApplication::initialize(APP_CONFIG config){
	blinkIndicator(100);
    _mqttsn.initialize(config);
    XTimer::initialize();
    setSubscribe();
}

int MqttsnClientApplication::exec(){
	wakeupXB();
	_mqttsn.readPacket(); //ModemeStatus or Stored packet
	_mqttsn.readPacket();
	checkInterupt();   // WDT routines are executed

	int rc = _mqttsn.exec();
	if(rc == MQTTSN_ERR_REBOOT_REQUIRED){
		resetArduino();
	}else if(rc == MQTTSN_ERR_NO_ERROR){
		sleepXB();
		sleepApp();   // waiting WDT interruption
	}
	return 0;
}


void MqttsnClientApplication::setKeepAlive(uint16_t sec){
    _mqttsn.setKeepAlive(sec);
}


void MqttsnClientApplication::setWillTopic(MQString* willTopic){
    _mqttsn.setWillTopic(willTopic);
}

void MqttsnClientApplication::setWillMessage(MQString* willMsg){
    _mqttsn.setWillMessage(willMsg);
}

void MqttsnClientApplication::setRetain(bool retain){
    _mqttsn.setRetain(retain);
}

void MqttsnClientApplication::setClean(bool clean){
    _mqttsn.setClean(clean);
}

/*----------- Sleep related functions ------------*/
void MqttsnClientApplication::startWdt(){
    _wdTimer.start();
}

void MqttsnClientApplication::stopWdt(){
    _wdTimer.stop();
}

void MqttsnClientApplication::sleepApp(){
	if(_deviceType == ZB_PIN_HIBERNATE && _sleepFlg){
		set_sleep_mode(SLEEP_MODE_PWR_SAVE);
		MQwatchdogEnable();
		sleep_enable();
		MQ_wdtStat = WAIT;
		sleep_mode();      // waiting WDT interrupt
		sleep_disable();
		XTimer::setStopTimeDuration(MQ_WDT_TIME_MSEC);
	}
}

void MqttsnClientApplication::sleepXB(){
	if(_deviceType == ZB_PIN_HIBERNATE && _sleepFlg){
		pinMode(XB_SLEEP_PIN, INPUT);
	}
}

void MqttsnClientApplication::wakeupXB(){
	if(_deviceType == ZB_PIN_HIBERNATE){
		pinMode(XB_SLEEP_PIN, OUTPUT);
		digitalWrite(XB_SLEEP_PIN, LOW);
		delay(20);
	}
}

void MqttsnClientApplication::startSleepMode(){
	if(_deviceType == ZB_PIN_HIBERNATE){
		_sleepFlg = true;
		MQ_intStat = WAIT;
		MQ_wdtStat = WAIT;
		setInterrupt();
	}else{
		_sleepFlg = false;
	}
}

void MqttsnClientApplication::setZBPinHibernate(){
	_deviceType = ZB_PIN_HIBERNATE;
	pinMode(MQ_INT0_PIN,INPUT_PULLUP);
	wakeupXB();
	_mqttsn.getClientStatus()->setModeSleep();
}

/*-------------- Interrupt related functions ------*/
void MqttsnClientApplication::checkInterupt(){

    // interrupt Event
    if (MQ_intStat == INT0_LL){
        MQ_intStat = INT0_WAIT_HL;
        interruptHandler();
        setInterrupt();
    }
    // WDT event
    if (MQ_wdtStat == INT_WDT){
		_wdTimer.wakeUp();    // check interval & execute callback
        _wdTimer.start();     // WDT restart
    }
}

void MqttsnClientApplication::interruptHandler(){
    wakeupXB();
    _intHandler();
    sleepXB();
}

void MqttsnClientApplication::setInterrupt(){
    if (MQ_intStat == INT0_WAIT_HL){
		while(digitalRead(MQ_INT0_PIN) == 0){
				// wait LL to HL
		}
    }
    MQ_intStat = WAIT;
    attachInterrupt(0,MQInt0,LOW);
}


/*--------------------  MQTT-SN functions ---------------*/

int MqttsnClientApplication::registerTopic(MQString* topic){
    return _mqttsn.registerTopic(topic);
}

int MqttsnClientApplication::publish(MQString* topic, const char* data, int dataLength, uint8_t qos){
	return _mqttsn.publish(topic, data, dataLength, qos);
}

int MqttsnClientApplication::publish(MQString* topic, Payload* payload, uint8_t qos){
	return _mqttsn.publish(topic, payload, qos);
}

int MqttsnClientApplication::subscribe(MQString* topic, TopicCallback callback, uint8_t qos){
    return _mqttsn.subscribe(topic, callback,qos);
}

int MqttsnClientApplication::subscribe(uint16_t predefinedId, TopicCallback callback, uint8_t qos){
    return _mqttsn.subscribe(predefinedId, callback, qos);
}

int MqttsnClientApplication::publish(uint16_t predefinedId, const char* data, int dataLength, uint8_t qos){
    return _mqttsn.publish(predefinedId, data, dataLength, qos);
}

int MqttsnClientApplication::unsubscribe(MQString* topic){
    return _mqttsn.unsubscribe(topic);
}

int MqttsnClientApplication::disconnect(uint16_t duration){
    return _mqttsn.disconnect(duration);
}

/*------------- UTC functions -------------*/
uint32_t MqttsnClientApplication::getUnixTime(){
	return XTimer::getUnixTime();
}
/*
#define LEAP_YEAR(Y)  ( ((1970+Y)>0) && !((1970+Y)%4) && ( ((1970+Y)%100) || !((1970+Y)%400) ) )
static const uint8_t monthDays[]={31,28,31,30,31,30,31,31,30,31,30,31};

void MqttsnClientApplication::getDateTime(char* buf) {
	uint16_t year;
	uint8_t month;
	uint8_t mLen;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint16_t days;
	uint32_t utc;

	utc = getUnixTime();

	sec = utc % 60;
	utc /= 60;
	min = utc % 60;
	utc /= 60;
	hour = utc % 24;
	utc /= 24;

	year = days = 0;
	while((unsigned)(days += (LEAP_YEAR(year) ? 366 : 365)) <= utc) {
	    year++;
	}
	days -= LEAP_YEAR(year) ? 366 : 365;
	year += 1970;
	utc  -= days;

	days = 0;
	for (month = 0; month < 12; month++) {
		if (month == 1) {
		  if (LEAP_YEAR(year)) {
			  mLen = 29;
		  } else {
			  mLen = 28;
		  }
		} else {
			mLen = monthDays[month];
		}
		if (utc >= mLen) {
		  utc -= mLen;
		} else {
			break;
		}
	}
	month += 1;
	day = utc + 1;

	sprintf(buf,"GMT%4d%2d%2d%2d%2d%2d",year,month,day,hour,min,sec);
	char* pos = buf + 7;
	for(int i = 0; i < 10; i++){
		if(*( pos + i) == 0x20){
			*(pos + i) = 0x30;
		}
	}
}
*/

/*-------------- Indicator --------------*/
void MqttsnClientApplication::indicatorOn(){
    digitalWrite(MQ_LED_PIN,MQ_ON);
}

void MqttsnClientApplication::indicatorOff(){
    digitalWrite(MQ_LED_PIN,MQ_OFF);
}

void MqttsnClientApplication::blinkIndicator(int msec){
    digitalWrite(MQ_LED_PIN,MQ_ON);
    delay(msec);
    digitalWrite(MQ_LED_PIN,MQ_OFF);
}


/*-------------- Callback related functions ---------------*/
void MqttsnClientApplication::registerInt0Callback(void (*callback)()){
    _intHandler = callback;
}

void MqttsnClientApplication::refleshWdtCallbackTable(){
  _wdTimer.refleshRegisterTable();
}

/*======================================
               Class WdTimer
========================================*/
WdTimer::WdTimer(void) {
    _timerTbls = 0;
    _timerCnt = 0;
    _initFlg = true;

}

void WdTimer::start(void) {    
	MQwatchdogEnable();
}


void WdTimer::stop(void){
    cli();
    MCUSR = 0;
    wdt_reset();
    WDTCSR |= B00011000;   // WDCE:on, WDE:on
    WDTCSR = 0x00;
    MQ_wdtStat = WAIT;
    sei();
}


bool WdTimer::wakeUp(void){
    bool rcflg = false;
    int rc;
	for(uint8_t i = 0; i < _timerCnt; i++) {
		if ((_timerTbls[i].prevTime + _timerTbls[i].interval < getUnixTime()) || _initFlg){
			rc = (_timerTbls[i].callback)();
			if(rc == MQTTSN_ERR_REBOOT_REQUIRED || rc == MQTTSN_ERR_INVALID_TOPICID){
				resetArduino();
			}
			_timerTbls[i].prevTime = getUnixTime();
			rcflg = true;
		}
	}
	_initFlg = false;
    return rcflg;
}

int WdTimer::registerCallback(uint32_t sec, int (*callback)(void)){
    MQ_TimerTbl *savTbl = _timerTbls;
    MQ_TimerTbl *newTbl = (MQ_TimerTbl*)calloc(_timerCnt + 1,sizeof(MQ_TimerTbl));

    if ( newTbl != 0 ) {
        _timerTbls = newTbl;
        for(uint8_t i = 0; i < _timerCnt; i++ ){
			_timerTbls[i].prevTime = savTbl[i].prevTime;
			_timerTbls[i].interval = savTbl[i].interval;
			_timerTbls[i].callback = savTbl[i].callback;
        }
        free(savTbl);

        //_timerTbls[_timerCnt].prevTime = getUnixTime();
        _timerTbls[_timerCnt].prevTime = (uint32_t)0;
        _timerTbls[_timerCnt].interval = sec;
        _timerTbls[_timerCnt].callback = callback;
        _timerCnt++;
        return MQTTSN_ERR_NO_ERROR;
    }
    return MQTTSN_ERR_OUT_OF_MEMORY;
} 

void WdTimer::refleshRegisterTable(){
    for(uint8_t i = 0; i < _timerCnt; i++) {
        _timerTbls[i].prevTime = getUnixTime();
    }
}



#endif  /* ARDUINO */
