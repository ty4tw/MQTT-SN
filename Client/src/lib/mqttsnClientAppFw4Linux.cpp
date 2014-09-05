/*
 * mqttsnClientAppAppFwLinux.cpp
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

#ifdef LINUX
#include "mqttsnClientAppFw4Linux.h"
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>

using namespace std;
using namespace tomyClient;

MqttsnClientApplication* theApplication = new MqttsnClientApplication();

extern TaskList theTaskList[];
extern void  setup();
extern MQString theTopics[];

#ifdef NETWORK_XBEE
XBeeAppConfig  theAppConfig = { { 0, 0, 0 },{ 0, 0, false, false, 0, 0 } };
#endif

#ifdef NETWORK_UDP
UdpAppConfig   theAppConfig = {{ {0,0,0,0}, 0, {0,0,0,0}, 0, {0,0,0,0,0,0} },{ 0, 0, false, false, 0, 0 } };
#endif


/*========================================
		main function
=========================================*/
int main(int argc, char **argv){
	theApplication->addTask();
	setup();
	theApplication->initialize(argc, argv);
	theApplication->run();
	return 0;
}

/*========================================
		Class MqttsnClientApplication
=========================================*/
MqttsnClientApplication::MqttsnClientApplication(){

}

MqttsnClientApplication::~MqttsnClientApplication(){

}


void MqttsnClientApplication::startWdt(){
    _wdTimer.start();
}

void MqttsnClientApplication::stopWdt(){
    _wdTimer.stop();
}

/*------------ Client execution  forever --------------*/
int MqttsnClientApplication::run(){
	while(true){
		_wdTimer.wakeUp();
		_mqttsn.readPacket();
		int rc = _mqttsn.exec();
		if(rc == MQTTSN_ERR_REBOOT_REQUIRED){
			_mqttsn.subscribe();
		}
	}
	return 0;
}


void MqttsnClientApplication::initialize(int argc, char** argv){
	int arg;
	unsigned int val;
	char* id = 0;
	MQString* willTopic;
	MQString* willMsg;
#ifdef NETWORK_XBEE
	unsigned int br = B57600;
	char* dev = 0;
#endif
#ifdef NETWORK_UDP
	char* ipAddr = 0;
	uint16_t gPortNo = 0;
	uint16_t uPortNo = 0;
#endif

	while((arg = getopt(argc, argv, "hcb:d:u:i:k:t:m:g:p:"))!= -1){
		switch(arg){
		case 'h':
			printf("Usage:  -b: [baudrate]      (XBee)\n");
			printf("        -d: [device]        (XBee)\n");
			printf("        -g: [groupIp]       (UDP)\n");
			printf("        -p: [group portNo]  (UDP)\n");
			printf("        -u: [client portNo] (UDP)\n");
			printf("        -c: CleanSession\n");
			printf("        -i: [ClientId]\n");
			printf("        -k: [keepAliveTime in second]\n");
			printf("        -t: [willTopic]\n");
			printf("        -m: [willMessage]\n");
			exit(0);
			break;
		case 'c':
			_mqttsn.setClean(true);
			break;

		case 'i':
			id = strdup(optarg);
			break;
		case 't':
			willTopic = new MQString((const char*)optarg);
			_mqttsn.setWillTopic(willTopic);
			break;
		case 'm':
			willMsg = new MQString((const char*)optarg);
			_mqttsn.setWillMessage(willMsg);
			break;
		case 'k':
			val = atoi(optarg);
			_mqttsn.setKeepAlive(val);
			break;
#ifdef NETWORK_XBEE
		case 'd':
			dev = strdup(optarg);
			break;
		case 'b':
			val = atoi(optarg);
			switch(val){
			case 9600:
				br = B9600;
				break;
			case 19200:
				br =B19200;
				break;
			case 38400:
				br =B38400;
				break;
			case 57600:
				br =B57600;
				break;
			default:
				printf("Invalid baudrate!\n");
				exit(-1);
			}
			break;
#endif
#ifdef NETWORK_UDP
		case 'g':
			ipAddr = optarg;
			break;
		case 'p':
			gPortNo = atoi(optarg);
			break;
		case 'u':
			uPortNo = atoi(optarg);
			break;
#endif
		case '?':
			printf("Unrecognized option! %c\n", arg);
			exit(-1);
		}
	}

#ifdef NETWORK_XBEE
	theAppConfig.netCfg.baudrate = br;
	theAppConfig.netCfg.device = strdup(dev);
#endif
#ifdef NETWORK_UDP
	if(gPortNo && ipAddr[0] && uPortNo){
		theAppConfig.netCfg.gPortNo = gPortNo;
		theAppConfig.netCfg.uPortNo = uPortNo;
		uint32_t ipaddr = inet_addr(ipAddr);
		theAppConfig.netCfg.ipAddress[0] = (ipaddr & 0xff000000) >> 24;
		theAppConfig.netCfg.ipAddress[1] = (ipaddr & 0x00ff0000) >> 16;
		theAppConfig.netCfg.ipAddress[2] = (ipaddr & 0x0000ff00) >> 8;
		theAppConfig.netCfg.ipAddress[3] = (ipaddr & 0x000000ff);
	}else{
		printf("argument error\n");
		exit(1);
	}
#endif

	if(id){
		theAppConfig.mqttsnCfg.nodeId = strdup(id);
	}else{
		theAppConfig.mqttsnCfg.nodeId ="node";
	}

	_mqttsn.initialize(theAppConfig);

	setSubscribe();
}

void MqttsnClientApplication::setSubscribe(){
	_mqttsn.setSubscribing(true);
    _mqttsn.createTopics();
	_mqttsn.subscribe();
	_mqttsn.setSubscribing(false);
}

void MqttsnClientApplication::addTask(){
	for(int i = 0; theTaskList[i].sec; i++){
		_wdTimer.registerCallback(theTaskList[i].sec, theTaskList[i].callback);
	}
}

int MqttsnClientApplication::publish(MQString* topic, const char* data, int dataLength, uint8_t qos){
	return _mqttsn.publish(topic, data, dataLength, qos);
}

int MqttsnClientApplication::publish(uint16_t predefinedId, const char* data, int dataLength, uint8_t qos){
    return _mqttsn.publish(predefinedId, data, dataLength, qos);
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


int MqttsnClientApplication::unsubscribe(MQString* topic){
    return _mqttsn.unsubscribe(topic);
}

int MqttsnClientApplication::disconnect(uint16_t duration){
    return _mqttsn.disconnect(duration);
}

void MqttsnClientApplication::setRetain(bool flag){
    _mqttsn.setRetain(flag);
}



/*-------------- Callback related functions ---------------*/


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
	//MQwatchdogEnable();
}


void WdTimer::stop(void){
    //
}


bool WdTimer::wakeUp(void){
    bool rcflg = false;
    int rc;
	for(uint8_t i = 0; i < _timerCnt; i++) {
		if ((_timerTbls[i].prevTime + _timerTbls[i].interval < (uint32_t)time(0)) || _initFlg){
			rc = (_timerTbls[i].callback)();
			if(rc == MQTTSN_ERR_REBOOT_REQUIRED){
				theApplication->setSubscribe();
			}
			_timerTbls[i].prevTime = time(0);
			rcflg = true;
		}
	}
	_initFlg = false;
    return rcflg;
}

int WdTimer::registerCallback(uint32_t sec, int (*callback)(void)){
    MQ_TimerTbl *savTbl = _timerTbls;
    MQ_TimerTbl *newTbl = (MQ_TimerTbl*)calloc((unsigned int)_timerCnt + 1,sizeof(MQ_TimerTbl));

    if ( newTbl != 0 ) {
        _timerTbls = newTbl;
        for(uint8_t i = 0; i < _timerCnt; i++ ){
			_timerTbls[i].prevTime = savTbl[i].prevTime;
			_timerTbls[i].interval = savTbl[i].interval;
			_timerTbls[i].callback = savTbl[i].callback;
        }
        free(savTbl);

        _timerTbls[_timerCnt].prevTime = time(0);
        _timerTbls[_timerCnt].interval = sec;
        _timerTbls[_timerCnt].callback = callback;
        _timerCnt++;
        return MQTTSN_ERR_NO_ERROR;
    }
    return MQTTSN_ERR_OUT_OF_MEMORY;
} 

void WdTimer::refleshRegisterTable(){
    for(uint8_t i = 0; i < _timerCnt; i++) {
        _timerTbls[i].prevTime = time(0);
    }
}


#endif
#endif
