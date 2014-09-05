/*
 * MQTTSN_Application.h
 *
 *                    The BSD License
 *
 *           Copyright (c) 2014, tomoaki@tomy-tech.com
 *                   All rights reserved.
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

#ifndef MATTSNAPPLICATION_H_
#define MATTSNAPPLICATION_H_

/****************************************
      Select Platform and Network
*****************************************/

/*----------- Select Platform  ---------*/
#ifndef ARDUINO
	#define LINUX
	//#define MBED
#endif

/*--------- Select byte order ----------*/
#define CPU_LITTLEENDIANN
//#define CPU_BIGENDIANN


/*-------- Select Network  -------------*/
#define NETWORK_XBEE
//#define NETWORK_UDP


/*--- XBee Buffer Flow Control --*/
#ifdef NETWORK_XBEE
	//#define XBEE_FLOWCTL_CRTSCTS
#endif

/*======================================
 *         Debug Flag
 ======================================*/
//#define NW_DEBUG
//#define MQTTSN_DEBUG


/****************************************
      MQTT-SN Packet length
*****************************************/
#ifdef ARDUINO
	#define MQTTSN_MAX_FRAME_SIZE               70
#else
	#ifdef NETWORK_XBEE
		#define MQTTSN_MAX_FRAME_SIZE           70
	#else
		#ifdef NETWORK_UDP
			#define MQTTSN_MAX_FRAME_SIZE     1024
		#endif
	#endif
#endif
/****************************************
      Application config structures
*****************************************/
#ifdef LINUX
typedef unsigned char  uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int   uint32_t;
typedef signed char    int8_t;
typedef signed short   int16_t;
typedef signed int     int32_t;
#endif

#ifdef MBED
	#include "mbed.h"
#endif

#ifdef ARDUINO
	#include <Arduino.h>
#endif

typedef struct {
	const char* nodeId;
	uint16_t keepAlive;
	bool     cleanSession;
	bool     endDevice;
	const char* willTopic;
	const char* willMsg;
}MqttsnConfig;

typedef struct {
	long baudrate;
	uint8_t  portNo;
	char* device;
}XBeeConfig;

typedef struct {
	XBeeConfig netCfg;
	MqttsnConfig mqttsnCfg;
}XBeeAppConfig;

typedef struct {
	uint8_t  ipAddress[4];
	uint16_t gPortNo;
	uint8_t  ipLocal[4];
	uint16_t uPortNo;
	uint8_t  macAddr[6];
}UdpConfig;

typedef struct {
	UdpConfig netCfg;
	MqttsnConfig mqttsnCfg;
}UdpAppConfig;

/*======================================
      MACROs for Application
=======================================*/
#ifdef NETWORK_XBEE
    #define XBEE_APP_CONFIG       XBeeAppConfig  theAppConfig
	#define APP_CONFIG            XBeeAppConfig
	#define NETWORK_CONFIG        XBeeConfig
#endif

#ifdef NETWORK_UDP
    #define UDP_APP_CONFIG        UdpAppConfig   theAppConfig
	#define APP_CONFIG            UdpAppConfig
	#define NETWORK_CONFIG        UdpConfig
#endif

#define TASK_LIST         TaskList theTaskList[]
#define END_OF_TASK_LIST  {0,0}
#define SUBSCRIBE_LIST   OnPublishList theOnPublishList[]
#define END_OF_SUBSCRIBE_LIST {0,0,0}
#define TOPICS_IN_CALLBACK MQString* theTopics[]
#define END_OF_TOPICS    0

#define PUBLISH(...)     theApplication->publish(__VA_ARGS__)
#define SUBSCRIBE(...)   theApplication->subscribe(__VA_ARGS__)
#define UNSUBSCRIBE(...) theApplication->unsubscribe(__VA_ARGS__)
#define DISCONNECT()     theApplication->disconnect()
#define SETRETAIN(...)   theApplication->setRetain(__VA_ARGS__)

/*======================================
      MACROs for debugging
========================================*/
#ifdef ARDUINO
	#ifdef NW_DEBUG
		#define D_NWSTACK(...)    debug.print(__VA_ARGS__)
		#define D_NWSTACKLN(... ) debug.println(__VA_ARGS__)
		#define D_NWSTACKW(...)   debug.print(__VA_ARGS__)
		#define D_NWSTACKF(...)
	#else
		#define D_NWSTACK(...)
		#define D_NWSTACKLN(...)
		#define D_NWSTACKW(...)
		#define D_NWSTACKF(...)
	#endif

	#ifdef MQTTSN_DEBUG
		#define D_MQTT(...)    debug.print(__VA_ARGS__)
		#define D_MQTTW(...)   debug.print(__VA_ARGS__)
		#define D_MQTTLN(...)  debug.println(__VA_ARGS__)
		#define D_MQTTF(...)
	#else
		#define D_MQTT(...)
		#define D_MQTTLN(...)
		#define D_MQTTW(...)
		#define D_MQTTF(...)
	#endif
#else
	#ifdef NW_DEBUG
		#define D_NWSTACKF(...)    printf(__VA_ARGS__)
		#define D_NWSTACKW(...)   printf(__VA_ARGS__)
		#define D_NWSTACKLN(...)
		#define D_NWSTACK(...)
	#else
		#define D_NWSTACK(...)
		#define D_NWSTACKLN(...)
		#define D_NWSTACKW(...)
		#define D_NWSTACKF(...)
	#endif
	#ifdef MQTTSN_DEBUG
		#define D_MQTTF(...)    printf(__VA_ARGS__)
		#define D_MQTTW(...)   printf("%s",__VA_ARGS__)
		#define D_MQTTLN(...)
		#define D_MQTT(...)
	#else
		#define D_MQTT(...)
		#define D_MQTTLN(...)
		#define D_MQTTW(...)
		#define D_MQTTF(...)
		#endif
#endif

#endif /* MATTSNAPPLICATION_H_ */

