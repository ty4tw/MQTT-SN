/*
 * mqUtil.h
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

#ifndef XTIMER_H_
#define XTIMER_H_


#if defined(ARDUINO)
	#include <MQTTSN_Application.h>
    #if ARDUINO >= 100
        #include "Arduino.h"
        #include <inttypes.h>
    #else
        #if ARDUINO < 100
            #include "WProgram.h"
            #include <inttypes.h>
        #endif
    #endif

	#define XB_CTS_PIN   3   // XBee CTS
	#define XB_SLEEP_PIN 4   // XBee Pinhybernate
#else
	#include "MQTTSN_Application.h"
#endif /* ARDUINO */


#ifdef LINUX
    #include <sys/time.h>
#endif

#ifdef MBED
    #include "mbed.h"
#endif

namespace tomyClient {


#ifdef ARDUINO
/*============================================
       XBeeTimer for Arduino
 ============================================*/
class XTimer{
public:
    XTimer();
    ~XTimer(){};
    void start(uint32_t msec = 0);
    bool isTimeUp(uint32_t msec);
    bool isTimeUp(void);
    void stop();
    static uint32_t getUnixTime();
    static void setUnixTime(uint32_t utc);
	static void setStopTimeDuration(uint32_t msec);
    static void initialize();
    static void changeUTC();

private:
    uint32_t _startTime;
    uint32_t _currentTime;
    uint32_t _millis;
	uint32_t _timeupUnixTime;
	static uint32_t _unixTime;
	static uint32_t _epochTime;
	static uint32_t _timerStopTimeAccum;
	static bool _utcFlag;

};


#endif

#ifdef MBED
/*============================================
    XBeeTimer  for MBED
 ============================================*/
class XTimer{
public:
    XTimer();
    ~XTimer(){};
    void start(uint32_t msec = 0);
    bool isTimeUp(uint32_t msec);
    bool isTimeUp(void);
    void stop(void);
    void changeUTC(void);

private:
    Timer _timer;
    time_t _startTime;
    uint32_t _millis;
    time_t _timeupTime;
    bool _utcFlg;
};

#endif


#ifdef LINUX
/*============================================
                XBeeTimer
 ============================================*/
class XTimer{
public:
    XTimer();
    ~XTimer();
    void start(uint32_t msec = 0);
    bool isTimeUp(uint32_t msec);
    bool isTimeUp(void);
    void stop(void);
    void changeUTC(void){};
private:
    struct timeval _startTime;
    uint32_t _millis;
};

#endif
}

#endif  /* XTIMER_H_ */
