/*
 * mqUtil.cpp
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
        #include "mqUtil.h"
		#include "stdio.h"
		#include "string.h"
#else
        #include <MQTTSN_Application.h>
        #include <mqUtil.h>
		#include <stdio.h>
		#include <string.h>
#endif  /* ARDUINO */

#ifdef MBED
        #include "mbed.h"
#endif /* MBED */


using namespace std;
using namespace tomyClient;

/*=====================================
        Global functions
 ======================================*/

#ifdef CPU_LITTLEENDIANN

/*--- For Little endianness ---*/

uint16_t getUint16(uint8_t* pos){
	uint16_t val = ((uint16_t)*pos++ << 8);
	return val += *pos;
}

void setUint16(uint8_t* pos, uint16_t val){
    *pos++ = (val >> 8) & 0xff;
	*pos   = val & 0xff;
}

uint32_t getUint32(uint8_t* pos){
    uint32_t val = uint32_t(*pos++) <<  24;
	val += uint32_t(*pos++) << 16;
	val += uint32_t(*pos++) <<  8;
	return val += *pos++;
}

void setUint32(uint8_t* pos, uint32_t val){
	*pos++ = (val >> 24) & 0xff;
	*pos++ = (val >> 16) & 0xff;
	*pos++ = (val >>  8) & 0xff;
	*pos   =  val & 0xff;
}

float getFloat32(uint8_t* pos){
	union{
		float flt;
		uint8_t d[4];
	}val;
    val.d[3] = *pos++;
	val.d[2] = *pos++;
	val.d[1] = *pos++;
	val.d[0] = *pos;
	return val.flt;
}

void setFloat32(uint8_t* pos, float flt){
	union{
		float flt;
		uint8_t d[4];
	}val;
	val.flt = flt;
    *pos++ = val.d[3];
    *pos++ = val.d[2];
    *pos++ = val.d[1];
    *pos   = val.d[0];
}

#endif  // CPU_LITTLEENDIANN

/*--- For Big endianness ---*/
#ifdef CPU_BIGENDIANN

uint16_t getUint16(uint8_t* pos){
  uint16_t val = *pos++;
  return val += ((uint16_t)*pos++ << 8);
}

void setUint16(uint8_t* pos, uint16_t val){
	*pos++ =  val & 0xff;
	*pos   = (val >>  8) & 0xff;
}

uint32_t getUint32(uint8_t* pos){
    long val = uint32_t(*(pos + 3)) << 24;
    val += uint32_t(*(pos + 2)) << 16;
	val += uint32_t(*(pos + 1)) <<  8;
	return val += *pos;
}

void setUint32(uint8_t* pos, uint32_t val){
    *pos++ =  val & 0xff;
    *pos++ = (val >>  8) & 0xff;
    *pos++ = (val >> 16) & 0xff;
    *pos   = (val >> 24) & 0xff;
}

float getFloat32(uint8_t* pos){
	union{
		float flt;
		uint8_t d[4];
	}val;

    val.d[0] = *pos++;
	val.d[1] = *pos++;
	val.d[2] = *pos++;
	val.d[3] = *pos;
	return val.flt;
}

void setFloat32(uint8_t* pos, float flt){
	union{
		float flt;
		uint8_t d[4];
	}val;
	val.flt = flt;
    *pos++ = val.d[0];
    *pos++ = val.d[1];
    *pos++ = val.d[2];
    *pos   = val.d[3];
}

#endif  // CPU_BIGENDIANN

/*=========================================
             Class XBeeTimer
 =========================================*/

#ifdef ARDUINO
/**
 *   for Arduino
 */
uint32_t XTimer::_unixTime;
uint32_t XTimer::_epochTime;
uint32_t XTimer::_timerStopTimeAccum;
bool    XTimer::_utcFlag;


XTimer::XTimer(){
    stop();
}

void XTimer::initialize(){
	_unixTime = 0;
	_epochTime = 0;
	_timerStopTimeAccum = 0;
	_utcFlag = false;
}

void XTimer::start(uint32_t msec){
    _startTime = millis();
    _millis = msec;
    _currentTime = 0;
	_timeupUnixTime =  getUnixTime() + msec / 1000;
}

bool XTimer::isTimeUp(){
    return isTimeUp(_millis);
}

bool XTimer::isTimeUp(uint32_t msec){
	if(_utcFlag){
		_utcFlag = false;
		if(_timeupUnixTime > 1000000000 && _unixTime){
			return (getUnixTime() >= _timeupUnixTime);
		}else{
			return false;
		}
	}else{
		if ( _startTime){
			_currentTime = millis();
			if ( _currentTime < _startTime){
				return (0xffffffff - _startTime + _currentTime > msec);
			}else{
				return (_currentTime - _startTime > msec);
			}
		}else{
			return false;
		}
	}
}

void XTimer::stop(){
    _startTime = 0;
    _millis = 0;
    _currentTime = 0;
    _timeupUnixTime = 0;
}

void XTimer::setUnixTime(uint32_t utc){
    _epochTime = millis();
    _timerStopTimeAccum = 0;
    _unixTime = utc;
}

uint32_t XTimer::getUnixTime(){
    uint32_t tm = _timerStopTimeAccum + millis();
    if (_epochTime > tm ){
        return _unixTime + (uint32_t)((0xffffffff - tm - _epochTime) / 1000);
    }else{
        return _unixTime + (uint32_t)((tm - _epochTime) / 1000);
    }
}

void XTimer::setStopTimeDuration(uint32_t msec){
	_timerStopTimeAccum += msec;
}

void XTimer::changeUTC(){
	_utcFlag = true;
}

#endif


#ifdef MBED
/**
 *   for MBED
 */
XTimer::XTimer(){
    stop();
}

void XTimer::start(uint32_t msec){
    _timer.start();
    _millis = msec;
    _timeupTime = time(0) + (uint32_t)(msec/1000UL);
}

bool XTimer::isTimeUp(){
    return isTimeUp(_millis);
}

bool XTimer::isTimeUp(uint32_t msec){
	if(_utcFlg){
		if(_timeupTime > 100000000){
			return( time(0) > _timeupTime);
		}else{
			return false;
		}
	}else{
		return _timer.read_ms() > msec;
	}
}

void XTimer::stop(){
    _timer.stop();
    _millis = 0;
    _timeupTime = 0;
}

void XTimer::changeUTC(){
	_utcFlg = true;
}

#endif

#ifdef LINUX
/**
 *   for LINUX
 */
XTimer::XTimer(){
	_startTime.tv_sec = 0;
	_millis = 0;
}

XTimer::~XTimer(){

}

void XTimer::start(uint32_t msec){
  gettimeofday(&_startTime, 0);
  _millis = msec;
}

bool XTimer::isTimeUp(void){
  return isTimeUp(_millis);
}

bool XTimer::isTimeUp(uint32_t msec){
    struct timeval curTime;
    uint32_t secs, usecs;
    if (_startTime.tv_sec == 0){
        return false;
    }else{
        gettimeofday(&curTime, 0);
        secs  = (curTime.tv_sec  - _startTime.tv_sec) * 1000;
        usecs = (curTime.tv_usec - _startTime.tv_usec) / 1000.0;
        return ((secs + usecs) > (uint32_t)msec);
    }
}

void XTimer::stop(){
  _startTime.tv_sec = 0;
  _millis = 0;
}

#endif
