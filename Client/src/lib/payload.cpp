/*
 * payload.cpp
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
 *  Created on: 2014/09/01
 *    Modified:
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 0.0.0
 */

#ifndef ARDUINO
	#include "MQTTSN_Application.h"
	#include "mqUtil.h"
	#include "payload.h"
#else
	#include <MQTTSN_Application.h>
	#include <mqUtil.h>
	#include <payload.h>
#endif
/* ARDUINO */

#ifdef MBED
	#include "mbed.h"
#endif /* MBED */

#ifdef LINUX
        #include <sys/time.h>
        #include <sys/types.h>
        #include <stdlib.h>
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
	_buff = msg->getData();
	_pos = _buff;
	_len = msg->getDataLength();
}


int Payload::getAvailableLength(){
	return _len - (_pos - _buff);
}

int Payload::getLen(){
	return _pos - _buff;
}

uint8_t* Payload::getBuf(){
	return _buff;
}

/*======================
 *     setter
 ======================*/
void Payload::set_uint32(uint32_t val){
	*_pos++ = MSGPACK_UINT32;
	setUint32(_pos, val);
	_pos += 4;
	_elmCnt++;
}

void Payload::set_int32(int32_t val){
	if((val > -32) && (val < 0)){
		*_pos++ = val | MSGPACK_NEGINT;
	}else if((val >= 0) && (val < 128)){
		*_pos++ = val | MSGPACK_POSINT;
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
}

void Payload::set_float(float val){
	*_pos++ = MSGPACK_FLOAT32;
	setFloat32(_pos, val);
	_pos += 4;
	_elmCnt++;
}

void Payload::set_str(char* val){
	set_str((const char*) val);
}

void Payload::set_str(const char* val){
	if(strlen(val) < 32){
		*_pos++ = (uint8_t)strlen(val) | MSGPACK_FIXSTR;
	}else{
		*_pos++ = MSGPACK_STR8;
		*_pos++ = (uint8_t)strlen(val);
	}
	memcpy(_pos, val, strlen(val));
	_pos += strlen(val);
}

/*======================
 *     getter
 ======================*/
uint32_t Payload::get_uint32(uint8_t index){
	uint8_t* val = getBufferPos(index);
	if(*val == MSGPACK_UINT32){
		return getUint32(val + 1);
	}else{
		return 0;
	}
}

int32_t Payload::get_int32(uint8_t index){
	int32_t rc = 0;
	uint8_t* val = getBufferPos(index);
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
	return rc;
}

float Payload::get_float(uint8_t index){
	uint8_t* val = getBufferPos(index);
	if(*val == MSGPACK_FLOAT32){
		return getFloat32(val + 1);
	}
	return 0;

}

const char* Payload::get_str(uint8_t index, uint16_t* len){
	uint8_t* val = getBufferPos(index);
	if(*val == MSGPACK_STR8){
		*len = *(val + 1);
		return (const char*)(val + 2);
	}else if((*val & MSGPACK_FIXSTR) == MSGPACK_FIXSTR){
		*len = *(val) & (~MSGPACK_FIXSTR);
		return (const char*)(val + 1);
	}
	*len = 0;
	return (const char*) 0;

}


uint8_t* Payload::getBufferPos(uint8_t index){
	uint8_t* bpos = 0;
	uint8_t* pos = _buff;
	if(index > _elmCnt){
		return 0;
	}else{
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
			default:
				if(((*pos & MSGPACK_POSINT) ==  MSGPACK_POSINT) ||
					((*pos & MSGPACK_NEGINT) == MSGPACK_NEGINT)) {
					pos++;
				}else if((*pos & MSGPACK_FIXSTR) == MSGPACK_FIXSTR){
					pos += *pos & (~MSGPACK_FIXSTR);
				}
			}
		}
		return bpos;
	}
}

void Payload::print(){
	for(int i = 0; i < _len; i++){
		printf(" 0x%x", *(_buff + i));
	}
	printf("\n");
}
