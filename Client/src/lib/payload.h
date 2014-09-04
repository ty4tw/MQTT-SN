/*
 * payload.h
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

#ifndef PAYLOAD_H_
#define PAYLOAD_H_

#ifndef ARDUINO
  	  #include "MQTTSN_Application.h"
	  #include "mqttsn.h"
#else
  	  #include <MQTTSN_Application.h>
  	  #include <mqttsn.h>
#endif  /* ARDUINO */

#ifdef MBED
  #include "mbed.h"
#endif  /* MBED */

#ifdef LINUX
  #include <stdio.h>
  #include <sys/time.h>
  #include <sys/types.h>
  #include <sys/stat.h>
  #include <unistd.h>
  #include <stdlib.h>
  #include <string.h>
#endif /* LINUX */


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

/*=====================================
        Class Payload
  =====================================*/
class MqttsnPublish;

class Payload{
public:
	Payload();
	Payload(uint16_t len);
	~Payload();
	void init(void);
	void set_uint32(uint32_t val);
	void set_int32(int32_t val);
	void set_float(float val);
	void set_str(char* val);
	void set_str(const char* val);

	uint32_t get_uint32(uint8_t index);
	int32_t  get_int32(uint8_t index);
    float    get_float(uint8_t index);
    const char* get_str(uint8_t index, uint16_t* len);

	void 	 getPayload(MqttsnPublish* msg);
	int      getAvailableLength();
	int      getLen();
	uint8_t* getBuf();

void print();
private:
	uint8_t* getBufferPos(uint8_t index);
	uint8_t* _buff;
	int      _len;
	uint8_t  _elmCnt;
	uint8_t* _pos;
	uint8_t  _memDlt;
};

#endif  // PAYLOAD_H_
