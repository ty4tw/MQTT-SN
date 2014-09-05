/*
 * Network.cpp
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

#ifndef  NEWORK_H_
#define  NEWORK_H_

#ifndef ARDUINO
    #include "MQTTSN_Application.h"
#else
	#include <MQTTSN_Application.h>
#endif

/***********************************
 *     Send Request Type
 ***********************************/
enum SendReqType{
    NoReq = 0,
    UcastReq,
    BcastReq
};

/*=================================
 *    Network stacks
 ==================================*/
#ifdef NETWORK_XBEE
	#ifdef ARDUINO
		#include <zbeeStack.h>
	#else
		#include "zbeeStack.h"
	#endif
#endif

#ifdef NETWORK_UDP
	#ifdef ARDUINO
		#include <udpStack.h>
	#else
		#include "udpStack.h"
	#endif
#endif


namespace tomyClient {
/***********************************
 *     MQTT-SN  Client's state
 ***********************************/
enum NodeStatus {
    NdDisconnected = 0,
    NdActive,
    NdAsleep,
    NdAwaik,
    NdLost
};

#define PACKET_SENDED           0
#define PACKET_ERROR_RESPONSE  -1
#define PACKET_ERROR_UNKOWN    -2
#define PACKET_ERROR_NODATA    -3
#define PACKET_MODEM_STATUS    -4

}   /* end of namespace */

#endif  /* NEWORK_H_ */
