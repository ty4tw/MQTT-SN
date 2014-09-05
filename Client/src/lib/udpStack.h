/*
 * udpStack.h
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

#ifndef UDPSTACK_H_
#define UDPSTACK_H_

#ifdef ARDUINO
	#include <MQTTSN_Application.h>
	#include <mqUtil.h>
	#include <Network.h>
#else
	#include "MQTTSN_Application.h"
	#include "mqUtil.h"
	#include "Network.h"
#endif

#ifdef NETWORK_UDP

#ifdef ARDUINO
	#include <SPI.h>
	#include <Ethernet.h>
	#include <EthernetUdp.h>
    #if ARDUINO >= 100
        #include "Arduino.h"
        #include <inttypes.h>
    #else
        #if ARDUINO < 100
            #include "WProgram.h"
            #include <inttypes.h>
        #endif
    #endif
#endif /* ARDUINO */


#ifdef MBED
    #include "mbed.h"
#endif

#ifdef LINUX
    #include <sys/time.h>
    #include <iostream>
	#include <sys/types.h>
	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <netdb.h>
	#include <unistd.h>
	#include <string>
	#include <arpa/inet.h>
#endif

#define STAT_UNICAST   1
#define STAT_MULTICAST 2

#define SOCKET_MAXHOSTNAME  200
#define SOCKET_MAXCONNECTIONS  5
#define SOCKET_MAXRECV  500
#define SOCKET_MAXBUFFER_LENGTH 500 // buffer size

#define PACKET_TIMEOUT_CHECK   200  // msec

using namespace std;

namespace tomyClient {

/*============================================
              NWAddress64
 =============================================*/
class NWAddress64 {
public:
	NWAddress64(uint32_t msb, uint32_t lsb);
	NWAddress64(void);
	uint32_t getMsb();
	uint32_t getLsb();
	void setMsb(uint32_t msb);
	void setLsb(uint32_t lsb);
private:
	uint32_t _msb;
	uint32_t _lsb;
};

/*============================================
               NWResponse
 =============================================*/

class NWResponse {
public:
	NWResponse();
	bool    isAvailable();
	uint8_t  isError();
	uint8_t  getType();
	uint8_t  getFrameLength();
	uint8_t  getPayload(uint8_t index);
	uint8_t* getPayload();
	uint8_t* getBody();
	uint16_t getBodyLength();
	uint8_t  getPayloadLength();
	uint16_t getAddress16();
	NWAddress64& getAddress64();
	void setLength(uint16_t len);
//	void setType(uint8_t type);
	void setFrame(uint8_t* framePtr);
	void setAddress64(uint32_t msb, uint32_t ipAddress);
	void setAddress16(uint16_t portNo);
	void setErrorCode(uint8_t);
	void setAvailable(bool);
	void resetResponse();
private:
	NWAddress64 _addr64;
	uint16_t _addr16;
	uint16_t _len;
	uint8_t* _frameDataPtr;
	uint8_t  _type;
	uint8_t  _errorCode;
	bool    _complete;
};


/*========================================
       Class UpdPort
 =======================================*/
class UdpPort{
public:
	UdpPort();
	virtual ~UdpPort();

	bool open(NETWORK_CONFIG config);

	int unicast(const uint8_t* buf, uint32_t length, uint32_t ipaddress, uint16_t port  );
	int multicast( const uint8_t* buf, uint32_t length );
	int recv(uint8_t* buf, uint16_t len, bool nonblock, uint32_t* ipaddress, uint16_t* port );
	int recv(uint8_t* buf, uint16_t len, int flags);
	bool checkRecvBuf();
	bool isUnicast();

private:
	void close();
	int recvfrom ( uint8_t* buf, uint16_t len, int flags, uint32_t* ipaddress, uint16_t* port );

#ifdef LINUX
	int _sockfdUcast;
	int _sockfdMcast;
	uint16_t _gPortNo;
	uint16_t _uPortNo;
	uint32_t _gIpAddr;
	uint8_t  _castStat;
#endif
#ifdef ARDUINO
	EthernetUDP _udpUnicast;
	EthernetUDP _udpMulticast;
	IPAddress   _gIpAddr;
	IPAddress   _cIpAddr;
	uint16_t    _gPortNo;
	uint16_t    _uPortNo;
	uint8_t*    _macAddr;
	uint8_t     _castStat;
#endif

	bool   _disconReq;

};

#define NO_ERROR	0
#define PACKET_EXCEEDS_LENGTH  1
/*===========================================
               Class  Network
 ============================================*/
class Network : public UdpPort {
public:
    Network();
    ~Network();

    void send(uint8_t* xmitData, uint8_t dataLen, SendReqType type);
    int  readPacket(uint8_t type = 0);
    void setGwAddress();
    void resetGwAddress(void);
    void setRxHandler(void (*callbackPtr)(NWResponse* data, int* returnCode));
    void setSleep();
    int  initialize(UdpConfig  config);
private:
    int  readApiFrame();

	NWResponse _nlResp;
    uint32_t _gwIpAddress;
	uint16_t _gwPortNo;
    int     _returnCode;
    bool _sleepflg;
	void (*_rxCallbackPtr)(NWResponse* data, int* returnCode);
    uint8_t _rxFrameDataBuf[MQTTSN_MAX_FRAME_SIZE];

};


}    /* end of namespace */

#endif
#endif   /*  UDPSTACK_H__  */
