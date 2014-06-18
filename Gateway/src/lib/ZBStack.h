/*
 * ZBStack.h
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
 *    Modified:
 *      Author: Tomoaki YAMAGUCHI
 *     Version: 0.0.0
 */

#ifndef ZBSTACK_H_
#define ZBSTACK_H_

#include "Defines.h"
#ifdef NETWORK_XBEE

#include <sys/time.h>
#include <iostream>
#include "ProcessFramework.h"


#define START_BYTE 0x7e
#define ESCAPE     0x7d
#define XON        0x11
#define XOFF       0x13

#define MAX_FRAME_DATA_SIZE  128

#define XB_BROADCAST_ADDRESS32    0x0000ffff
#define XB_BROADCAST_ADDRESS16     0xfffe

#define DEFAULT_FRAME_ID 1

#define XB_PACKET_ACKNOWLEGED  0x01
#define XB_BROADCAST_PACKET    0x02
#define XB_BROADCAST_RADIUS_MAX_HOPS 0

#define API_ID_INDEX  3
#define PACKET_OVERHEAD_LENGTH 6
#define ZB_TX_API_LENGTH  12
#define ZB_PAYLOAD_OFFSET 11

/**
 * API ID Constant
 */
#define XB_TX_REQUEST          0x10
#define XB_RX_RESPONSE         0x90

/**
 * TX STATUS
 */
#define SUCCESS           0x0

#define NO_ERROR                          0
#define CHECKSUM_FAILURE                  1
#define PACKET_EXCEEDS_BYTE_ARRAY_LENGTH  2
#define UNEXPECTED_START_BYTE             3


#define NW_TX_UNICAST 0
#define NW_TX_BROADCAST 8

#define NW_MAX_NODEID 24

/*
 *   MQTTS  Client's state
 */
#define MQTTS_DEVICE_DISCONNECTED     0
#define MQTTS_DEVICE_ACTIVE           1
#define MQTTS_DEVICE_ASLEEP           2
#define MQTTS_DEVICE_AWAKE            3
#define MQTTS_DEVICE_LOST             4

namespace tomyGateway{

/*============================================
              NWAddress64
 =============================================*/
class NWAddress64 {
public:
	NWAddress64(uint32_t msb, uint32_t lsb);
	NWAddress64();
	uint32_t getMsb();
	uint32_t getLsb();
	void setMsb(uint32_t msb);
	void setLsb(uint32_t lsb);
	bool operator==(NWAddress64&);
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
	~NWResponse();
	uint8_t getMsbLength();
	uint8_t getLsbLength();
	uint8_t getChecksum();
	uint8_t getFrameDataLength();
	uint8_t getOption();
	uint8_t getType();
	uint8_t getPayload(int index);
	uint8_t getPayloadLength();
	uint8_t getPayloadOffset();
	uint8_t getApiId();
	uint8_t getMsgType();
	uint8_t getErrorCode();
	uint8_t* getFrameData();
	uint8_t* getPayloadPtr();
	uint16_t getPacketLength();
	uint16_t getClientAddress16();
	NWAddress64* getClientAddress64();

	bool isBroadcast();
	bool isAvailable();
	bool isError();

	void setApiId(uint8_t apiId);
	void setMsbLength(uint8_t msbLength);
	void setLsbLength(uint8_t lsbLength);
	void setChecksum(uint8_t checksum);
	void setFrameData(uint8_t *frameDataPtr);
	void setFrameDataLength(uint8_t frameLength);
	void setAvailable(bool complete);
	void setErrorCode(uint8_t errorCode);

void reset();
	void absorb(NWResponse* resp);
protected:
	uint8_t *_frameDataPtr;
private:
	uint32_t getRemoteAddressMsb();
	uint32_t getRemoteAddressLsb();

	uint8_t _apiId;
	uint8_t _msbLength;
	uint8_t _lsbLength;
	uint8_t _checksum;
	uint8_t _frameLength;
	uint8_t _errorCode;
	bool _complete;
	NWAddress64 _addr64;
};

/*============================================
          Class NWRequest
 =============================================*/

class NWRequest{
public:
	NWRequest();
	NWRequest(NWAddress64 &addr64, uint8_t *payload, uint8_t payLoadLength);
	NWRequest(NWAddress64 &addr64, uint16_t addr16, uint8_t broadcastRadius,
				uint8_t option, uint8_t *payload, uint8_t payloadLength);
	~NWRequest();
	uint8_t* getPayloadPtr();
	uint8_t getBroadcastRadius();
	uint8_t getOption();
	uint8_t getPayloadLength();
	uint8_t getApiId();
	uint8_t getFrameData(uint8_t pos);
	uint8_t getFrameDataLength();
	uint16_t getAddress16();
	NWAddress64& getAddress64();

	void setApiId(uint8_t apiId);
	void setClientAddress64(NWAddress64* addr64);
	void setClientAddress16(uint16_t addr16);
	void setBroadcastRadius(uint8_t broadcastRadius);
	void setOption(uint8_t option);
	void setPayload(uint8_t *payload);
	void setPayloadLength(uint8_t payLoadLength);

private:
	uint8_t _apiId;
	uint8_t _broadcastRadius;
	uint8_t _option;
	uint8_t _payloadLength;
	uint8_t* _payloadPtr;
	uint16_t _addr16;
	NWAddress64 _addr64;
};

/*===========================================
                SerialPort
 ============================================*/
#include <termios.h>
class SerialPort{
public:
	SerialPort();
	~SerialPort();
	int open(XBeeConfig  config);
	bool send(unsigned char b);
	bool recv(unsigned char* b);
	void flush();

private:
	int open(const char* devName, unsigned int boaurate,  bool parity, unsigned int stopbit, unsigned int flg);

	int _fd;  // file descriptor
	struct termios _tio;
};


/*============================================
                 XBee
 ============================================*/

class XBee {
public:
	XBee();
	~XBee();

protected:
	int  initialize(XBeeConfig  config);
	bool receiveResponse(NWResponse* response);
	void sendRequest(NWRequest& request);
private:
	void readPacket(void);
	bool read(uint8_t* buff);
	bool write(uint8_t val);
	void sendByte(uint8_t, bool escape);
	void resetResponse();
	void flush();

	SerialPort *_serialPort;
	NWResponse _response;
	bool _escape;
	uint8_t _pos;
	uint8_t _checksumTotal;
	uint8_t _bd;
};

/*===========================================
               Class  NetworkStack
 ============================================*/
class Network : public XBee{
public:
	Network();
	~Network();
	void unicast(NWAddress64* addr64, uint16_t addr16,
			uint8_t* payload, uint16_t payloadLength);
	void broadcast(uint8_t* payloadLength, uint16_t bodyLenght);
	bool getResponse(NWResponse* response);
	int initialize(XBeeConfig  config);

private:
	NWRequest   _txRequest;
};

}

#endif /* NETWORK_XBEE */
#endif  /* ZBSTACK_H_ */
