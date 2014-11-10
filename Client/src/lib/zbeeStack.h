/*
 * zbeeStack.h
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

#ifndef ZBEESTACK_H_
#define ZBEESTACK_H_

#ifdef ARDUINO
	#include <MQTTSN_Application.h>
#else
	#include "MQTTSN_Application.h"
	#include "mqUtil.h"
	#include "Network.h"
#endif

#ifdef NETWORK_XBEE

#if defined(ARDUINO)
	#include <MQTTSN_Application.h>
    #include <Network.h>
    #if ARDUINO >= 100
        #include "Arduino.h"
        #include <inttypes.h>
		#include <mqUtil.h>
    #else
        #if ARDUINO < 100
            #include "WProgram.h"
            #include <inttypes.h>
			#include <mqUtil.h>
        #endif
    #endif

	#define XB_CTS_PIN   3   // XBee CTS
	#define XB_SLEEP_PIN 4   // XBee Pinhybernate
#endif


#ifdef MBED
    #include "mbed.h"
    #define ZB_MBED_SERIAL_TXPIN  p13
    #define ZB_MBED_SERIAL_RXPIN  p14
#endif

#ifdef LINUX
    #include <sys/time.h>
    #include <iostream>
#endif


namespace tomyClient {

#define START_BYTE 0x7e
#define ESCAPE     0x7d
#define XON        0x11
#define XOFF       0x13


#define ZB_API_REQUEST               0x10
#define ZB_API_RESPONSE              0x90
#define ZB_API_MODEMSTATUS           0x8A

#define ZB_PACKET_ACKNOWLEGED        0x01
#define ZB_BROADCAST_PACKET          0x02
#define ZB_BROADCAST_RADIUS_MAX_HOPS 0
#define ZB_RSP_DATA_OFFSET           11
#define ZB_REQ_DATA_OFFSET           13

#define API_ID_POS                    3
#define PACKET_OVERHEAD_LENGTH        6

#define ZB_MAX_NODEID  20


/*====  STATUS ====== */
#define SUCCESS           0x0

#define NO_ERROR                          0
#define CHECKSUM_FAILURE                  1
#define PACKET_EXCEEDS_BYTE_ARRAY_LENGTH  2
#define UNEXPECTED_START_BYTE             3


/*
 *   Packet Read Constants
 */
#if defined(ARDUINO) || defined(MBED)
  #define PACKET_TIMEOUT_CHECK   300
#else
  #define PACKET_TIMEOUT_CHECK   200
#endif

#define RING_BUFFER_SIZE  256


/*============================================
              NWAddress64
 =============================================*/
class NWAddress64 {
public:
  NWAddress64(uint32_t msb, uint32_t lsb);
  NWAddress64(void);
  uint32_t getMsb();
  uint32_t getLsb();
  void    setMsb(uint32_t msb);
  void    setLsb(uint32_t lsb);
private:
  uint32_t _msb;
  uint32_t _lsb;
};

 /*============================================
                XBResponse
 =============================================*/

class XBResponse {
public:
    XBResponse();
    uint8_t  getApiId();
    uint8_t  getMsbLength();
    uint8_t  getLsbLength();
    uint8_t  getChecksum();
    uint8_t  getFrameLength();
    uint8_t* getFrameDataPtr();

    void setApiId(uint8_t api);
    void setMsbLength(uint8_t msbLength);
    void setLsbLength(uint8_t lsbLength);
    void setChecksum(uint8_t checksum);
    void setFrameDataPtr(uint8_t* frameDataPtr);
    void setFrameLength(uint8_t frameLength);

    bool isAvailable();
    void setAvailable(bool complete);
    bool isError();
    void setErrorCode(uint8_t errorCode);
    void reset();
    uint8_t getErrorCode();

protected:
    uint8_t* _frameDataPtr;

private:
    uint8_t _msbLength;
    uint8_t _frameLength;
    uint8_t _apiId;
    uint8_t _checksum;
    uint8_t _errorCode;
    bool   _complete;
};

/*============================================
               XBModemStatus
=============================================*/
class XBModemStatus : public XBResponse{
public:
	XBModemStatus();
	uint8_t getStatus();
};
		
/*============================================
               NLResponse
=============================================*/

class NWResponse : public XBResponse{
public:
	NWResponse();
	uint8_t  getPayload(uint8_t index);
	uint8_t  getPayloadLength();
	uint8_t  getType();
	uint8_t  getOption();
	uint8_t* getPayload();
	uint8_t* getBody();
	uint16_t getBodyLength();
	uint16_t getRemoteAddress16();
	NWAddress64& getRemoteAddress64();

	void setRemoteAddress64(NWAddress64& addr64);
	void setRemoteAddress64();
	void setRemoteAddress16(uint16_t addr16);
	void setRemoteAddress16();
	void setOption(uint8_t options);
	void setOption();
	bool isBrodcast();

private:
	NWAddress64 _remoteAddress64;
	uint16_t    _remoteAddress16;
	uint8_t     _options;
};

/*============================================*
                NLRequest
 =============================================*/

class NWRequest {
public:
    NWRequest();
    ~NWRequest(){};
//    uint8_t getFrameData(uint8_t pos);
    uint8_t  getFrameDataLength();
    uint8_t  getBroadcastRadius();
    uint8_t  getOption();
    uint8_t  getPayloadLength();
    uint8_t* getPayload();

    void setBroadcastRadius(uint8_t broadcastRadius);
    void setOption(uint8_t option);
    void setPayload(uint8_t *payload);
    void setPayloadLength(uint8_t payLoadLength);

private:
    uint8_t _broadcastRadius;
    uint8_t _option;
    uint8_t _payloadLength;
    uint8_t* _payloadPtr;
};


/*===========================================
                Serial Port
 ============================================*/

#ifdef ARDUINO
#include <Stream.h>
class SerialPort{
public:
    SerialPort( );
    int  open(NETWORK_CONFIG config);
    bool send(unsigned char b);
    bool recv(unsigned char* b);
    void flush();
    bool checkRecvBuf();
private:
    Stream* _serialDev;
};
#endif /* ARDUINO */

#ifdef MBED
/*-------------------------
    For MBED
 --------------------------*/
class SerialPort{
public:
    SerialPort( );
    int  open(XBeeConfig config);
    bool send(unsigned char b);
    bool recv(unsigned char* b);
    void flush();
    bool checkRecvBuf();
    void setBuff(void);
    void putc(uint8_t c);
private:
	Serial* _serialDev;
	uint8_t _data[RING_BUFFER_SIZE];
	int _head;
	int _tail;
};
#endif /* MBED */

#ifdef LINUX
/*-------------------------
    For Linux
 --------------------------*/
#include <termios.h>
class SerialPort{
public:
    SerialPort();
    ~SerialPort();
    int  open(XBeeConfig  config);
    bool send(unsigned char b);
    bool recv(unsigned char* b);
    bool checkRecvBuf();
    void flush();
private:
    int open(const char* devName, unsigned int boaurate,  bool parity, unsigned int stopbit);
    int _fd;  // file descriptor
    struct termios _tio;
};
#endif /* LINUX */


/*===========================================
               Class  Network
 ============================================*/
class Network {
public:
    Network();
    ~Network();

    void send(uint8_t* xmitData, uint8_t dataLen, SendReqType type);
    int  readPacket(uint8_t type = 0);
    void setGwAddress();
    void resetGwAddress(void);
    void setRxHandler(void (*callbackPtr)(NWResponse* data, int* returnCode));
    int  initialize(XBeeConfig  config);

private:
    void setSleep();
    NWAddress64& getRxRemoteAddress64();
	uint16_t       getRxRemoteAddress16();
	const char*   getNodeId();
	void          getResponse(NWResponse& response);
	NWResponse*    getResponse();

	void setSerialPort(SerialPort *serialPort);

    void sendZBRequest(NWRequest& request, SendReqType type);
    int  packetHandle();
    void execCallback();
    void readApiFrame(void);
    bool readApiFrame(uint16_t timeoutMillsec);
    void flush();
    void resetResponse();
    bool read(uint8_t* buff);
    bool write(uint8_t val);
    void sendByte(uint8_t, bool escape);
    uint8_t getAddrByte(uint8_t pos, SendReqType type);

    NWRequest   _txRequest;
    NWResponse  _rxResp;
    XBResponse  _response;    //  Received data
    uint8_t     _responsePayload[MQTTSN_MAX_FRAME_SIZE];
    XTimer      _tm;
    SerialPort* _serialPort;
    NWAddress64 _gwAddress64;
    uint16_t    _gwAddress16;
    uint8_t     _pos;
    uint8_t     _byteData;
    uint8_t     _checksumTotal;
    uint16_t    _addr16;
    uint32_t    _addr32;
    int  _returnCode;
    bool _escape;
    bool _sleepflg;

    void (*_rxCallbackPtr)(NWResponse* data, int* returnCode);
};

}
#endif
#endif  /* ZBEESTACK_H_ */
