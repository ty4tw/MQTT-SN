/*
 * ZBeeStack.cpp
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

#include "Defines.h"
#ifdef  NETWORK_XBEE

#include "Messages.h"
#include "ZBStack.h"
#include "ProcessFramework.h"
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

using namespace std;

extern uint8_t* mqcalloc(uint8_t length);
extern uint16_t getUint16(uint8_t* pos);
extern uint32_t getUint32(uint8_t* pos);
extern void setUint16(uint8_t* pos, uint16_t val);
extern void setUint32(uint8_t* pos, uint32_t val);

/*=========================================
             Class NWAddress64
 =========================================*/
NWAddress64::NWAddress64(){
  _msb = _lsb = 0;
}

NWAddress64::NWAddress64(uint32_t msb, uint32_t lsb){
  _msb = msb;
  _lsb = lsb;
}

uint32_t NWAddress64::getMsb(){
  return _msb;
}

uint32_t NWAddress64::getLsb(){
  return _lsb;
}

void NWAddress64::setMsb(uint32_t msb){
  _msb = msb;
}

void NWAddress64::setLsb(uint32_t lsb){
  _lsb = lsb;
}

bool NWAddress64::operator==(NWAddress64& addr){
	if(_msb == addr.getMsb() && _lsb == addr.getLsb()){
		return true;
	}else{
		return false;
	}
}
/*=========================================
             Class NWBResponse
 =========================================*/
NWResponse::NWResponse(){
	_frameDataPtr = 0;
	_msbLength = 0;
	_lsbLength = 0;
	_checksum = 0;
	_frameLength = 0;
	_errorCode = NO_ERROR;
	_complete = false;
	_apiId = 0;
}

NWResponse::~NWResponse(){
  if(_frameDataPtr){
	  free(_frameDataPtr);
  }
}

uint8_t NWResponse::getMsbLength(){
  return _msbLength;
}

uint8_t NWResponse::getLsbLength(){
  return _lsbLength;
}

uint8_t NWResponse::getChecksum(){
  return _checksum;
}

uint8_t NWResponse::getFrameDataLength(){
  return _frameLength;
}

uint8_t* NWResponse::getFrameData(){
  return _frameDataPtr;
}

uint16_t NWResponse::getPacketLength() {
        return ((_msbLength << 8) & 0xff) + (_lsbLength & 0xff);
}

void NWResponse::setMsbLength(uint8_t msbLength){
  _msbLength = msbLength;
}

void NWResponse::setLsbLength(uint8_t lsbLength){
  _lsbLength = lsbLength;
}

void NWResponse::setChecksum(uint8_t checksum){
  _checksum = checksum;
}

void NWResponse::setFrameDataLength(uint8_t frameLength){
  _frameLength = frameLength;
}
void NWResponse::setFrameData(uint8_t *frameDataPtr){
  _frameDataPtr = frameDataPtr;
}

bool NWResponse::isAvailable(){
  return _complete;
}

void NWResponse::setAvailable(bool complete){
  _complete = complete;
}

bool NWResponse::isError(){
  return _errorCode > 0;
}

uint8_t NWResponse::getErrorCode(){
  return _errorCode;
}

void NWResponse::setErrorCode(uint8_t errorCode){
  _errorCode = errorCode;
}

void NWResponse::setApiId(uint8_t apiId){
  _apiId = apiId;
}

void NWResponse::reset(){
	if(_frameDataPtr){
		  free(_frameDataPtr);
	}
	_msbLength = 0;
	_lsbLength = 0;
	_checksum = 0;
	_frameLength = 0;
	_errorCode = NO_ERROR;
_complete = false;
}

uint8_t NWResponse::getPayload(int index){
  return getFrameData()[ZB_PAYLOAD_OFFSET + index];
}

uint8_t* NWResponse::getPayloadPtr(){
  return getFrameData() + ZB_PAYLOAD_OFFSET;
}

uint8_t NWResponse::getPayloadLength(){
  return getFrameDataLength() - ZB_PAYLOAD_OFFSET;
}

uint16_t NWResponse::getClientAddress16(){
	return getUint16(getFrameData() + 8);
}


uint32_t NWResponse::getRemoteAddressMsb(){
	return getUint32(getFrameData());
}

uint32_t NWResponse::getRemoteAddressLsb(){
	return getUint32(getFrameData() + 4);
}

NWAddress64* NWResponse::getClientAddress64(){
	return &_addr64;
}

uint8_t NWResponse::getOption(){
  return getFrameData()[10];
}

uint8_t NWResponse::getType(){
	if(_frameLength > 255){
		return getPayload(3);
	}else{
		return getPayload(1);
	}
}
uint8_t NWResponse::getApiId(){
  return _apiId;
}

uint8_t NWResponse::getMsgType(){
  return getPayloadPtr()[1];
}

bool NWResponse::isBroadcast(){
  return ( getOption() && 0x02);
}

void NWResponse::absorb(NWResponse* resp){
	if(_frameDataPtr){
		free(_frameDataPtr);
	}
	_apiId = resp->getApiId();
	_msbLength = resp->getMsbLength();
	_lsbLength = resp->getLsbLength();
	_checksum = resp->getChecksum();
	_frameLength = resp->getFrameDataLength();
	_errorCode = resp->getErrorCode();
	_complete = resp->isAvailable();
	_addr64.setMsb(resp->getRemoteAddressMsb());
	_addr64.setLsb(resp->getRemoteAddressLsb());
	_frameDataPtr = mqcalloc(resp->getFrameDataLength());
	memcpy(_frameDataPtr, resp->getFrameData(), resp->getFrameDataLength());
}

/*=========================================
             Class NWRequest
 =========================================*/

NWRequest::NWRequest(){
	_apiId = 0x10;
	_addr16 = 0;
	_broadcastRadius = 0;
	_option = 0;
	_payloadPtr = 0;
	_payloadLength = 0;
}


NWRequest::~NWRequest(){

}

NWAddress64& NWRequest::getAddress64(){
    return _addr64;
}

uint16_t NWRequest::getAddress16(){
    return _addr16;
}

uint8_t NWRequest::getApiId(){
    return _apiId;
}

uint8_t NWRequest::getBroadcastRadius(){
    return _broadcastRadius;
}

uint8_t  NWRequest::getOption(){
    return _option;
}

uint8_t*  NWRequest::getPayloadPtr(){
    return _payloadPtr;
}

uint8_t  NWRequest::getPayloadLength(){
  return _payloadLength;
}

void NWRequest::setClientAddress64(NWAddress64* addr64){
	_addr64.setMsb(addr64->getMsb());
	_addr64.setLsb(addr64->getLsb());
}

void NWRequest::setClientAddress16(uint16_t addr16){
    _addr16 = addr16;
}

void NWRequest::setBroadcastRadius(uint8_t broadcastRadius){
    _broadcastRadius = broadcastRadius;
}

void NWRequest::setOption(uint8_t option){
    _option = option;
}

void NWRequest::setApiId(uint8_t apiId){
    _apiId = apiId;
}

void NWRequest::setPayload(uint8_t* payload){
    _payloadPtr = payload;
}

void NWRequest::setPayloadLength(uint8_t payloadLength){
    _payloadLength = payloadLength;
}


uint8_t NWRequest::getFrameData(uint8_t pos){
	uint8_t buf[4];

	if (pos == 0){
		return 0;    // Frame ID
	}else if (pos == 1){
		setUint32(buf, _addr64.getMsb());
		return buf[0];
	}else if (pos == 2){
		setUint32(buf, _addr64.getMsb());
		return buf[1];
	}else if (pos == 3){
		setUint32(buf, _addr64.getMsb());
		return buf[2];
	}else if (pos == 4){
		setUint32(buf, _addr64.getMsb());
		return buf[3];
	}else if (pos == 5){
		setUint32(buf, _addr64.getLsb());
		return buf[0];
	}else if (pos == 6){
		setUint32(buf, _addr64.getLsb());
		return buf[1];
	}else if (pos == 7){
		setUint32(buf, _addr64.getLsb());
		return buf[2];
	}else if (pos == 8){
		setUint32(buf, _addr64.getLsb());
		return buf[3];
	}else if (pos == 9){
		setUint16(buf,_addr16);
		return buf[0];
	}else if (pos == 10){
		setUint16(buf,_addr16);
		return buf[1];
	}else if (pos == 11){
		return _broadcastRadius;
	}else if (pos == 12){
		return _option;
	}
	return getPayloadPtr()[pos - ZB_TX_API_LENGTH -1 ];
}

uint8_t NWRequest::getFrameDataLength(){
    return ZB_TX_API_LENGTH + 1 + getPayloadLength();
}


/*=========================================
             Class XBee
 =========================================*/

XBee::XBee(){
	_pos = 0;
	_escape = false;
	_checksumTotal = 0;
	_response.setFrameData(mqcalloc(MAX_FRAME_DATA_SIZE));
	_serialPort = new SerialPort();
	_bd = 0;
}

XBee::~XBee(){

}

int XBee::initialize(XBeeConfig  config){
	return _serialPort->open(config);
}

void XBee::readPacket(){

	while(read(&_bd)){
	  // Check Start Byte
	  if( _pos > 0 && _bd == START_BYTE){
		  _pos = 0;
	  }
	  // Check ESC
	  if(_pos > 0 && _bd == ESCAPE){
		  if(read(&_bd )){
			  _bd = 0x20^_bd;  // decode
		  }else{
			  _escape = true;
			  continue;
		  }
	  }

	  if(_escape){
		  _bd = 0x20 ^ _bd;
		  _escape = false;
	  }

	  if(_pos >= API_ID_INDEX){
		  _checksumTotal+= _bd;
	  }
	  switch(_pos){
		case 0:
		  if(_bd == START_BYTE){
			  _pos++;
		  }
		  break;
		case 1:
		  _response.setMsbLength(_bd);
		  _pos++;
		  break;
		case 2:
		  _response.setLsbLength(_bd);
		  _pos++;
		  D_NWSTACK("\r\n===> Recv start: ");
		  break;
		case 3:
		  _response.setApiId(_bd);
		  _pos++;
		  break;
		default:
		  if(_pos > MAX_FRAME_DATA_SIZE){
			  _response.setErrorCode(PACKET_EXCEEDS_BYTE_ARRAY_LENGTH);
			  return;
		  }else if(_pos == (_response.getPacketLength() + 3)){  // 3 = 2(packet len) + 1(checksum)
			  if((_checksumTotal & 0xff) == 0xff){
				  _response.setChecksum(_bd);
				  _response.setAvailable(true);
				  _response.setErrorCode(NO_ERROR);
			  }else{
				  _response.setErrorCode(CHECKSUM_FAILURE);
			  }
			  _response.setFrameDataLength(_pos - 4);    // 4 = 2(packet len) + 1(Api) + 1(checksum)
			  _pos = 0;
			  _checksumTotal = 0;
			  return;
		  }else{
			  uint8_t* buf = _response.getFrameData();
			  buf[_pos - 4] = _bd;
			  _pos++;
			  if (_response.getApiId() == XB_RX_RESPONSE && _pos == 15){
				  D_NWSTACK( "\r\n     Payload: ");
			  }
		  }
		  break;
	  }
	}
}

bool XBee::receiveResponse(NWResponse* response){

    while(true){
    	readPacket();

        if(_response.isAvailable()){
        	D_NWSTACK("\r\n<=== CheckSum OK\r\n\n");
			response->absorb(&_response);
            return true;

        }else if(_response.isError()){
        	D_NWSTACK("\r\n<=== Packet Error Code = %d\r\n\n",_response.getErrorCode());
			_response.reset();
			response->reset();
            return false;
        }
    }
    return false;
}


void XBee::sendRequest(NWRequest &request){
	D_NWSTACK("\r\n===> Send start: ");

	sendByte(START_BYTE, false);

	uint8_t msbLen = ((request.getFrameDataLength() + 1) >> 8) & 0xff; // 1 = 1B(Api)  except Checksum
	uint8_t lsbLen = (request.getFrameDataLength() + 1) & 0xff;
	sendByte(msbLen, true);
	sendByte(lsbLen, true);

	sendByte(request.getApiId(), true);

	uint8_t checksum = 0;
	checksum+= request.getApiId();

	for( int i = 0; i < request.getFrameDataLength(); i++ ){
	  if (request.getApiId() == XB_TX_REQUEST && i == 13){
		  D_NWSTACK("\r\n     Payload:    ");
	  }
	  sendByte(request.getFrameData(i), true);
	  checksum+= request.getFrameData(i);
	}
	checksum = 0xff - checksum;
	sendByte(checksum, true);

	//flush();  // clear receive buffer

	D_NWSTACK("\r\n<=== Send completed\r\n\n" );
}

void XBee::sendByte(uint8_t b, bool escape){
	if(escape && (b == START_BYTE || b == ESCAPE || b == XON || b == XOFF)){
	  write(ESCAPE);
	  write(b ^ 0x20);
	}else{
	  write(b);
	}
}

void XBee::resetResponse(){
	_pos = 0;
	_escape = 0;
	_response.reset();
}

void XBee::flush(){
	_serialPort->flush();
}

bool XBee::write(uint8_t val){
	return (_serialPort->send(val) ? true : false );
}

bool XBee::read(uint8_t *buff){
	return  _serialPort->recv(buff);
}

/*===========================================
              Class  Network
 ============================================*/
Network::Network(){

}

Network::~Network(){

}

void Network::unicast(NWAddress64* addr64, uint16_t addr16,
		uint8_t* payload, uint16_t payloadLength ){

	_txRequest.setClientAddress64(addr64);
	_txRequest.setClientAddress16(addr16);
	_txRequest.setOption(0);
	_txRequest.setPayload(payload);
	_txRequest.setPayloadLength(payloadLength);
	sendRequest(_txRequest);
}

void Network::broadcast(uint8_t* payload, uint16_t payloadLength){
	NWAddress64 addr;
	addr.setMsb(0);
	addr.setLsb(XB_BROADCAST_ADDRESS32);
	unicast(&addr, XB_BROADCAST_ADDRESS16, payload, payloadLength);
}

bool Network::getResponse(NWResponse* response){
	return receiveResponse(response);
}

int Network::initialize(XBeeConfig config){
	return XBee::initialize(config);
}

/*=========================================
       Class SerialPort
 =========================================*/
SerialPort::SerialPort(){
    _tio.c_iflag = IGNBRK | IGNPAR;
    _tio.c_cflag = CS8 | CLOCAL | CREAD | CRTSCTS;
    _tio.c_cc[VINTR] = 0;
    _tio.c_cc[VTIME] = 0;
    _tio.c_cc[VMIN] = 1;
    _fd = 0;
}

SerialPort::~SerialPort(){
	  if (_fd){
		  close(_fd);
	  }
}

int SerialPort::open(XBeeConfig config){
  return open(config.device, config.baudrate, false, 1, config.flag);
}

int SerialPort::open(const char* devName, unsigned int baudrate,  bool parity, unsigned int stopbit, unsigned int flg){
	_fd = ::open(devName, flg | O_NOCTTY);
	if(_fd < 0){
	  return _fd;
	}

	if (parity){
	  _tio.c_cflag = _tio.c_cflag | PARENB;
	}
	if (stopbit == 2){
	  _tio.c_cflag = _tio.c_cflag | CSTOPB ;
	}
	switch(baudrate){
	case B9600:
	case B19200:
	case B38400:
	case B57600:
	case B115200:
	  if( cfsetspeed(&_tio, baudrate)<0){
		return errno;
	  }
	  break;
	default:
	  return -1;
	}
	return tcsetattr(_fd, TCSANOW, &_tio);
}

bool SerialPort::send(unsigned char b){
	if (write(_fd, &b,1) != 1){
	    return false;
	}else{
		D_NWSTACK( " %02x", b);
	    return true;
	}
}

bool SerialPort::recv(unsigned char* buf){
	if(read(_fd, buf, 1) == 0){
	    return false;
	}else{
		D_NWSTACK( " %02x",buf[0] );
	    return true;
	}
}

void SerialPort::flush(void){
	tcsetattr(_fd, TCSAFLUSH, &_tio);
}

#endif /* NETWORK_XBEE */
