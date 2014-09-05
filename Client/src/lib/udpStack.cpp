/*
 * udpStack.cpp
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
		#include "Network.h"
#else
        #include <MQTTSN_Application.h>
		#include <Network.h>
#endif

#ifdef NETWORK_UDP

#ifdef ARDUINO
  #include <udpStack.h>
  #include <mqUtil.h>
  #include <SPI.h>
  #include <Ethernet.h>
  #include <EthernetUdp.h>

  #if defined( NW_DEBUG) || defined(MQTTSN_DEBUG)
        #include <SoftwareSerial.h>
        extern SoftwareSerial debug;
  #endif

#endif  /* ARDUINO */

#ifdef MBED
        #include "mbed.h"
        #include "udpStack.h"
		#include "mqUtil.h"
#endif /* MBED */

#ifdef LINUX
        #include "udpStack.h"
		#include "mqUtil.h"
        #include <stdio.h>
        #include <sys/time.h>
        #include <sys/types.h>
		#include <sys/socket.h>
        #include <sys/stat.h>
        #include <unistd.h>
        #include <stdlib.h>
        #include <string.h>
        #include <fcntl.h>
        #include <errno.h>
        #include <termios.h>

#endif /* LINUX */

using namespace std;
using namespace tomyClient;

extern uint16_t getUint16(uint8_t* pos);
extern void setUint16(uint8_t* pos, uint16_t val);
extern uint32_t getUint32(uint8_t* pos);
extern void setUint32(uint8_t* pos, uint32_t val);

/*=========================================
       Class Network
 =========================================*/
Network::Network(){
	_sleepflg = false;
	resetGwAddress();
}

Network::~Network(){

}

void Network::send(uint8_t* xmitData, uint8_t dataLen, SendReqType type){
	if(type == BcastReq){
		multicast(xmitData, (uint16_t)dataLen);
	}else if(type == UcastReq ){
		unicast(xmitData, (uint16_t)dataLen, _gwIpAddress, _gwPortNo);
	}
}

int  Network::readPacket(uint8_t type){
	_returnCode = 0;

	if(checkRecvBuf()){
		if(readApiFrame()){
			if(_nlResp.isAvailable()){
				if(_rxCallbackPtr){
					_rxCallbackPtr(&_nlResp, &_returnCode);
				}
			}
		}
	}
	return _returnCode;
}


int  Network::readApiFrame(){
	uint16_t portNo = 0;
	uint32_t ipAddress = 0;
	uint16_t len;

	if (_nlResp.isAvailable() || _nlResp.isError()){
	   _nlResp.setAvailable(false);
	   _nlResp.setErrorCode(NO_ERROR);
	}

	uint16_t recvLen = recv(_rxFrameDataBuf, MQTTSN_MAX_FRAME_SIZE, false, &ipAddress, &portNo);

	if( recvLen > 0){
		if(*_rxFrameDataBuf == 0x01){
			len = getUint16(_rxFrameDataBuf + 1);
		}else{
			len = *_rxFrameDataBuf;
		}

		if( len != recvLen){
			_nlResp.setErrorCode(PACKET_EXCEEDS_LENGTH);
						return false;
		}else if(_gwIpAddress && isUnicast() &&
		 		 (_nlResp.getAddress64().getLsb() != _gwIpAddress) &&
				 (_nlResp.getAddress16() != _gwPortNo)){
			D_NWSTACKW("  Sender is not Gateway!\r\n" );
			return false;
		}else{
			_nlResp.setLength(len);
			_nlResp.setAvailable(true);
			_nlResp.setFrame(_rxFrameDataBuf);
			_nlResp.setAddress16(portNo);
			_nlResp.setAddress64(0,ipAddress);
			return true;
		}
	}
	return false;
}

void Network::setGwAddress(){
	_gwPortNo = _nlResp.getAddress16();
	_gwIpAddress = _nlResp.getAddress64().getLsb();
}

void Network::resetGwAddress(void){
	_gwIpAddress = 0;
	_gwPortNo = 0;
}

void Network::setRxHandler(void (*callbackPtr)(NWResponse* data, int* returnCode)){
	_rxCallbackPtr = callbackPtr;
}

int Network::initialize(UdpConfig  config){
	return open(config);
}

void Network::setSleep(){
	_sleepflg = true;
}

/*=========================================
       Class udpStack
 =========================================*/
#ifdef ARDUINO
/**
 *  For Arduino
 */
UdpPort::UdpPort(){

}

UdpPort::~UdpPort(){
    close();
}

void UdpPort::close(){

}


bool UdpPort::open(UdpConfig config){
	_gIpAddr = IPAddress(config.ipAddress);
	_cIpAddr = IPAddress(config.ipLocal);
	_gPortNo = config.gPortNo;
	_uPortNo = config.uPortNo;

	memcpy(_macAddr, config.macAddr, 6);

	Ethernet.begin(_macAddr, _cIpAddr);

	if(_udpMulticast.beginMulti(_gIpAddr, _gPortNo) == 0){
		return false;
	}
	if(_udpUnicast.begin(_uPortNo) == 0){
		return false;
	}

	return true;
}

int UdpPort::unicast(const uint8_t* buf, uint32_t length, uint32_t ipAddress, uint16_t port  ){

	IPAddress ip = IPAddress(ipAddress);
	_udpUnicast.beginPacket(ip, port);
	_udpUnicast.write(buf, length);
	return _udpUnicast.endPacket();
}


int UdpPort::multicast( const uint8_t* buf, uint32_t length ){
	_udpMulticast.beginPacket(_gIpAddr, _gPortNo);
	_udpMulticast.write(buf, length);
	return _udpMulticast.endPacket();
}

bool UdpPort::checkRecvBuf(){
	int ps = _udpUnicast.parsePacket();
	if(ps > 0){
		_castStat = STAT_UNICAST;
		return true;
	}else if ( (ps = _udpMulticast.parsePacket()) > 0){
		_castStat = STAT_MULTICAST;
		return true;
	}
	_castStat = 0;
	return 0;
}

int UdpPort::recv(uint8_t* buf, uint16_t len, bool flg, uint32_t* ipAddressPtr, uint16_t* portPtr){
	return recvfrom ( buf, len, 0, ipAddressPtr, portPtr );
}

int UdpPort::recvfrom ( uint8_t* buf, uint16_t len, int flags, uint32_t* ipAddressPtr, uint16_t* portPtr ){
	IPAddress remoteIp;
	uint8_t packLen;
	if(_castStat == STAT_UNICAST){
		packLen = _udpUnicast.read(buf, len);
		*portPtr = _udpUnicast.remotePort();
		remoteIp = _udpUnicast.remoteIP();
	}else if(_castStat == STAT_MULTICAST){
		packLen = _udpMulticast.read(buf, len);
		*portPtr = _udpMulticast.remotePort();
		remoteIp = _udpMulticast.remoteIP();
	}else{
		return 0;
	}
	memcpy(ipAddressPtr,remoteIp.raw_address(), 4);
	return packLen;
}

bool UdpPort::isUnicast(){
	return ( _castStat == STAT_UNICAST);
}

#endif /* ARDUINO */


#ifdef MBED
/**
 *  For MBED
 */


#endif /* MBED */

#ifdef LINUX

UdpPort::UdpPort(){
    _disconReq = false;
    _sockfdUcast = -1;
    _sockfdMcast = -1;
    _castStat = 0;
}

UdpPort::~UdpPort(){
    close();
}


void UdpPort::close(){
	if(_sockfdMcast > 0){
		::close( _sockfdMcast);
		_sockfdMcast = -1;
	if(_sockfdUcast > 0){
			::close( _sockfdUcast);
			_sockfdUcast = -1;
		}
	}
}

bool UdpPort::open(UdpConfig config){
	const int reuse = 1;
	char loopch = 0;

	_gPortNo = htons(config.gPortNo);
	_gIpAddr = getUint32(config.ipAddress);
	_uPortNo = htons(config.uPortNo);

	if( _gPortNo == 0 || _gIpAddr == 0 || _uPortNo == 0){
		return false;
	}

	_sockfdUcast = socket(AF_INET, SOCK_DGRAM, 0);
	if (_sockfdUcast < 0){
		return false;
	}

	setsockopt(_sockfdUcast, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

	struct sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = _uPortNo;
	addr.sin_addr.s_addr = INADDR_ANY;

	if( ::bind ( _sockfdUcast, (struct sockaddr*)&addr,  sizeof(addr)) <0){
		return false;
	}

	_sockfdMcast = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (_sockfdMcast < 0){
		return false;
	}

	struct sockaddr_in addrm;
	addrm.sin_family = AF_INET;
	addrm.sin_port = _gPortNo;
	addrm.sin_addr.s_addr = INADDR_ANY;

	setsockopt(_sockfdMcast, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

	if( ::bind ( _sockfdMcast, (struct sockaddr*)&addrm,  sizeof(addrm)) <0){
		return false;
	}

	if(setsockopt(_sockfdUcast, IPPROTO_IP, IP_MULTICAST_LOOP,(char*)&loopch, sizeof(loopch)) <0 ){
		D_NWSTACKW("error IP_MULTICAST_LOOP in UdpPort::open\n");

		close();
		return false;
	}

	if(setsockopt(_sockfdMcast, IPPROTO_IP, IP_MULTICAST_LOOP,(char*)&loopch, sizeof(loopch)) <0 ){
		D_NWSTACKW("error IP_MULTICAST_LOOP in UdpPPort::open\n");
		close();
		return false;
	}

	ip_mreq mreq;
	mreq.imr_interface.s_addr = INADDR_ANY;
	mreq.imr_multiaddr.s_addr = getUint32(config.ipAddress);

	if( setsockopt(_sockfdMcast, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq) )< 0){
		D_NWSTACKF("error IP_ADD_MEMBERSHIP in UdpPort::open\n");
		close();
		return false;
	}
/*
	if( setsockopt(_sockfdUcast, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq) )< 0){
		D_NWSTACKF("error IP_ADD_MEMBERSHIP in UdpPort::open\n");
		close();
		return false;
	}
*/
	return true;
}

bool UdpPort::isUnicast(){
	return ( _castStat == STAT_UNICAST);
}


int UdpPort::unicast(const uint8_t* buf, uint32_t length, uint32_t ipAddress, uint16_t port  ){
	struct sockaddr_in dest;
	dest.sin_family = AF_INET;
	dest.sin_port = port;
	dest.sin_addr.s_addr = ipAddress;

	int status = ::sendto( _sockfdUcast, buf, length, 0, (const sockaddr*)&dest, sizeof(dest) );
	if( status < 0){
		D_NWSTACKF("errno == %d in UdpPort::unicast\n", errno);
	}else{
		D_NWSTACKF("sendto %s:%u  [",inet_ntoa(dest.sin_addr),htons(port));
		for(uint16_t i = 0; i < length ; i++){
			D_NWSTACKF(" %02x", *(buf + i));
		}
		D_NWSTACKF(" ]\n");
	}
	return status;
}


int UdpPort::multicast( const uint8_t* buf, uint32_t length ){
	struct sockaddr_in dest;
	dest.sin_family = AF_INET;
	dest.sin_port = _gPortNo;
	dest.sin_addr.s_addr = _gIpAddr;

	int status = ::sendto( _sockfdMcast, buf, length, 0, (const sockaddr*)&dest, sizeof(dest) );
	if( status < 0){
		D_NWSTACKF("errno == %d in UdpPort::multicast\n", errno);
	}else{
		D_NWSTACKF("sendto %s:%u  [",inet_ntoa(dest.sin_addr),htons(_gPortNo));
		for(uint16_t i = 0; i < length ; i++){
			D_NWSTACKF(" %02x", *(buf + i));
		}
		D_NWSTACKF(" ]\n");
	}
	return errno;
}

bool UdpPort::checkRecvBuf(){
	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 500000;    // 500 msec

	uint8_t buf[2];
	fd_set recvfds;
	int maxSock = 0;

	FD_ZERO(&recvfds);
	FD_SET(_sockfdUcast, &recvfds);
	FD_SET(_sockfdMcast, &recvfds);

	if(_sockfdMcast > _sockfdUcast){
		maxSock = _sockfdMcast;
	}else{
		maxSock = _sockfdUcast;
	}

	select(maxSock + 1, &recvfds, 0, 0, &timeout);

	if(FD_ISSET(_sockfdUcast, &recvfds)){
		if( ::recv(_sockfdUcast, buf, 1,  MSG_DONTWAIT | MSG_PEEK) > 0){
			_castStat = STAT_UNICAST;
			return true;
		}
	}else if(FD_ISSET(_sockfdMcast, &recvfds)){
		if( ::recv(_sockfdMcast, buf, 1,  MSG_DONTWAIT | MSG_PEEK) > 0){
			_castStat = STAT_MULTICAST;
			return true;
		}
	}
	_castStat = 0;
	return false;
}

int UdpPort::recv(uint8_t* buf, uint16_t len, bool flg, uint32_t* ipAddressPtr, uint16_t* portPtr){
	int flags = flg ? MSG_DONTWAIT : 0;
	return recvfrom (buf, len, flags, ipAddressPtr, portPtr );
}

int UdpPort::recvfrom ( uint8_t* buf, uint16_t len, int flags, uint32_t* ipAddressPtr, uint16_t* portPtr ){
	struct sockaddr_in sender;
	int status;
	socklen_t addrlen = sizeof(sender);
	memset(&sender, 0, addrlen);

	if(_castStat == STAT_UNICAST){
		status = ::recvfrom( _sockfdUcast, buf, len, flags, (struct sockaddr*)&sender, &addrlen );
	}else if(_castStat == STAT_MULTICAST){
		status = ::recvfrom( _sockfdMcast, buf, len, flags, (struct sockaddr*)&sender, &addrlen );
	}else{
		return 0;
	}

	if (status < 0 && errno != EAGAIN)	{
		D_NWSTACKF("errno == %d in UdpPort::recvfrom \n", errno);
	}else if(status > 0){
		*ipAddressPtr = sender.sin_addr.s_addr;
		*portPtr = sender.sin_port;
		D_NWSTACKF("recved from %s:%u [",inet_ntoa(sender.sin_addr), htons(*portPtr));
		for(uint16_t i = 0; i < status ; i++){
			D_NWSTACKF(" %02x", *(buf + i));
		}
		D_NWSTACKF(" ]\n");
	}else{
		return 0;
	}
	return status;
}


#endif

/*=========================================
             Class NLLongAddress
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

/*=========================================
             Class ZBResponse
 =========================================*/
NWResponse::NWResponse(){
    _addr16 = 0;
}

uint8_t  NWResponse::getFrameLength(){
	return _len;
}

void NWResponse::setLength(uint16_t len){
	_len = len;
}

void NWResponse::setFrame(uint8_t* framePtr){
	_frameDataPtr = framePtr;
}

NWAddress64&  NWResponse::getAddress64(){
    return _addr64;
}

uint16_t NWResponse::getAddress16(){
  return _addr16;
}

void  NWResponse::setAddress64(uint32_t msb, uint32_t lsb){
    _addr64.setMsb(msb);
    _addr64.setLsb(lsb);
}

void  NWResponse::setAddress16(uint16_t addr16){
	_addr16 = addr16;
}

void NWResponse::setErrorCode(uint8_t errCode){
	_errorCode = errCode;
}

void NWResponse::setAvailable(bool complete){
	_complete = complete;
}

uint8_t NWResponse::getType(){
	if(_len > 255){
		return _frameDataPtr[3];
	}else{
		return _frameDataPtr[1];
	}
}

uint8_t* NWResponse::getBody(){
	if(_len > 255){
		return _frameDataPtr + 4;
	}else{
		return _frameDataPtr + 2;
	}
}

uint16_t NWResponse::getBodyLength(){
	if(_len > 255){
		return getPayloadLength() - 4;
	}else{
		return getPayloadLength() - 2;
	}
}

uint8_t NWResponse::getPayload(uint8_t index){
		return _frameDataPtr[index];
}

uint8_t* NWResponse::getPayload(){
		return _frameDataPtr;
}

uint8_t NWResponse::getPayloadLength(){
	return _len;
}

void NWResponse::resetResponse(){
	_addr64.setLsb(0);
	_addr64.setMsb(0);
	_addr16 = 0;
	_len = 0;
	_errorCode = 0;
	_complete = false;;
}

bool NWResponse::isAvailable(){
	return _complete;
}

uint8_t NWResponse::isError(){
	return _errorCode;
}

#endif  /* NETWORK_UDP */
