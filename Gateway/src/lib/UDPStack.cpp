/*
 * UDPStack.cpp
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

#ifdef NETWORK_UDP

#include <stdio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include "ProcessFramework.h"
#include "UDPStack.h"

using namespace std;
using namespace tomyGateway;

extern uint16_t getUint16(uint8_t* pos);
extern uint32_t getUint32(uint8_t* pos);
extern void setUint16(uint8_t* pos, uint16_t val);
extern void setUint32(uint8_t* pos, uint32_t val);

/*=========================================
       Class Network
 =========================================*/
Network::Network(){
}

Network::~Network(){

}

void Network::unicast(NWAddress64* addr64, uint16_t addr16, uint8_t* payload, uint16_t payloadLength){
	UDPPort::unicast(payload, payloadLength, addr64->getLsb(), addr16);
}

void Network::broadcast(uint8_t* payload, uint16_t payloadLength){
	UDPPort::multicast(payload, payloadLength);
}

bool Network::getResponse(NWResponse* response){
	uint32_t ipAddress = 0;
	uint16_t portNo = 0;
	uint16_t msgLen;
	uint8_t  msgType;

	uint8_t* buf = response->getPayloadPtr();
	uint16_t recvLen = UDPPort::recv(buf, MQTTSN_MAX_FRAME_SIZE, &ipAddress, &portNo);
	if(recvLen < 0){
		return false;
	}else{
		if(buf[0] == 0x01){
			msgLen = getUint16(buf + 1);
			msgType = *(buf + 3);
		}else{
			msgLen = (uint16_t)*(buf);
			msgType = *(buf + 1);
		}
		if(msgLen != recvLen){
			return false;
		}
		response->setLength(msgLen);
		response->setMsgType(msgType);
		response->setClientAddress16(portNo);
		response->setClientAddress64(0, ipAddress);
		return true;
	}
}

int Network::initialize(UdpConfig  config){
	return UDPPort::initialize(config);
}


/*=========================================
       Class udpStack
 =========================================*/

UDPPort::UDPPort(){
    _disconReq = false;
    _sockfdUnicast = -1;
    _sockfdMulticast = -1;
	_gPortNo = 0;
	_gIpAddr = 0;

}

UDPPort::~UDPPort(){
    close();
}

void UDPPort::close(){
	if(_sockfdUnicast > 0){
		::close( _sockfdUnicast);
		_sockfdUnicast = -1;
	}
	if(_sockfdMulticast > 0){
		::close( _sockfdMulticast);
		_sockfdMulticast = -1;
	}
}

int UDPPort::initialize(){
	return initialize(_config);
}

int UDPPort::initialize(UdpConfig config){
	char loopch = 0;
	const int reuse = 1;

	if(config.uPortNo == 0 || config.gPortNo == 0){
		return -1;
	}
	_gPortNo = htons(config.gPortNo);
	_gIpAddr = inet_addr(config.ipAddress);

	_config.gPortNo = config.gPortNo;
	_config.uPortNo = config.uPortNo;
	_config.ipAddress = config.ipAddress;

	/*------ Create unicast socket --------*/
	_sockfdUnicast = socket(AF_INET, SOCK_DGRAM, 0);
	if (_sockfdUnicast < 0){
		return -1;
	}

	setsockopt(_sockfdUnicast, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

	sockaddr_in addru;
	addru.sin_family = AF_INET;
	addru.sin_port = htons(config.uPortNo);
	addru.sin_addr.s_addr = INADDR_ANY;

	if( ::bind ( _sockfdUnicast, (sockaddr*)&addru,  sizeof(addru)) <0){
		return -1;
	}

	if(setsockopt(_sockfdUnicast, IPPROTO_IP, IP_MULTICAST_LOOP,(char*)&loopch, sizeof(loopch)) <0 ){
		D_NWSTACK("error IP_MULTICAST_LOOP in UDPPort::open\n");
		close();
		return -1;
	}

	/*------ Create Multicast socket --------*/
	_sockfdMulticast = socket(AF_INET, SOCK_DGRAM, 0);
	if (_sockfdMulticast < 0){
		close();
		return -1;
	}

	setsockopt(_sockfdMulticast, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

	sockaddr_in addrm;
	addrm.sin_family = AF_INET;
	addrm.sin_port = htons(config.gPortNo);
	addrm.sin_addr.s_addr = INADDR_ANY;

	if( ::bind ( _sockfdMulticast, (sockaddr*)&addrm,  sizeof(addrm)) <0){
		return -1;
	}

	if(setsockopt(_sockfdMulticast, IPPROTO_IP, IP_MULTICAST_LOOP,(char*)&loopch, sizeof(loopch)) <0 ){
		D_NWSTACK("error IP_MULTICAST_LOOP in UDPPort::open\n");
		close();
		return -1;
	}

	struct ip_mreq mreq;
	mreq.imr_interface.s_addr = INADDR_ANY;
	mreq.imr_multiaddr.s_addr = inet_addr(config.ipAddress);

	if( setsockopt(_sockfdMulticast, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq))< 0){
		D_NWSTACK("error IP_ADD_MEMBERSHIP in UDPPort::open\n");
		close();
		return -1;
	}

	if( setsockopt(_sockfdUnicast, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq))< 0){
		D_NWSTACK("error IP_ADD_MEMBERSHIP in UDPPort::open\n");
		close();
		return -1;
	}
	return 0;
}


int UDPPort::unicast(const uint8_t* buf, uint32_t length, uint32_t ipAddress, uint16_t port  ){
	sockaddr_in dest;
	dest.sin_family = AF_INET;
	dest.sin_port = port;
	dest.sin_addr.s_addr = ipAddress;

	int status = ::sendto( _sockfdUnicast, buf, length, 0, (const sockaddr*)&dest, sizeof(dest) );
	if( status < 0){
		D_NWSTACK("errno == %d in UDPPort::sendto\n", errno);
	}
	D_NWSTACK("sendto %s:%u length = %d\n",inet_ntoa(dest.sin_addr), htons(port), status);
	return status;
}

int UDPPort::multicast( const uint8_t* buf, uint32_t length ){
	return unicast(buf, length,_gIpAddr, _gPortNo);
}

int UDPPort::recv(uint8_t* buf, uint16_t len, uint32_t* ipAddressPtr, uint16_t* portPtr){
	fd_set recvfds;
	int maxSock = 0;

	FD_ZERO(&recvfds);
	FD_SET(_sockfdUnicast, &recvfds);
	FD_SET(_sockfdMulticast, &recvfds);

	if(_sockfdMulticast > _sockfdUnicast){
		maxSock = _sockfdMulticast;
	}else{
		maxSock = _sockfdUnicast;
	}

	select(maxSock + 1, &recvfds, 0, 0, 0);

	if(FD_ISSET(_sockfdUnicast, &recvfds)){
		return recvfrom (_sockfdUnicast,buf, len, 0,ipAddressPtr, portPtr );
	}else if(FD_ISSET(_sockfdMulticast, &recvfds)){
		return recvfrom (_sockfdMulticast,buf, len, 0,ipAddressPtr, portPtr );
	}
	return 0;
}

int UDPPort::recvfrom (int sockfd, uint8_t* buf, uint16_t len, uint8_t flags, uint32_t* ipAddressPtr, uint16_t* portPtr ){
	sockaddr_in sender;
	socklen_t addrlen = sizeof(sender);
	memset(&sender, 0, addrlen);

	int status = ::recvfrom( sockfd, buf, len, flags, (sockaddr*)&sender, &addrlen );

	if ( status < 0 && errno != EAGAIN )	{
		D_NWSTACK("errno == %d in UDPPort::recvfrom\n", errno);
		return -1;
	}
	*ipAddressPtr = (uint32_t)sender.sin_addr.s_addr;
	*portPtr = (uint16_t)sender.sin_port;
	D_NWSTACK("recved from %s:%d length = %d\n",inet_ntoa(sender.sin_addr),htons(*portPtr),status);
	return status;
}


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

bool NWAddress64::operator==(NWAddress64& addr){
	if(_msb == addr.getMsb() && _lsb == addr.getLsb()){
		return true;
	}else{
		return false;
	}
}

/*=========================================
             Class ZBResponse
 =========================================*/
NWResponse::NWResponse(){
    _addr16 = 0;
    memset( _frameDataPtr, 0, MQTTSN_MAX_FRAME_SIZE);
}

uint8_t  NWResponse::getFrameLength(){
	return _len;
}

void NWResponse::setLength(uint16_t len){
	_len = len;
}

NWAddress64*  NWResponse::getClientAddress64(){
    return &_addr64;
}

uint16_t NWResponse::getClientAddress16(){
  return _addr16;
}

void  NWResponse::setClientAddress64(uint32_t msb, uint32_t lsb){
    _addr64.setMsb(msb);
    _addr64.setLsb(lsb);
}

void  NWResponse::setClientAddress16(uint16_t addr16){
	_addr16 = addr16;
}

void NWResponse::setMsgType(uint8_t type){
	_type = type;
}


uint8_t NWResponse::getMsgType(){
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
		return _frameDataPtr[index + 2];

}

uint8_t* NWResponse::getPayloadPtr(){

		return _frameDataPtr;

}

uint8_t NWResponse::getPayloadLength(){

	return _len;
}

#endif /* NETWORK_UDP */
