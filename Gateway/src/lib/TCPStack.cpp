/*
 * Socket.cpp
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
#include "TCPStack.h"
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <sys/socket.h>
#include <netdb.h>
using namespace std;

/*========================================
       Class Socket
 =======================================*/
TCPStack::TCPStack(){
    _addrinfo = 0;
    _disconReq = false;
    _sockfd = -1;
}

TCPStack::~TCPStack(){
    if(_addrinfo){
		freeaddrinfo(_addrinfo);
	}
}

bool TCPStack::isValid(){
	if(!_disconReq && _sockfd > 0){
		return true;
	}else if(_disconReq && _sockfd > 0){
		::close(_sockfd);
		_sockfd = -1;
    	_disconReq = false;
    	if(_addrinfo){
			freeaddrinfo(_addrinfo);
			_addrinfo = 0;
		}
		_sem.post();
	}
	return false;
}

void TCPStack::disconnect(){
    if ( _sockfd > 0 ){
    	_disconReq = true;
    	_sem.wait();
    }
}


bool TCPStack::bind ( const char* service ){
	if(isValid()){
		return false;
	}
	addrinfo hints;
	memset(&hints, 0, sizeof(addrinfo));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;

	if (_addrinfo){
		freeaddrinfo(_addrinfo);
	}
	int err = getaddrinfo(0, service, &hints, &_addrinfo);
    if (err) {
        printf("getaddrinfo(): %s\n", gai_strerror(err));
        return false;
    }

	_sockfd = socket(_addrinfo->ai_family, _addrinfo->ai_socktype, _addrinfo->ai_protocol);
    if (_sockfd < 0){
    	return false;
	}
	int on = 1;
	if ( setsockopt ( _sockfd, SOL_SOCKET, SO_REUSEADDR, ( const char* ) &on, sizeof ( on ) ) == -1 ){
		return false;
	}

	if( ::bind ( _sockfd, _addrinfo->ai_addr,  _addrinfo->ai_addrlen ) <0){
	    return false;
	}
	return true;
}

bool TCPStack::listen() {
	if ( !isValid() ){
	    return false;
	}
	int listen_return = ::listen ( _sockfd, SOCKET_MAXCONNECTIONS );
	if ( listen_return == -1 ){
	    return false;
	}
	return true;
}


bool TCPStack::accept ( TCPStack& new_socket ){
	sockaddr_storage sa;
	socklen_t len = sizeof ( sa );
	new_socket._sockfd = ::accept ( _sockfd, (struct sockaddr*) &sa,  &len );
	if ( new_socket._sockfd <= 0 ){
		return false;
	}else{
		return true;
	}
}

int TCPStack::send (const uint8_t* buf, uint16_t length  ){
	int status = ::send ( _sockfd, buf, length, MSG_NOSIGNAL );
	if( status == -1){
		printf("errno == %d in Socket::send\n", errno);
	}
	return status;
}

int TCPStack::recv ( uint8_t* buf, uint16_t len ){
	int status = ::recv ( _sockfd, buf, len, 0 );

	if ( status == -1 )	{
		printf("errno == %d in Socket::recv\n", errno);
	    return -1;
	}else if ( status == 0 ){
	    return 0;
	}else{
	    return status;
	}
}


bool TCPStack::connect ( const char* host, const char* service ){
	if(isValid()){
		return false;
	}
	addrinfo hints;
	memset(&hints, 0, sizeof(addrinfo));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;

	if (_addrinfo){
		freeaddrinfo(_addrinfo);
	}
	int err = getaddrinfo(host, service, &hints, &_addrinfo);
    if (err) {
        printf("getaddrinfo(): %s\n", gai_strerror(err));
        return false;
    }

	_sockfd = socket(_addrinfo->ai_family, _addrinfo->ai_socktype, _addrinfo->ai_protocol);
    if (_sockfd < 0){
    	return false;
	}
	int on = 1;
	if ( setsockopt ( _sockfd, SOL_SOCKET, SO_REUSEADDR, ( const char* ) &on, sizeof ( on ) ) == -1 ){
		return false;
	}

	if( ::connect ( _sockfd, _addrinfo->ai_addr,  _addrinfo->ai_addrlen ) <0){
		::close(_sockfd);
		_sockfd = -1;
	    return false;
	}
	return true;
}

void TCPStack::setNonBlocking ( const bool b ){
	int opts;

	opts = fcntl ( _sockfd, F_GETFL );

	if ( opts < 0 ){
	    return;
	}

	if ( b ){
	    opts = ( opts | O_NONBLOCK );
	}else{
	    opts = ( opts & ~O_NONBLOCK );
	}
	fcntl ( _sockfd,  F_SETFL,opts );
}
