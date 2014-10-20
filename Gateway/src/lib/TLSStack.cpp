/*
 * TLSStack.cpp
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
#include "TLSStack.h"
#include <string.h>
#include <openssl/ssl.h>
#include <openssl/err.h>
#include <openssl/rand.h>

using namespace std;

int TLSStack::_numOfInstance = 0;
SSL_CTX* TLSStack::_ctx = 0;
SSL_SESSION* TLSStack::_session = 0;

/*========================================
       Class TLSStack
 =======================================*/
TLSStack::TLSStack(){
    TLSStack(true);
}

TLSStack::TLSStack(bool secure):TCPStack(){
	char error[256];
	if(secure){
	    _numOfInstance++;
		if(_ctx == 0){
			SSL_load_error_strings();
			SSL_library_init();
			_ctx = SSL_CTX_new(TLSv1_2_client_method());
			if(_ctx == 0){
				ERR_error_string_n(ERR_get_error(), error, sizeof(error));
				LOGWRITE("SSL_CTX_new() %s\n",error);
				THROW_EXCEPTION(ExFatal, ERRNO_SYS_01, "can't create SSL context.");
			}
			if(!SSL_CTX_load_verify_locations(_ctx, 0,TLS_CA_DIR)){
				ERR_error_string_n(ERR_get_error(), error, sizeof(error));
				LOGWRITE("SSL_CTX_load_verify_locations() %s\n",error);
				THROW_EXCEPTION(ExFatal, ERRNO_SYS_01, "can't load CA_LIST.");
			}
		}
	}
    _ssl = 0;
    _disconReq = false;
    _secureFlg = secure;
    _busy = false;
}

TLSStack::~TLSStack(){
	if(_secureFlg){
		_numOfInstance--;
	}
    if(_ssl){
		SSL_free(_ssl);
	}
    if(_session && _numOfInstance == 0){
    	SSL_SESSION_free(_session);
    	_session = 0;
    }
    if(_ctx && _numOfInstance == 0){
    	SSL_CTX_free(_ctx);
    	_ctx = 0;
        ERR_free_strings();
    }
}

bool TLSStack::connect ( const char* host, const char* service ){
	char errmsg[256];
	int rc = 0;
	char peer_CN[256];
	SSL_SESSION* sess = 0;
	X509* peer;

	if(isValid()){
		return false;
	}
	if(!TCPStack::connect(host, service)){
		return false;
	}
	if(!_secureFlg){
		return true;
	}

	SSL* ssl = SSL_new(_ctx);
	if(ssl == 0){
		ERR_error_string_n(ERR_get_error(), errmsg, sizeof(errmsg));
		LOGWRITE("SSL_new()  %s\n",errmsg);
		return false;
	}

	rc = SSL_set_fd(ssl, TCPStack::getSock());
	if(rc == 0){
		SSL_free(ssl);
		return false;
	}

	if(_session){
		rc = SSL_set_session(ssl, sess);
	}else{
		rc = SSL_connect(ssl);
	}
	if(rc != 1){
		ERR_error_string_n(ERR_get_error(), errmsg, sizeof(errmsg));
		LOGWRITE("SSL_connect() %s\n",errmsg);
		SSL_free(ssl);
		return false;
	}

	if(SSL_get_verify_result(ssl) != X509_V_OK){
		LOGWRITE("SSL_get_verify_result() error: Certificate doesn't verify.\n");
		SSL_free(ssl);
		return false;
	}

	peer = SSL_get_peer_certificate(ssl);
	X509_NAME_get_text_by_NID(X509_get_subject_name(peer), NID_commonName, peer_CN, 256);
	if(strcasecmp(peer_CN, host)){
		LOGWRITE("SSL_get_peer_certificate() error: Broker dosen't much host name.\n");
		SSL_free(ssl);
		return false;
	}
	if(_session == 0){
		_session = sess;
	}
	_ssl = ssl;
	return true;
}


int TLSStack::send (const uint8_t* buf, uint16_t length  ){
	char errmsg[256];
	fd_set rset;
	fd_set wset;
	bool writeBlockedOnRead = false;
	int bpos = 0;

	if(_secureFlg){
		_mutex.lock();
		_busy = true;

		while(true){
			FD_ZERO(&rset);
			FD_ZERO(&wset);
			FD_SET(getSock(), &rset);
			FD_SET(getSock(), &wset);

			int activity =  select( getSock() + 1 , &rset , &wset , 0 , 0);
			if (activity > 0){
				if(FD_ISSET(getSock(),&wset) ||
					(writeBlockedOnRead && FD_ISSET(getSock(),&rset))){

					writeBlockedOnRead = false;
					int r = SSL_write(_ssl, buf + bpos, length);

					switch(SSL_get_error(_ssl, r)){
					case SSL_ERROR_NONE:
						length -= r;
						bpos += r;
						if( length == 0){
							_busy = false;
							_mutex.unlock();
							return bpos;
						}
						break;
					case SSL_ERROR_WANT_WRITE:
						break;
					case SSL_ERROR_WANT_READ:
						writeBlockedOnRead = true;
						break;
					default:
						ERR_error_string_n(ERR_get_error(), errmsg, sizeof(errmsg));
						LOGWRITE("TLSStack::send() default %s\n",errmsg);
						_busy = false;
						_mutex.unlock();
						return -1;
					}
				}
			}
		}
	}else{
		return TCPStack::send (buf, length);
	}
}


int TLSStack::recv ( uint8_t* buf, uint16_t len ){
	char errmsg[256];
	bool writeBlockedOnRead = false;
	bool readBlockedOnWrite = false;
	bool readBlocked = false;
	int rlen = 0;
	int bpos = 0;
	fd_set rset;
	fd_set wset;


	if(_secureFlg){
		if(_busy){
			return 0;
		}
		_mutex.lock();
		_busy = true;

	loop:
		do{
			readBlockedOnWrite = false;
			readBlocked = false;

			rlen = SSL_read(_ssl, buf + bpos, len - bpos);

			switch (SSL_get_error(_ssl, rlen)){
				case SSL_ERROR_NONE:
					_busy = false;
					_mutex.unlock();
					return rlen + bpos;
					break;
				case SSL_ERROR_ZERO_RETURN:
					SSL_shutdown(_ssl);
					_ssl = 0;
					TCPStack::close();
					_busy = false;
					_mutex.unlock();
					return -1;
					break;
				case SSL_ERROR_WANT_READ:
					readBlocked = true;
					break;
				case SSL_ERROR_WANT_WRITE:
					readBlockedOnWrite = true;
					break;
				default:
					ERR_error_string_n(ERR_get_error(), errmsg, sizeof(errmsg));
					LOGWRITE("TLSStack::recv() default %s\n", errmsg);
					_busy = false;
					_mutex.unlock();
					return -1;
			}
		}while(SSL_pending(_ssl) && !readBlocked);

		bpos += rlen;
		while(true){
			FD_ZERO(&rset);
			FD_ZERO(&wset);
			FD_SET(getSock(), &rset);
			FD_SET(getSock(), &wset);

			int activity =  select( getSock() + 1 , &rset , &wset , 0 , 0);
			if (activity > 0){
				if((FD_ISSET(getSock(),&rset) && !writeBlockedOnRead) ||
					(readBlockedOnWrite && FD_ISSET(getSock(),&wset))){
					goto loop;
				}
			}else{
				ERR_error_string_n(ERR_get_error(), errmsg, sizeof(errmsg));
				LOGWRITE("TLSStack::recv() select %s\n",errmsg);
				_busy = false;
				_mutex.unlock();
				return -1;
			}
		}
	}
	return TCPStack::recv (buf, len);
}


bool TLSStack::isValid(){
	if(!_secureFlg){
		return TCPStack::isValid();
	}
	if(_ssl){
		if(_disconReq){
			SSL_shutdown(_ssl);
			_ssl = 0;
			TCPStack::close();
			_disconReq = false;
		}else{
			return true;
		}
	}
	return false;
}

void TLSStack::disconnect(){
    if (_ssl){
    	_disconReq = true;
    	TCPStack::disconnect();
    }
}


int TLSStack::getSock(){
	return TCPStack::getSock();
}

SSL* TLSStack::getSSL(){
	if(_secureFlg){
		return _ssl;
	}else{
		return 0;
	}
}

bool TLSStack::isSecure(){
	return _secureFlg;
}
