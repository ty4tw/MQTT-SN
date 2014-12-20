/*
 *   Defines.h
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
#ifndef  DEFINES_H_
#define  DEFINES_H_

/*=================================
 *    Network  Selection
 =================================*/
#if ! defined(NETWORK_UDP) && ! defined (NETWORK_XXXXX)
#define NETWORK_XBEE
#endif

#if ! defined(NETWORK_XBEE) && ! defined (NETWORK_XXXXX)
#define NETWORK_UDP
#endif

#if ! defined(NETWORK_XBEE) && ! defined (NETWORK_UDP)
#define NETWORK_XXXXX
#endif

/*=================================
 *    CPU TYPE
 ==================================*/
#define CPU_LITTLEENDIANN
//#define CPU_BIGENDIANN

/*=================================
 *    Debug LOG
 ==================================*/
//#define DEBUG_NWSTACK

/*=================================
      Debug Print functions
 ==================================*/
#ifdef  DEBUG_NWSTACK
  #define D_NWSTACK(...)  printf(__VA_ARGS__)
#else
  #define D_NWSTACK(...)
#endif

/*=================================
 *    Data Type
 ==================================*/
typedef unsigned char  uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int   uint32_t;


typedef struct {
	long baudrate;
	char* device;
	unsigned int flag;
}XBeeConfig;

typedef struct {
	char* ipAddress;
	uint16_t gPortNo;
	uint16_t uPortNo;
}UdpConfig;

typedef struct {
	uint8_t  param1;
	uint16_t param2;
	uint16_t param3;
}XXXXXConfig;

#ifdef NETWORK_XBEE
#define NETWORK_CONFIG  XBeeConfig
#endif

#ifdef NETWORK_UDP
#define NETWORK_CONFIG UdpConfig
#endif

#ifdef NETWORK_XXXXX
#define NETWORK_CONFIG XXXXXConfig
#endif

#endif  /*  DEFINES_H_  */
