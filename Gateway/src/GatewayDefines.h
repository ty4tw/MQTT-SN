/*
 * GatewayDefines.h
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

#ifndef GATEWAYDEFINES_H_
#define GATEWAYDEFINES_H_

#define GATEWAY_VERSION "(Ver 1.1.0)"

#define BROKER      "Broker"
#define GREEN_BROKER "\x1b[0m\x1b[32mBroker\x1b[0m\x1b[37m"
#define GATEWAY     "Gateway"
#define CLIENT      "Client"
#define LEFTARROW   "<---"
#define RIGHTARROW  "--->"

#define SEND_COMPLETE  " \x1b[0m\x1b[32m[Tx completed]\x1b[0m\x1b[37m\n"

#define FORMAT      "%s   %-14s%-8s%-44s%s"
#define FORMAT1     "%s   %-14s%-8s%-26s%s\n"
#define FORMAT2     "\n%s   %-14s%-8s%-26s%s\n"

#define RED_FORMAT1      "%s   \x1b[0m\x1b[31m%-14s%-8s%-44s\x1b[0m\x1b[37m%s\n"
#define RED_FORMAT2    "\n%s   \x1b[0m\x1b[31m%-14s%-8s%-26s\x1b[0m\x1b[37m%s\n"

#define GREEN_FORMAT     "%s   \x1b[0m\x1b[32m%-14s%-8s%-44s\x1b[0m\x1b[37m%s"
#define GREEN_FORMAT1    "%s   \x1b[0m\x1b[32m%-14s%-8s%-26s\x1b[0m\x1b[37m%s\n"
#define GREEN_FORMAT2  "\n%s   \x1b[0m\x1b[32m%-14s%-8s%-26s\x1b[0m\x1b[37m%s\n"

#define YELLOW_FORMAT1   "%s   \x1b[0m\x1b[33m%-14s%-8s%-26s\x1b[0m\x1b[37m%s\n"
#define YELLOW_FORMAT2 "\n%s   \x1b[0m\x1b[33m%-14s%-8s%-26s\x1b[0m\x1b[37m%s\n"

#define BLUE_FORMAT      "%s   \x1b[0m\x1b[34m%-14s%-8s%-44s\x1b[0m\x1b[37m%s"
#define BLUE_FORMAT1     "%s   \x1b[0m\x1b[34m%-14s%-8s%-26s\x1b[0m\x1b[37m%s\n"
#define BLUE_FORMAT2   "\n%s   \x1b[0m\x1b[34m%-14s%-8s%-26s\x1b[0m\x1b[37m%s\n"

#define CYAN_FORMAT1     "%s   \x1b[0m\x1b[36m%-14s%-8s%-26s\x1b[0m\x1b[37m%s\n"
#define SYAN_FORMAT2   "\n%s   \x1b[0m\x1b[36m%-14s%-8s%-26s\x1b[0m\x1b[37m%s\n"

/*===========================================
 *   Gateway Control Constants
 ===========================================*/

#define BROKER_HOST_NAME  "localhost"
#define BROKER_PORT       "1883"


#define KEEP_ALIVE_TIME   900    // 900 sec = 15 min

#define TIMEOUT_PERIOD     10    //  10 sec = 10 sec

#define SEND_UNIXTIME_TIME 30    // 30sec after KEEP_ALIVE_TIME



#define MAX_CLIENT_NODES  500

/*==========================================================
 *           Light Indicators
 *
 *  Change Makefile line25 and 26
 *   Line25 comment out  #LDADD := -lpthread -lrt
 *   Line26 uncomment    LDADD := -lpthread -lrt -lwiringPi
 *
 ===========================================================*/
//#define RASPBERRY_LIGHT_INDICATOR

#define LIGHT_INDICATOR_GREEN   4    // RPi connector 16
#define LIGHT_INDICATOR_RED     5    // RPi connector 18
#define LIGHT_INDICATOR_BLUE    6    // RPi connector 22

#endif /* GATEWAYDEFINES_H_ */
