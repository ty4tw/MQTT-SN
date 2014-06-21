/*
 * Topics.h
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

#ifndef TOPICS_H_
#define TOPICS_H_

#include "ProcessFramework.h"
#include "Messages.h"
#include <stdlib.h>

#define MQTTSN_TOPIC_MULTI_WILDCARD   '#'
#define MQTTSN_TOPIC_SINGLE_WILDCARD  '+'

#define MQTTSN_TOPICID_NORMAL 256
#define MQTTSN_TOPICID_PREDEFINED_TIME   0x0001
#define MQTTSN_TOPIC_PREDEFINED_TIME     ("$GW/01")

#define MAX_TOPIC_COUNT   50        // Number of Topic Par ClientNode

/*=====================================
        Class Topic
======================================*/

class Topic {
public:
    Topic();
    Topic(string topic);
    ~Topic();
    uint16_t  getTopicId();
    uint8_t   getTopicLength();
    string*  getTopicName();
    void     setTopicId(uint16_t id);
    void     setTopicName(string topic);

    uint8_t isWildCard(uint8_t* pos);
    bool    isMatch(Topic* wildCard);
private:
    uint16_t  _topicId;
    string _topicStr;
};

/*=====================================
        Class Topics
 ======================================*/
class Topics {
public:
      Topics();
      ~Topics();
      uint16_t  createTopic(string* topic);
      uint16_t  getTopicId(string* topic);
      uint16_t  getNextTopicId();
      Topic*    getTopic(string* topic);
      Topic*    getTopic(uint16_t topicId);
      Topic*    match(string* topic);
      bool     deleteTopic(string* topic);

private:
    uint16_t _nextTopicId;
    uint8_t  _cnt;
    vector<Topic*>  _topics;

};

#endif /* TOPICS_H_ */
