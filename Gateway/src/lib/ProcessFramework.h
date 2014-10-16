/*
 * ProcessFramework.h
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

#ifndef PROCESSFRAMEWORK_H_
#define PROCESSFRAMEWORK_H_

#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <list>
#include <queue>
#include <exception>
#include <string>
#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <string>
#include <arpa/inet.h>
#include "Defines.h"
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/shm.h>

#define TOMYFRAME_PARAM_MAX     128
#define TOMYFRAME_CONFIG_FILE      "/usr/local/etc/tomygateway/config/param.conf"

#define TOMYFRAME_RINGBUFFER_KEY   "/usr/local/etc/tomygateway/config/ringbuffer.key"
#define TOMYFRAME_RB_MUTEX_KEY     "/usr/local/etc/tomygateway/config/rbmutex.key"
#define TOMYFRAME_RB_SEMAPHOR_NAME "/rbsemaphor"

#define LOGWRITE theProcess->putLog
//#define LOGWRITE printf
#define RINGBUFFER_SIZE 16384
#define PROCESS_LOG_BUFFER_SIZE  2048

#define ERRNO_SYS_01  1   // Application Frame Error

using namespace std;

/*=================================
 *    Data Type
 ==================================*/
typedef unsigned char  uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int   uint32_t;

/*=====================================
         Class Mutex
  ====================================*/
class Mutex{
public:
	Mutex();
	Mutex(const char* name);
	~Mutex();
	void lock(void);
	void unlock(void);

private:
	pthread_mutex_t _mutex;
	pthread_mutex_t* _pmutex;
	int   _shmid;
};

/*=====================================
         Class Semaphore
  ====================================*/
class Semaphore{
public:
	Semaphore();
	Semaphore(unsigned int val);
	Semaphore(const char* name, unsigned int val);
	~Semaphore();
	void post(void);
	void wait(void);
	void timedwait(uint16_t millsec);

private:
	sem_t* _psem;
	sem_t  _sem;
	char*  _name;
};

/*=====================================
         Class EventQue
  ====================================*/
template <class T>
class EventQue{
public:
	EventQue();
	~EventQue();
	T*  wait(void);
	T*  timedwait(uint16_t millsec);
    int post(T*);
    int size();

private:
    queue<T*> _que;
    Mutex         _mutex;
    Semaphore     _sem;
};

/*=====================================
        Class RingBuffer
 =====================================*/
class RingBuffer{
public:
	RingBuffer();
	~RingBuffer();
	void put(char* buffer);
	int get(char* buffer, int bufferLength);
	void reset();
private:
	void* _shmaddr;
	uint16_t* _length;
	uint16_t* _start;
	uint16_t* _end;
	char* _buffer;
	int _shmid;
	Mutex* _pmx;
	bool _createFlg;
};

/*=====================================
         Class Thread
  ====================================*/
#define MAGIC_WORD_FOR_TASK \
	public: void EXECRUN(){try{run();}catch(Exception& ex){ex.writeMessage();\
    if(ex.isFatal()){stopProcess();}}catch(...){throw;}}

class Runnable{
public:
	Runnable(){}
	virtual ~Runnable(){}
	virtual void initialized(int argc, char** argv){}
	virtual void EXECRUN(){}
};

class Thread : virtual public Runnable{
public:
	Thread();
    ~Thread();
	int start(void);
	int join(void);
	int cancel(void);
	static pthread_t getID();
	static bool equals(pthread_t*, pthread_t*);
	void initialize(int argc, char** argv);
	void stopProcess(void);
private:
	pthread_t _threadID;
	Semaphore* _stopProcessEvent;
	void (*_initializePtr)(int argc, char** argv);

	static void* _run(void*);

};

/*=====================================
         Class Process
  ====================================*/
class Process{
public:
	Process();
	virtual ~Process();
	void initialize(int argc, char** argv);
	virtual void run();
	int getArgc();
	char** getArgv();
	char*  getArgv(char option);
	int    getParam(const char* param, char* value);
	void   putLog(const char* format, ...);
	void   resetRingBuffer();
	int    checkSignal();
	const char*  getLog(void);
private:
	int _argc;
	char** _argv;
	RingBuffer* _rb;
	Mutex _mt;
	Semaphore* _rbsem;
	char _rbdata[PROCESS_LOG_BUFFER_SIZE + 1];

};

extern Process* theProcess;

/*=====================================
         Class MultiTaskProcess
  ====================================*/
class MultiTaskProcess: public Process{
	friend int main(int, char**);
	friend class Thread;
public:
	MultiTaskProcess();
	virtual ~MultiTaskProcess();
	void attach(Thread* thread);
	int getArgc();
	char** getArgv();
	char*  getArgv(char option);
	int    getParam(const char* param, char* value);
protected:
	void initialize(int argc, char** argv);
	void run();
	Semaphore* getStopProcessEvent();

private:
	list<Thread*> _threadList;
	Semaphore _stopProcessEvent;
	Mutex  _mutex;
};

extern MultiTaskProcess* theMultiTask;


/*=====================================
        Class Exception
 =====================================*/
enum ExceptionType {
	ExInfo = 0,
	ExWarn,
	ExError,
	ExFatal,
	ExDebug
};

#define THROW_EXCEPTION(type, exceptionNo, message) \
	throw Exception(type, exceptionNo, message, __FILE__, __func__, __LINE__)


class Exception : public exception {
public:
    Exception(const ExceptionType type, const int exNo, const string& message);
    Exception(const ExceptionType type, const int exNo, const string& message,
    		   const char*, const char*, const int);
    virtual ~Exception() throw();
    const char* getFileName();
    const char* getFunctionName();
    const int getLineNo();
    const int getExceptionNo();
    virtual const char* what() const throw();
    void writeMessage();
    bool isFatal();

private:
    const char* strType();
    ExceptionType _type;
    int _exNo;
    string _message;
    const char* _fileName;
    const char* _functionName;
    int _line;
};


/*============================================
                Timer
 ============================================*/
class Timer {
public:
    Timer();
    void start(uint32_t msec = 0);
    bool isTimeup(uint32_t msec);
    bool isTimeup(void);
    void stop();
private:
    struct timeval _startTime;
    uint32_t _millis;
};

/*=====================================
        Class EventQue
 =====================================*/
template<class T> EventQue<T>::EventQue(){

}

template<class T> EventQue<T>::~EventQue(){
	_mutex.lock();
	while(!_que.empty()){
		delete _que.front();
		_que.pop();
	}
	_mutex.unlock();
}

template<class T> T* EventQue<T>::wait(void){
	T* ev;
	_sem.wait();
	_mutex.lock();
	ev = _que.front();
	_que.pop();
	_mutex.unlock();
	return ev;
}

template<class T> T* EventQue<T>::timedwait(uint16_t millsec){
	T* ev;
	_sem.timedwait(millsec);
	_mutex.lock();

	if(_que.empty()){
		ev = new T();
		ev->setTimeout();
	}else{
		ev = _que.front();
		_que.pop();
	}
	_mutex.unlock();
	return ev;
}

template<class T> int EventQue<T>::post(T* ev){
	_mutex.lock();
	_que.push(ev);
	_mutex.unlock();
	_sem.post();
	return 0;
}

template<class T> int EventQue<T>::size(){
	_mutex.lock();
	int sz = _que.size();
	_mutex.unlock();
	return sz;
}


#endif /* PROCESSFRAMEWORK_H_ */
