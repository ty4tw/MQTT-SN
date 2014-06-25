/*
 * ProcessFramework.cpp
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

#include "ProcessFramework.h"
#include "Defines.h"
#include "ErrorMessage.h"
#include <string.h>
#include <exception>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

using namespace std;
extern const char* theCmdlineParameter;
extern int optind;

/*=====================================
        Global functions & variables
 ======================================*/
Process* theProcess = 0;
MultiTaskProcess* theMultiTask = 0;

int main(int argc, char** argv){
	theProcess->initialize(argc, argv);
	theProcess->run();
	return 0;
}

uint8_t* mqcalloc(uint8_t len){
	uint8_t* pos = (uint8_t*)calloc(len, sizeof(uint8_t));
	if( pos == 0){
		THROW_EXCEPTION(ExFatal, ERRNO_SYS_01, "can't allocate memory.");
	}
	return pos;
}

#ifdef CPU_LITTLEENDIANN

/*--- For Little endianness ---*/

uint16_t getUint16(uint8_t* pos){
	uint16_t val = ((uint16_t)*pos++ << 8);
	return val += *pos;
}

void setUint16(uint8_t* pos, uint16_t val){
    *pos++ = (val >> 8) & 0xff;
	*pos   = val & 0xff;
}

uint32_t getUint32(uint8_t* pos){
    uint32_t val = (uint32_t(*pos++ <<  24));
	val += (uint32_t(*pos++ << 16));
	val += (uint32_t(*pos++ <<  8));
	return val += *pos++;
}

void setUint32(uint8_t* pos, uint32_t val){
	*pos++ = (val >> 24) & 0xff;
	*pos++ = (val >> 16) & 0xff;
	*pos++ = (val >>  8) & 0xff;
	*pos   =  val & 0xff;
}

#endif
#ifdef CPU_BIGENDIANN
/*--- For Big endianness ---*/

uint16_t getUint16(uint8_t* pos){
  uint16_t val = *pos++;
  return val += ((uint16_t)*pos++ << 8);
}

void setUint16(uint8_t* pos, uint16_t val){
	*pos++ =  val & 0xff;
	*pos   = (val >>  8) & 0xff;
}

long getUint32(uint8_t* pos){
    long val = (uint32_t(*(pos + 3)) << 24) +
        (uint32_t(*(pos + 2)) << 16) +
        (uint32_t(*(pos + 1)) <<  8);
        return val += *pos;
}

void setUint32(uint8_t* pos, uint32_t val){
    *pos++ =  val & 0xff;
    *pos++ = (val >>  8) & 0xff;
    *pos++ = (val >> 16) & 0xff;
    *pos   = (val >> 24) & 0xff;
}


#endif


void utfSerialize(uint8_t* pos, string str){
	setUint16(pos, (uint16_t)str.size());
	str.copy((char*)pos + 2, str.size(), 0);
}

char theCurrentTime[32];

char* currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    tstruct = *localtime(&now);
    strftime(theCurrentTime, sizeof(theCurrentTime), "%Y%m%d %H%M%S", &tstruct);
    return theCurrentTime;
}

/*=====================================
         Class MultiTaskProcess
  ======================================*/
MultiTaskProcess::MultiTaskProcess(){
	theMultiTask = this;
	_argc = 0;
	_argv = 0;
}

MultiTaskProcess::~MultiTaskProcess(){

}

void MultiTaskProcess::initialize(int argc, char** argv){
	_argc = argc;
	_argv = argv;
	list<Thread*>::iterator thread = _threadList.begin();
	while( thread != _threadList.end()){
		(*thread)->initialize(argc, argv);
		++thread;
	}
}

void MultiTaskProcess::run(){
	list<Thread*>::iterator thread = _threadList.begin();
	while(thread != _threadList.end()){
		(*thread)->start();
		thread++;
	}

	_stopProcessEvent.wait();

	thread = _threadList.begin();
	while(thread != _threadList.end()){
		(*thread)->cancel();
		(*thread)->join();
		thread++;
	}
}

Semaphore* MultiTaskProcess::getStopProcessEvent(){
	return &_stopProcessEvent;
}

void MultiTaskProcess::attach(Thread* thread){
	_threadList.push_back(thread);
}

int MultiTaskProcess::getArgc(){
	return _argc;
}

char** MultiTaskProcess::getArgv(){
	return _argv;
}

char* MultiTaskProcess::getArgv(char option){
	char arg;
	_mutex.lock();
	optind = 1;
	while((arg = getopt(_argc, _argv, theCmdlineParameter))!= -1 && (arg != 255)){
		if(arg == option){
			_mutex.unlock();
			return optarg;
		}
	}
	_mutex.unlock();
	return 0;
}

/*=====================================
        Class Thread
 =====================================*/
Thread::Thread(){
	_stopProcessEvent = theMultiTask->getStopProcessEvent();
	_threadID = 0;
	_stopProcessEvent = 0;
}

Thread::~Thread(){

}

void* Thread::_run(void* runnable){
	static_cast<Runnable*>(runnable)->EXECRUN();
	return 0;
}

void Thread::initialize(int argc, char** argv){

}

pthread_t Thread::getID(){
	return pthread_self();
}

bool Thread::equals(pthread_t *t1, pthread_t *t2){
		return (pthread_equal(*t1, *t2) ? false : true);
}

int Thread::start(void){
	Runnable *runnable = this;
	return pthread_create(&_threadID, 0, _run, runnable);
}

int Thread::join(void){
	return pthread_join(_threadID, 0);
}

int Thread::cancel(void){
	return pthread_cancel(_threadID);
}


void Thread::stopProcess(void){
	_stopProcessEvent->post();
}

/*=====================================
        Class Mutex
 =====================================*/

Mutex::Mutex(void){
	pthread_mutexattr_t  attr;
	pthread_mutexattr_init(&attr);
	pthread_mutex_init(&_mutex, &attr);
	pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_ERRORCHECK);
}

Mutex::~Mutex(void){
	pthread_mutex_lock(&_mutex);
	pthread_mutex_unlock(&_mutex);
	pthread_mutex_destroy(&_mutex);
}

void Mutex::lock(void){
	try{
		if(pthread_mutex_lock(&_mutex)){
			throw " The same thread can't aquire a mutex twice\n";
		}
	}catch(char* errmsg){
		cerr << "Fatal exception Mutex::lock(): " << errmsg;
	}


}

void Mutex::unlock(void){
	try{
		if(pthread_mutex_unlock(&_mutex)){
			throw "Can't release a mutex\n";
		}
	}catch(char* errmsg){
		cerr << "Fatal exception Mutex::unlock(): " << errmsg;
	}
}

/*=====================================
        Class Semaphore
 =====================================*/

Semaphore::Semaphore(){
	Semaphore(0);
}

Semaphore::Semaphore(unsigned int val){
	sem_init(&_sem, 0, val);
}

Semaphore::~Semaphore(){
	sem_destroy(&_sem);
}

void Semaphore::post(void){
	sem_post(&_sem);
}

void Semaphore::wait(void){
	sem_wait(&_sem);
}

void Semaphore::timedwait(uint16_t millsec){
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	ts.tv_sec += millsec / 1000;
	ts.tv_nsec = (millsec % 1000) * 1000000;
	sem_timedwait(&_sem, &ts);
}

/*=====================================
        Class Exception
 ======================================*/
Exception::Exception(const ExceptionType type, const int exNo, const string& message){
	_message = message;
	_type = type;
	_exNo = exNo;
	_fileName = 0;
	_functionName = 0;
	_line = 0;
}

Exception::Exception(const ExceptionType type, const int exNo, const string& message,
		                const char* file, const char* function, const int line){
	_message = message;
	_type = type;
	_exNo = exNo;
	_fileName = file;
	_functionName = function;
	_line = line;
}

Exception::~Exception() throw(){

}

const char* Exception::what() const throw() {
    return _message.c_str();
}

const char* Exception::getFileName(){
        return _fileName;
}

const char* Exception::getFunctionName(){
        return _functionName;
}

const int Exception::getLineNo(){
        return _line;
}

const int Exception::getExceptionNo(){
        return _exNo;
}

bool Exception::isFatal(){
	return _type == ExFatal;
}

const char* Exception::strType(){
	switch(_type){
	case ExInfo:
		return "Info";
		break;
	case ExWarn:
		return "Warn";
		break;
	case ExError:
		return "Error";
		break;
	case ExFatal:
		return "Fatal";
		break;
	case ExDebug:
		return "Debug";
		break;
	default:
		break;
	}
	return "";
}

void Exception::writeMessage(){

	fprintf(stdout, "%s %5s:%-6d   %s  line %-4d %s() : %s\n",
			currentDateTime(), strType(), getExceptionNo(), getFileName(), getLineNo(), getFunctionName(), what());
}

/*=========================================
             Class Timer
 =========================================*/

Timer::Timer(){
  stop();
}

void Timer::start(uint32_t msec){
  gettimeofday(&_startTime, 0);
  _millis = msec;
}

bool Timer::isTimeup(){
  return isTimeup(_millis);
}

bool Timer::isTimeup(uint32_t msec){
    struct timeval curTime;
    long secs, usecs;
    if (_startTime.tv_sec == 0){
        return false;
    }else{
        gettimeofday(&curTime, 0);
        secs  = (curTime.tv_sec  - _startTime.tv_sec) * 1000;
        usecs = (curTime.tv_usec - _startTime.tv_usec) / 1000.0;
        return ((secs + usecs) > (long)msec);
    }
}

void Timer::stop(){
  _startTime.tv_sec = 0;
  _millis = 0;
}

