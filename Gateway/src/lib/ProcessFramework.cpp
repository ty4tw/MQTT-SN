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
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <stdarg.h>
#include <assert.h>
#include <signal.h>

using namespace std;
extern const char* theCmdlineParameter;
extern int optind;

/*=====================================
        Global functions & variables
 ======================================*/
Process* theProcess = 0;
MultiTaskProcess* theMultiTask = 0;
static volatile int theSignaled = 0;

static void signalHandler(int sig){
	assert(sig == SIGINT || sig == SIGHUP || sig == SIGTERM);
	theSignaled = sig;
}

int main(int argc, char** argv){
	try{
		signal(SIGHUP, signalHandler);
		signal(SIGINT, signalHandler);
		signal(SIGTERM, signalHandler);

		theProcess->initialize(argc, argv);
		theProcess->run();
	}catch(Exception& ex){
		ex.writeMessage();
	}
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
         Class Process
  ======================================*/
Process::Process(){
	_argc = 0;
	_argv = 0;
	_rb = new RingBuffer();
	_rbsem = new Semaphore(TOMYFRAME_RB_SEMAPHOR_NAME, 0);
}

Process::~Process(){
	delete(_rb);
	delete(_rbsem);
}

void Process::run(){

}

void Process::initialize(int argc, char** argv){
	_argc = argc;
	_argv = argv;
}

int Process::getArgc(){
	return _argc;
}

char** Process::getArgv(){
	return _argv;
}

char* Process::getArgv(char option){
	char arg;
	optind = 1;
	while((arg = getopt(_argc, _argv, theCmdlineParameter))!= -1 && (arg != 255)){
		if(arg == option){
			return optarg;
		}
	}
	return 0;
}

int Process::getParam(const char* parameter, char* value){
	char str[TOMYFRAME_PARAM_MAX], param[TOMYFRAME_PARAM_MAX];
	FILE *fp;
	int i = 0, j = 0;

	if ((fp = fopen(TOMYFRAME_CONFIG_FILE, "r")) == NULL) {
		LOGWRITE("No config file:[%s]\n", TOMYFRAME_CONFIG_FILE);
		THROW_EXCEPTION(ExFatal, ERRNO_SYS_01, "No config file.");
		return -1;
	}

	while(true) {
		if (fgets(str, TOMYFRAME_PARAM_MAX - 1, fp) == NULL) {
			fclose(fp);
			return -3;
		}
		if (!strncmp(str, parameter,strlen(parameter))) {
			while (str[i++] != '=') {
				;
			}
			while (str[i] != '\n') {
				param[j++] = str[i++];
			}
			param[j] = '\0';

		    for( i = strlen(param)-1; i >= 0 && isspace( param[i] ); i-- ) ;
		    param[i+1] = '\0';
		    for( i = 0; isspace( param[i] ); i++ ) ;
		    if( i > 0 ) {
		        j = 0;
		        while( param[i] ) param[j++] = param[i++];
		        param[j] = '\0';
		    }
			strcpy(value,param);
			fclose(fp);
			return 0;
		}
	}
	fclose(fp);
	return -1;
}


void Process::putLog(const char* format, ...){
	_mt.lock();
	va_list arg;
	va_start(arg, format);
	vsprintf(_rbdata, format, arg);
	va_end(arg);
	if(strlen(_rbdata)){
		_rb->put(_rbdata);
		_rbsem->post();
	}
	_mt.unlock();
}

const char* Process::getLog(){
	int len = 0;
	_mt.lock();
	while((len = _rb->get(_rbdata, PROCESS_LOG_BUFFER_SIZE)) == 0){
		_rbsem->wait();
	}
	*(_rbdata + len) = 0;
	_mt.unlock();
	return _rbdata;
}

void Process::resetRingBuffer(){
	_rb->reset();
}

int Process::checkSignal(){
	return theSignaled;
}
/*=====================================
         Class MultiTaskProcess
  ======================================*/
MultiTaskProcess::MultiTaskProcess(){
	theMultiTask = this;
}

MultiTaskProcess::~MultiTaskProcess(){

}

void MultiTaskProcess::initialize(int argc, char** argv){
	Process::initialize(argc, argv);
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

char* MultiTaskProcess::getArgv(char option){
	_mutex.lock();
	char* arg = Process::getArgv(option);
	_mutex.unlock();
	return arg;
}

int MultiTaskProcess::getParam(const char* parameter, char* value){
	_mutex.lock();
	int rc = Process::getParam(parameter, value);
	_mutex.unlock();
	return rc;
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
	_shmid = 0;
	_pmutex = 0;
}

Mutex::Mutex(const char* fileName){
	pthread_mutexattr_t  attr;

	key_t key = ftok(fileName, 1);

	if((_shmid = shmget(key, sizeof(pthread_mutex_t), IPC_CREAT | 0666)) < 0){
		perror("Mutex");
		THROW_EXCEPTION(ExFatal, ERRNO_SYS_01, "Can't create a shared memory.");
	}
	_pmutex = (pthread_mutex_t*)shmat(_shmid, NULL, 0);
	if(_pmutex  < 0 ){
		perror("Mutex");
		THROW_EXCEPTION(ExFatal, ERRNO_SYS_01, "can't attach shared memory for Mutex.");
	}

	pthread_mutexattr_init(&attr);

	if(pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED) != 0) {
		perror("Mutex pthread_mutexattr_setpshared");
		THROW_EXCEPTION(ExFatal, ERRNO_SYS_01, "Can't create a Mutex.");
	}
	if(pthread_mutex_init(_pmutex, &attr) != 0) {
		perror("Mutex pthread_mutexattr_setpshared");
		THROW_EXCEPTION(ExFatal, ERRNO_SYS_01, "Can't create a Mutex.");
	}
}

Mutex::~Mutex(void){
	if(_pmutex){
		pthread_mutex_lock(_pmutex);
		pthread_mutex_unlock(_pmutex);
		pthread_mutex_destroy(_pmutex);
	}else{
		pthread_mutex_lock(&_mutex);
		pthread_mutex_unlock(&_mutex);
		pthread_mutex_destroy(&_mutex);
	}
	if(_shmid){
		shmctl(_shmid, IPC_RMID, NULL);
	}
}

void Mutex::lock(void){
	if(_pmutex){
		pthread_mutex_lock(_pmutex);
	}else{
		try{
			if(pthread_mutex_lock(&_mutex)){
				throw;
			}
		}catch(char* errmsg){
			perror("Mutex");
			THROW_EXCEPTION(ExFatal, ERRNO_SYS_01, "The same thread can't aquire a mutex twice.");
		}
	}
}

void Mutex::unlock(void){

	if(_pmutex){
		pthread_mutex_unlock(_pmutex);
	}else{
		try {
			if(pthread_mutex_unlock(&_mutex)){
				throw;
			}
		}catch(char* errmsg){
			perror("Mutex");
			THROW_EXCEPTION(ExFatal, ERRNO_SYS_01, "Can't release a mutex.");
		}
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
	_name = 0;
	_psem = 0;
}

Semaphore::Semaphore(const char* name,unsigned int val){
	_psem = sem_open(name, O_CREAT, 0666, val);
	if(_psem == SEM_FAILED ){
		perror("Semaphore");
		THROW_EXCEPTION(ExFatal, ERRNO_SYS_01, "Can't create a Semaphore.");
	}
	_name = (char*)mqcalloc(strlen(name + 1));
	strcpy(_name, name);
}

Semaphore::~Semaphore(){
	if(_name){
		sem_close(_psem);
		sem_unlink(_name);
		free((void*)_name);
	}else{
		sem_destroy(&_sem);
	}
}

void Semaphore::post(void){
	int val = 0;
	if(_psem){
		sem_getvalue(_psem,&val);
		if(val <= 0){
			sem_post(_psem);
		}
	}else{
		sem_getvalue(&_sem,&val);
		if(val <= 0){
			sem_post(&_sem);
		}
	}
}

void Semaphore::wait(void){
	if(_psem){
		sem_wait(_psem);
	}else{
		sem_wait(&_sem);
	}
}

void Semaphore::timedwait(uint16_t millsec){
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	ts.tv_sec += millsec / 1000;
	ts.tv_nsec = (millsec % 1000) * 1000000;
	if(_psem){
		sem_timedwait(_psem, &ts);
	}else{
		sem_timedwait(&_sem, &ts);
	}
}

/*=========================================
             Class RingBuffer
 =========================================*/
RingBuffer::RingBuffer(){
	key_t key = ftok(TOMYFRAME_RINGBUFFER_KEY, 1);

	if((_shmid = shmget(key, RINGBUFFER_SIZE, IPC_CREAT | IPC_EXCL | 0666)) >= 0){
		if((_shmaddr = (uint16_t*)shmat(_shmid, NULL, 0)) > 0 ){
			_length = (uint16_t*)_shmaddr;
			_start = (uint16_t*)_length + sizeof(uint16_t*);
			_end = (uint16_t*)_start + sizeof(uint16_t*);
			_buffer = (char*)_end + sizeof(uint16_t*);
			_createFlg = true;

			*_length = RINGBUFFER_SIZE - sizeof(uint16_t*) * 3 - 16;
			*_start = *_end = 0;
		}else{
			perror("RingBuffer");
			THROW_EXCEPTION(ExFatal, ERRNO_SYS_01, "can't attach shared memory.");
		}
	}else if((_shmid = shmget(key, RINGBUFFER_SIZE, IPC_CREAT | 0666)) >= 0){
		if((_shmaddr = (uint16_t*)shmat(_shmid, NULL, 0)) > 0 ){
			_length = (uint16_t*)_shmaddr;
			_start = (uint16_t*)_length + sizeof(uint16_t*);
			_end = (uint16_t*)_start + sizeof(uint16_t*);
			_buffer = (char*)_end + sizeof(uint16_t*);
			_createFlg = false;
		}else{
			perror("RingBuffer");
			THROW_EXCEPTION(ExFatal, ERRNO_SYS_01, "Can't create a shared memory.");
		}
	}else{
		perror("RingBuffer");
		THROW_EXCEPTION(ExFatal, ERRNO_SYS_01, "Can't create a shared memory.");
	}

	_pmx = new Mutex(TOMYFRAME_RB_MUTEX_KEY);
}

RingBuffer::~RingBuffer(){
	if(_createFlg){
		if(_shmid > 0){
			shmctl(_shmid, IPC_RMID, NULL);
		}
		if(_pmx > 0){
			delete _pmx;
		}
	}else{
		if(_shmid > 0){
			shmdt(_shmaddr);
		}
	}
}

void RingBuffer::put(char* data){
	_pmx->lock();

	uint16_t dlen = strlen(data);
	uint16_t blen = *_length - *_end;

	if(*_end > *_start){
		if(dlen < blen){
			strncpy(_buffer + *_end, data, dlen);
			if(*_end - *_start == 1){ // Buffer is empty.
				*_start = *_end;
			}
			*_end += dlen;
		}else{
			strncpy(_buffer + *_end, data, blen);
			strncpy(_buffer, data + blen, dlen - blen);
			if(*_end - *_start == 1){ // Buffer is empty.
				*_start = *_end;
				*_end = dlen - blen;
			}else{
				*_end = dlen - blen;
				*_start = *_end + 1;
			}
		}
	}else if(*_end == *_start){
		if(dlen < blen){
			strncpy(_buffer + *_end, data, dlen);
			*_end += dlen;
		}else{
			const char* errmsg = "RingBuffer Error: data is too long";
			strcpy(_buffer + *_end, errmsg);
			*_end += strlen(errmsg);
		}
	}else{    // *_end < *_start
		if(dlen < *_start - *_end){
			strncpy(_buffer + *_end, data, dlen);
			*_end += dlen;
			*_start = *_end + 1;
		}else {
			if( dlen < blen){
				strncpy(_buffer + *_end, data, dlen);
				*_end += dlen;
				*_start = *_end + 1;
			}else {
				strncpy(_buffer + *_end, data, blen);
				strncpy(_buffer, data + blen, dlen - blen);
				*_start = *_end;
				*_end = dlen - blen;
			}
		}
	}
	_pmx->unlock();
}

int RingBuffer::get(char* buf, int length){
	int len = 0;
	_pmx->lock();

	if(*_end > *_start){
		if(length > *_end - *_start){
			len = *_end - *_start;
			if (len == 1){
				len = 0;
			}
			strncpy(buf, _buffer + *_start, len);
			*_start = *_end - 1;
		}else{
			len = length;
			strncpy(buf, _buffer + *_start, len);
			*_start = *_start + len;
		}
	}else if(*_end < *_start){
		int blen = *_length - *_start;
		if(length > blen ){
			strncpy(buf, _buffer + *_start, blen);
			*_start = 0;
			if(length - (blen + *_end) > 0){
				strncpy(buf + blen , _buffer, *_end);
				len = blen + *_end;
				if(*_end > 0){
					*_start = *_end - 1;
				}
			}else{
				printf("length=%d blen=%d len = %d\n", length, blen, length - *_end );
				strncpy(buf + blen, _buffer, length - blen);
				len = length;
				*_start = length - blen;
			}
		}else{
			strncpy(buf, _buffer + *_start, length);
			*_start += length;
			len = length;
		}
	}
	_pmx->unlock();
	return len;
}

void RingBuffer::reset(){
	_pmx->lock();
	*_start = *_end = 0;
	_pmx->unlock();
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

