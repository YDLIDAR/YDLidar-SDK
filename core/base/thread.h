#pragma once
#include "v8stdint.h"

#ifdef _WIN32
#include <conio.h>
#include <windows.h>
#include <io.h>
#include <process.h>
#else
#include <pthread.h>
#include <assert.h>
#endif
#include "timer.h"

#if defined(__ANDROID__)
#define    pthread_cancel(x) 0
#endif

#define CLASS_THREAD(c , x ) Thread::ThreadCreateObjectFunctor<c, &c::x>(this)

class Thread {
 public:

  template <class CLASS, int (CLASS::*PROC)(void)> static Thread
  ThreadCreateObjectFunctor(
    CLASS *pthis) {
    return createThread(createThreadAux<CLASS, PROC>, pthis);
  }

  template <class CLASS, int (CLASS::*PROC)(void) > static _size_t THREAD_PROC
  createThreadAux(
    void *param) {
    return (static_cast<CLASS *>(param)->*PROC)();
  }

  static Thread createThread(thread_proc_t proc, void *param = NULL) {
    Thread thread_(proc, param);
#if defined(_WIN32)
    thread_._handle = (_size_t)(_beginthreadex(NULL, 0,
                                (unsigned int (__stdcall *)(void *))proc, param,
                                0, NULL));
#else
    assert(sizeof(thread_._handle) >= sizeof(pthread_t));

    int rv = pthread_create((pthread_t *)&thread_._handle, NULL,
                            (void *(*)(void *))proc,
                            param);

    if (rv != 0) {
      fprintf(stderr, "failed to create thread: %s\n", strerror(rv));
    }

#endif
    return thread_;
  }

 public:
  explicit Thread(): _param(NULL), _func(NULL), _handle(0),
    doing_join_state(false), thread_finished_(true) {
    pthread_mutex_init(&mutex, NULL);
    pthread_mutex_init(&thread_finished_lock, NULL);
  }
  virtual ~Thread() {
    pthread_mutex_destroy(&mutex);
    pthread_mutex_destroy(&thread_finished_lock);
  }
  _size_t getHandle() {
    return _handle;
  }
  int terminate() {
    if (isDoingLock()) {
      return 0;
    }

    if (!this->_handle) {
      updateDoingState(false);
      return 0;
    }

    int ret = 0;
#if defined(_WIN32)

    if (TerminateThread(reinterpret_cast<HANDLE>(this->_handle), -1)) {
      CloseHandle(reinterpret_cast<HANDLE>(this->_handle));
      this->_handle = NULL;
      ret = 0;
    } else {
      ret = -2;
    }

#else
    ret = pthread_cancel((pthread_t)this->_handle);
#endif
    updateDoingState(false);
    return ret;
  }
  void *getParam() {
    return _param;
  }
  int join(unsigned long timeout = -1) {
    if (isDoingLock()) {
      return 0;
    }

    if (!this->_handle) {
      updateDoingState(false);
      return 0;
    }

    int ret = 0;
#if defined(_WIN32)

    switch (WaitForSingleObject(reinterpret_cast<HANDLE>(this->_handle), timeout)) {
      case WAIT_OBJECT_0:
        CloseHandle(reinterpret_cast<HANDLE>(this->_handle));
        this->_handle = NULL;
        ret = 0;
        break;

      case WAIT_ABANDONED:
        ret = -2;
        break;

      case WAIT_TIMEOUT:
        ret = -1;
        break;
    }

#else
    UNUSED(timeout);
    void *res;
    int s;
//    s = pthread_cancel((pthread_t)(this->_handle));

//    if (s != 0) {
//    }

    s = pthread_join((pthread_t)(this->_handle), &res);

    if (s != 0) {
    }

    if (res == PTHREAD_CANCELED) {
      printf("#%lu thread has been canceled\n", this->_handle);
      this->_handle = 0;
      updateThreadState(true);
    } else {
      waitingThreadFinished();

      if (!isThreadFinshed()) {
        s = pthread_cancel((pthread_t)(this->_handle));

        if (s != 0) {
        }

        s = pthread_join((pthread_t)(this->_handle), &res);

        if (s != 0) {
        }

        if (res == PTHREAD_CANCELED) {
          printf("##%lu thread has been canceled\n", this->_handle);
          this->_handle = 0;
        } else {
          printf("#%lu thread wasn't canceled (shouldn't happen!)\n", this->_handle);
        }
      } else {
        printf("#%lu thread has been finished\n", this->_handle);
        this->_handle = 0;
      }
    }

#endif
    updateDoingState(false);
    return ret;
  }

  bool operator== (const Thread &right) {
    return this->_handle == right._handle;
  }

  bool isDoingLock() {
    bool flag = false;
    pthread_mutex_lock(&mutex);
    flag = doing_join_state;

    if (!doing_join_state) {
      doing_join_state = true;
    }

    pthread_mutex_unlock(&mutex);
    return flag;
  }
  void updateDoingState(bool state) {
    pthread_mutex_lock(&mutex);
    doing_join_state = state;
    pthread_mutex_unlock(&mutex);
  }

  void waitingThreadFinished() {
    int time_count = 0;

    while (!isThreadFinshed() && time_count < 11) {
      delay(101);
      time_count++;
    }
  }

  bool isThreadFinshed()  {
    bool flag = false;
    pthread_mutex_lock(&thread_finished_lock);
    flag = thread_finished_;
    pthread_mutex_unlock(&thread_finished_lock);
    return flag;
  }

  void updateThreadState(bool finished) {
    pthread_mutex_lock(&thread_finished_lock);
    thread_finished_ = finished;
    pthread_mutex_unlock(&thread_finished_lock);
  }

 protected:
  explicit Thread(thread_proc_t proc, void *param): _param(param), _func(proc),
    _handle(0), doing_join_state(false), thread_finished_(true) {
    pthread_mutex_init(&mutex, NULL);
    pthread_mutex_init(&thread_finished_lock, NULL);
  }
  void *_param;
  thread_proc_t _func;
  _size_t _handle;
  bool doing_join_state ;  ///<
  pthread_mutex_t mutex;
  bool thread_finished_;
  pthread_mutex_t thread_finished_lock;

};

