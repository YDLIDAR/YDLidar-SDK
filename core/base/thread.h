#pragma once
#include "v8stdint.h"
#include "timer.h"
#ifdef _WIN32
#include <conio.h>
#include <windows.h>
#include <io.h>
#include <process.h>
#else
#include <pthread.h>
#include <assert.h>
#endif

#if defined(__ANDROID__)
#define pthread_cancel(x) 0
#endif

#define CLASS_THREAD(c, x) Thread::ThreadCreateObjectFunctor<c, &c::x>(this)

namespace ydlidar
{
  namespace core
  {
    namespace base
    {

      class Thread
      {
      public:
        template <class CLASS, int (CLASS::*PROC)(void)>
        static Thread
        ThreadCreateObjectFunctor(CLASS *pthis)
        {
          return createThread(createThreadAux<CLASS, PROC>, pthis);
        }

        template <class CLASS, int (CLASS::*PROC)(void)>
        static _size_t THREAD_PROC
        createThreadAux(void *param)
        {
          return (static_cast<CLASS *>(param)->*PROC)();
        }

        static Thread createThread(thread_proc_t proc, void *param = NULL)
        {
          Thread thread_(proc, param);
#if defined(_WIN32)
          thread_._handle = (_size_t)(_beginthreadex(NULL, 0,
            (unsigned int(__stdcall *)(void *))proc, param, 0, NULL));
#else
          assert(sizeof(thread_._handle) >= sizeof(pthread_t));

          int ret = pthread_create((pthread_t *)&thread_._handle,
                                   NULL, (void *(*)(void *))proc,
                                   param);
          if (ret != 0)
          {
            thread_._handle = 0;
            fprintf(stderr, "[YDLIDAR] Fail to create thread!\n\tError[%s]\n",
              strerror(ret));
          }
#endif
          return thread_;
        }

      public:
        explicit Thread() : _param(NULL), _func(NULL), _handle(0) {}
        virtual ~Thread() {}
        _size_t getHandle()
        {
          return _handle;
        }
        int terminate()
        {
#if defined(_WIN32)

          if (!this->_handle)
          {
            return 0;
          }

          if (TerminateThread(reinterpret_cast<HANDLE>(this->_handle), -1))
          {
            CloseHandle(reinterpret_cast<HANDLE>(this->_handle));
            this->_handle = NULL;
            return 0;
          }
          else
          {
            return -2;
          }

#else
          if (!this->_handle)
          {
            return 0;
          }

          return pthread_cancel((pthread_t)this->_handle);
#endif
        }
        void *getParam()
        {
          return _param;
        }
        //等待线程退出
        int join(unsigned long timeout = -1)
        {
          if (!_handle)
            return 0;

#if defined(_WIN32)
          switch (WaitForSingleObject(reinterpret_cast<HANDLE>(this->_handle), timeout))
          {
          case WAIT_OBJECT_0:
            CloseHandle(reinterpret_cast<HANDLE>(this->_handle));
            this->_handle = NULL;
            return 0;

          case WAIT_ABANDONED:
            return -2;

          case WAIT_TIMEOUT:
            return -1;
          }
#else
          UNUSED(timeout);
          int s = -1;
          uint32_t t = getms();
          s = pthread_cancel((pthread_t)(_handle));
          if (s != 0)
          {
            // return s;
          }
          printf("[YDLIDAR DEBUG] Thread [0x%X] ready to cancel[%d]\n", _handle, s);
          s = pthread_join((pthread_t)(_handle), NULL);
          printf("[YDLIDAR DEBUG] Thread [0x%X] ready to cancel[%d] time[%u]\n",
            _handle, s, getms() - t);
          if (ESRCH == s)
          {
            printf("[YDLIDAR] Thread [0x%X] has been canceled in other thread\n", _handle);
            return s;
          }
          if (s != 0)
          {
            fprintf(stderr, "[YDLIDAR] An error occurred while thread[0x%X] cancelled!\n", _handle);
            return s;
          }

          printf("[YDLIDAR] Thread [0x%X] has been canceled\n", _handle);
          _handle = 0;
#endif
          return 0;
        }
        //判断是否需要退出线程（限子线程内调用）
        static void needExit()
        {
#if defined(_WIN32)
#else
          pthread_testcancel();
#endif
        }

        bool operator==(const Thread &right)
        {
          return this->_handle == right._handle;
        }

      protected:
        explicit Thread(thread_proc_t proc, void *param) : _param(param), _func(proc),
                                                           _handle(0) {}
        void *_param;
        thread_proc_t _func;
        _size_t _handle;
      };

    } // base
  }  // core
} // ydlidar
