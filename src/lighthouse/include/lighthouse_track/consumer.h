#pragma once
#include "lighthouse_track/ringbuffer.h"
#include <unistd.h>
#include <thread>
#include <iostream>

template<typename T>
class Consumer {
  public:
    Consumer(RingBuffer<T>* rb) :
      ringbuffer_(rb) {
        shall_stop_ = true;
      }
    virtual ~Consumer() {
      if (ringbuffer_) delete ringbuffer_;
    }

    virtual bool Consume() = 0;//虚成员函数=0;表示本类不对这个成员函数进行实现，即该成员函数没有函数体{ }，这种函数就叫纯虚函数。一个类中只要有一个成员函数是纯虚函数，就成了不能实例化的虚类，相当于接口，只能给派生类继承，让派生类来进行函数覆盖重写这个虚方法。

    virtual void Start() {
      shall_stop_ = false;
      while(!shall_stop_) {
        Consume();//由继承类track_object定义
        usleep(1);
      };
      std::cout << "stoping consume" << std::endl;
    }

    inline virtual void Stop() {
      shall_stop_ = true;
    }

    RingBuffer<T>* ringbuffer_;
  private:
    bool shall_stop_;
};

