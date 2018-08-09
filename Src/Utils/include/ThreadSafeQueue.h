/**
 * @file Utils/include/ThreadSafeQueue.h
 *
 * This file declares and implements a threadsafe queue.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017  
 */
#pragma once

#include <queue>
#include <pthread.h>
#include <string>

using namespace std;

template<typename T>
  class ThreadSafeQueue
  {
  public:
    /**
     * Default construtor of this class.
     * Queue for maintaining the variable T.
     */
    ThreadSafeQueue()
    {
      pthread_mutex_init(&AccessMutex, NULL);
    }

    /**
     * Default destrutor of this class.
     */
    ~ThreadSafeQueue()
    {
      pthread_mutex_destroy(&AccessMutex);
    }

  public:
    /**
     * Pushes an element to the queue.
     * 
     * @param in: Input variable.
     */
    void
    pushToQueue(T in)
    {
      pthread_mutex_lock(&AccessMutex);
      tQueue.push(in);
      pthread_mutex_unlock(&AccessMutex);
    }

    /**
     * Gets T from the queue front.
     * 
     * @return T: the output variable.
     */
    T
    queueFront()
    {
      pthread_mutex_lock(&AccessMutex);
      T temp = tQueue.front();
      pthread_mutex_unlock(&AccessMutex);
      return temp;
    }

    /**
     * Pops the element from queue front.
     */
    void
    popQueue()
    {
      pthread_mutex_lock(&AccessMutex);
      tQueue.pop();
      pthread_mutex_unlock(&AccessMutex);
    }

    /**
     * Checks whether the queue is empty.
     * 
     * @return boolean.
     */
    bool
    isEmpty()
    {
      pthread_mutex_lock(&AccessMutex);
      bool temp = tQueue.empty();
      pthread_mutex_unlock(&AccessMutex);
      return temp;
    }

    /**
     * Checks whether the queue is empty.
     * 
     * @return int.
     */
    int
    getSize()
    {
      pthread_mutex_lock(&AccessMutex);
      int temp = tQueue.size();
      pthread_mutex_unlock(&AccessMutex);
      return temp;
    }

    /**
     * Mutex for threadsafe access to the queue.
     * 
     * @var pthread_mutex_t
     */
    pthread_mutex_t AccessMutex;

    /**
     * Base queue for the actuator requests.
     * 
     * @var queue.
     */
    queue<T> tQueue;
  };

template<typename T>
  class RequestQueue : public ThreadSafeQueue<T>
  {
    using ThreadSafeQueue<T>::pushToQueue;
    using ThreadSafeQueue<T>::queueFront;
    using ThreadSafeQueue<T>::popQueue;
    using ThreadSafeQueue<T>::isEmpty;
  public:
    /**
     * Default construtor of this class.
     * Queue for maintaining actuator requests.
     */
    RequestQueue() :
      ThreadSafeQueue<T>()
    {
    }

    void
    pushRequest(T req)
    {
      ThreadSafeQueue<T>::pushToQueue(req);
    }

    T
    getRequest()
    {
      T request;
      this->queueFront(request);
      return request;
    }

    void
    popRequest()
    {
      this->popQueue();
    }

    bool
    noRequests()
    {
      bool isEmpty;
      this->isEmpty(isEmpty);
      return isEmpty;
    }

    int
    queueSize()
    {
      int size;
      this->getSize(size);
      return size;
    }
  };
