/**
 * @file MemoryModule/Variable.cpp
 *
 * This file implements the class Variable.
 * 
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017  
 */

#include "../include/Variable.h"

template<class T>
  T
  Variable<T>::increment()
  {
  }

template<class T>
  T
  Variable<T>::decrement()
  {
  }

//!Integer Opertaions

template<>
  int
  Variable<int>::increment()
  {
    pthread_mutex_lock(&accessMutex);
    int temp = value++;
    pthread_mutex_unlock(&accessMutex);
    return temp;
  }

template<>
  int
  Variable<int>::decrement()
  {
    pthread_mutex_lock(&accessMutex);
    int temp = value--;
    pthread_mutex_unlock(&accessMutex);
    return temp;
  }

//!Float Opertaions

template<>
  float
  Variable<float>::increment()
  {
    pthread_mutex_lock(&accessMutex);
    float temp = value++;
    pthread_mutex_unlock(&accessMutex);
    return temp;
  }

template<>
  float
  Variable<float>::decrement()
  {
    pthread_mutex_lock(&accessMutex);
    float temp = value--;
    pthread_mutex_unlock(&accessMutex);
    return temp;
  }
