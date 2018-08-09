/**
 * @file Utils/ThreadSafeVariable.cpp
 *
 * * This file implements the class ThreadSafeVariable.
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017  
 */

#include "../include/ThreadSafeVariable.h"

ThreadSafeVariable::ThreadSafeVariable(string variableName)
{
  this->variableName = variableName;
  initVariable();
}

ThreadSafeVariable::~ThreadSafeVariable()
{
  pthread_mutex_destroy(&accessMutex);
  if (conditionInitialised) {
    pthread_cond_destroy(&waitCond);
  }
}
