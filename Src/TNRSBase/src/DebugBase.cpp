/**
 * @file TNRSBase/src/DebugBase.cpp
 *
 * This file implements the class DebugBase.
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 09 Nov 2017  
 */

#include "TNRSBase/include/DebugBase.h"

map<string, DebugBase*> DebugBase::classMap = map<string, DebugBase*>();
pthread_mutex_t DebugBase::classMapAccessMutex = PTHREAD_MUTEX_INITIALIZER;

DebugBase::DebugBase(const string& name, DebugBase* ptr)
{
  updateClassMap(name, ptr);
}

void
DebugBase::updateClassMap(const string& name, DebugBase* ptr)
{
  pthread_mutex_lock(&classMapAccessMutex);
  classMap.insert(pair<string, DebugBase*>(name, ptr));
  pthread_mutex_unlock(&classMapAccessMutex);
}
