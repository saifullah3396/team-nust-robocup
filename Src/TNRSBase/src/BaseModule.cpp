/**
 * @file TNRSBase/src/BaseModule.cpp
 *
 * This file implements the class BaseModule and helper class
 * ThreadException
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017
 */

#include "TNRSBase/include/BaseModule.h"

vector<BaseModule*> BaseModule::runningModules;

ThreadException::ThreadException(const string& sMessage,
  const bool& blSysMsg) throw () :
  sMsg(sMessage)
{
  if (blSysMsg) {
    sMsg.append(": ");
  }
}

ThreadException::~ThreadException() throw ()
{
}

BaseModule::BaseModule(
  void* parent, 
  const unsigned& moduleId,
  const string& moduleName,
  const size_t& inRequestsMax) :
  MemoryBase(), parent(parent), moduleId(moduleId), moduleName(moduleName),
  inRequestsMax(inRequestsMax), moduleTime(0), parentSuspendFlag(false), suspendFlag(false)
{
  pthread_mutex_init(&suspendMutex, NULL);
  pthread_cond_init(&resumeCond, NULL);
  runningModules.push_back(this);
}

BaseModule::~BaseModule()
{
  pthread_cond_destroy(&resumeCond);
  pthread_mutex_destroy(&suspendMutex);
}

void
BaseModule::setupModule()
{
  initMemoryConn();
  PRINT("Initialized " + moduleName + " memory connection.")
  startUpInputSync();
  init();
  startUpOutputSync();
}

void
BaseModule::startModule() throw (ThreadException)
{
  createThread();
}

void
BaseModule::joinModule() throw (ThreadException)
{
  int rc = pthread_join(threadId, NULL);
  if (rc != 0) {
    throw ThreadException("Error in thread join... +(pthread_join())", true);
  }
}

void*
BaseModule::threadFunc(void* pTr)
{
  BaseModule* pThis = static_cast<BaseModule*>(pTr);
  while (true) {
    pThis->setIterationStartTime();
    pThis->handleRequests();
    pThis->mainRoutine();
    pThis->onIterationComplete();
  }
  pthread_exit(0);
  return NULL;
}

void
BaseModule::createThread() throw (ThreadException)
{
  int rc = pthread_create(&threadId, NULL, threadFunc, this);
  if (rc != 0) {
    throw ThreadException(
      "Error in thread creation... (pthread_create())",
      true);
  }
}

void
BaseModule::suspendMe(const bool& parentCommand)
{
  pthread_mutex_lock(&suspendMutex);
  if (parentCommand) {
    parentSuspendFlag = 1;
  } else {
    suspendFlag = 1;
  }
  pthread_mutex_unlock(&suspendMutex);
}

void
BaseModule::resumeMe(const bool& parentCommand)
{
  pthread_mutex_lock(&suspendMutex);
  if (parentCommand) {
    parentSuspendFlag = 0;
  } else {
    suspendFlag = 0;
  }
  pthread_cond_broadcast(&resumeCond);
  pthread_mutex_unlock(&suspendMutex);
}

void
BaseModule::wait(const bool& condition)
{
  pthread_mutex_lock(&condWaitMutex);
  while (!condition)
    pthread_cond_wait(&resumeCond, &condWaitMutex);
  pthread_mutex_unlock(&condWaitMutex);
}

void
BaseModule::checkSuspend()
{
  pthread_mutex_lock(&suspendMutex);
  while (suspendFlag != 0 || parentSuspendFlag != 0)
    pthread_cond_wait(&resumeCond, &suspendMutex);
  pthread_mutex_unlock(&suspendMutex);
}

void
BaseModule::startUpOutputSync()
{
  if (genericOutputConnector != 0) {
    PRINT("Syncing " + moduleName + " memory output.")
    genericOutputConnector->syncToMemory();
  }
  moduleTime = 0;
  checkSuspend();
}

void
BaseModule::startUpInputSync()
{
  if (genericInputConnector != 0) {
    PRINT("Syncing " + moduleName + " memory input.")
    genericInputConnector->syncFromMemory();
  }
  setThreadPeriod();
}

void
BaseModule::onIterationComplete()
{
  if (genericOutputConnector != 0) genericOutputConnector->syncToMemory();
  auto timeNow = high_resolution_clock::now();
  auto lastIterationTimeMS =
    duration_cast < milliseconds > (timeNow - iterationStartTime).count();
   //if (moduleName == "VisionModule")
   //  cout << moduleName << " took time " << lastIterationTimeMS << endl;
  //if (moduleName == "CommModule")
  //  cout << moduleName << lastIterationTimeMS << endl;
  if (lastIterationTimeMS < periodMinMS) {
    int waitTimeMS = periodMinMS - lastIterationTimeMS;
    usleep(waitTimeMS * 1000);
    /*if (moduleName == "MotionModule") {
     high_resolution_clock::time_point t2 = high_resolution_clock::now();
     cout << "time: " <<
     duration_cast<milliseconds>(t2 - iterationStartTime).count() << endl;
     }
     if (moduleName == "ControlModule") {
     high_resolution_clock::time_point t2 = high_resolution_clock::now();
     cout << "time: " <<
     duration_cast<milliseconds>(t2 - iterationStartTime).count() << endl;
     }*/
    moduleTime = moduleTime + periodMinMS * 1e-3;
  } else {
    moduleTime = moduleTime + lastIterationTimeMS * 1e-3;
  }
  checkSuspend();
  if (genericInputConnector != 0) genericInputConnector->syncFromMemory();
  setThreadPeriod();
}

void
BaseModule::getStringHeader(string& out)
{
  if (genericInputConnector != 0 && !genericInputConnector->getMUX().empty()) {
    size_t size = genericInputConnector->getMUX().size();
    size_t commaLimit = size - 1;
    DataUtils::getStringStartArray(out);
    out += DataUtils::varToString(size);
    out += ':';
    for (size_t j = 0; j < genericInputConnector->getMUX().size(); j++) {
      genericInputConnector->getMUX().at(j)->getStringHeader(out);
      if (j != commaLimit) out += ',';
    }
    DataUtils::getStringEndArray(out);
  } else {
    DataUtils::getStringStartArray(out);
    out += DataUtils::varToString(0);
    out += ':';
    DataUtils::getStringEndArray(out);
  }
  out += '&';
  if (genericOutputConnector != 0 && !genericOutputConnector->getSharedMemoryVariables().empty()) {
    size_t size = genericOutputConnector->getSharedMemoryVariables().size();
    size_t commaLimit = size - 1;
    DataUtils::getStringStartArray(out);
    out += DataUtils::varToString(size);
    out += ':';
    for (size_t j = 0;
      j < genericOutputConnector->getSharedMemoryVariables().size(); j++) {
      out +=
        genericOutputConnector->getSharedMemoryVariables().at(j)->getVariableName();
      if (j != commaLimit) out += ',';
    }
    DataUtils::getStringEndArray(out);
  } else {
    DataUtils::getStringStartArray(out);
    out += DataUtils::varToString(0);
    out += ':';
    DataUtils::getStringEndArray(out);
  }
}

void
BaseModule::getInputMuxSelectLineString(string & out)
{
  int size = genericInputConnector->getMUX().size();
  int commaLimit = size - 1;
  DataUtils::getStringStartArray(out);
  out += DataUtils::varToString(size);
  out += ':';

  for (size_t j = 0; j < genericInputConnector->getMUX().size(); j++) {
    if (genericInputConnector->getMUX().at(j)->getSelectLine() == 0) out += '0';
    else out += '1';
    if (j != commaLimit) out += ',';
  }
  DataUtils::getStringEndArray(out);
}
