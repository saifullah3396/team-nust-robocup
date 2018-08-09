/**
 * @file TNRSBase/include/BaseModule.h
 *
 * This file declares the class BaseModule and defines a helper class
 * ThreadException
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017
 */

#pragma once

#include "TNRSBase/include/MemoryBase.h"
#include "TNRSBase/include/MemoryConnector.h"
#include "TNRSBase/include/SharedMemory.h"
#include "TNRSBase/include/ModuleRequest.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/ThreadSafeQueue.h"

using namespace Utils;

/**
 * @class ThreadException
 * @brief Thread exception management class
 */
class ThreadException : public exception
{
public:
  /**
   * Constructs a ThreadException with an explanatory message
   *
   * @param message explanatory message
   * @param bSysMsg true if a system message (from strerror(errno))
   *   should be postfixed to the user provided message
   */
  ThreadException(const string& message, const bool& bSysMsg) throw ();

  /**
   * Virtual destructor
   */
  virtual
  ~ThreadException() throw ();

  /**
   * Returns a pointer to the (constant) error description
   *
   * @return a pointer to a const char
   */
  virtual const char*
  what() const throw ()
  {
    return sMsg.c_str();
  }

protected:
  //! Error message.
  string sMsg;
};

/**
 * @class BaseModule
 * @brief A class that defines basic threaded modules
 *
 * The class BaseModule is the base class for all the core threaded
 *   modules in our software. A class that inherits from BaseModule
 *   gains its own thread with its period defined in SharedMemory
 *   and also directly gains access to the SharedMemory on definition.
 *   Furthermore, only the classes inheriting from BaseModule have the
 *   ability to define which of the SharedMemory variables will be
 *   their input and which variables will be taken as an output from
 *   the given class. These input and output variables must be defined
 *   for every class inheriting from BaseModule, by using the macros
 *   defined in InoutConnectors.h.
 */
class BaseModule : public MemoryBase
{
public:
  /**
   * Constructs a threaded module with an id, name and pointer
   * to the parent class
   *
   * @param parent a pointer to the parent class. Type of the parent
   *   can differ in each case.
   * @param moduleId a unique module identity
   * @param moduleName a unique module name
   * @param inRequestsMax maximum possible number of requests at a time
   */
  BaseModule(
  void* parent, 
  const unsigned& moduleId, 
  const string& moduleName,
  const size_t& inRequestsMax = 20);

  /**
   * Virtual destructor
   */
  virtual
  ~BaseModule();

  /**
   * Sets up the module by making memory connections, syncing memory and
   * calling init() function
   *
   * @return void
   */
  void
  setupModule();

  /**
   * Starts the module thread
   *
   * @return void
   */
  void
  startModule() throw (ThreadException);

  /**
   * Joins the module thread
   *
   * @return void
   */
  void
  joinModule() throw (ThreadException);

  /**
   * Suspends the module thread
   *
   * @param parentCommand whether the suspend command is from the parent
   * @return void
   */
  void
  suspendMe(const bool& parentCommand = false);

  /**
   * Resumes the module thread
   *
   * @param parentCommand whether the resume command is from the parent
   * @return void
   */
  void
  resumeMe(const bool& parentCommand = false);

  /**
   * Suspends the module thread based on some condition
   *
   * @param condition condition to be true
   * @return void
   */
  void
  wait(const bool& condition);

  /**
   * Pushes a module request to the module request queue
   * 
   * @param request: The request object
   */ 
  void addRequest(const ModuleRequestPtr& request) {
    if (inRequests.getSize() >= inRequestsMax)
      return;
    inRequests.pushToQueue(request);
  }

  /**
   * Can be called from any BaseModule to publish a module request to
   * another BaseModule based on its id.
   * 
   * @param request: The request to be published
   * @param moduleId: Id of the module to which the request is to be 
   *   published
   */
  static void publishModuleRequest(const ModuleRequestPtr& request)
  {
    for (size_t i = 0; i < runningModules.size(); ++i) {
      if (request->getModuleId() == runningModules[i]->getModuleId()) {
        runningModules[i]->addRequest(request);
      }
    }
  }

  /**
   * Returns a base module that matches the given id
   * 
   * @param moduleId: Id of requested module
   *
   * @return BaseModule*
   */
  static BaseModule* getModule(const unsigned& moduleId)
  {
    for (size_t i = 0; i < runningModules.size(); ++i) {
      if (moduleId == runningModules[i]->getModuleId()) {
        return runningModules[i];
      }
    }
    return NULL;
  }

  /**
   * Returns the module name
   *
   * @return string
   */
  string
  getModuleName() const
  {
    return moduleName;
  }

  /**
   * Returns the module id
   *
   * @return unsigned
   */
  unsigned
  getModuleId() const
  {
    return moduleId;
  }

  /**
   * Returns the module parent
   *
   * @return void*
   */
  void*
  getParent() const
  {
    return parent;
  }

  /**
   * Sets the module time period in milliseconds
   * 
   * @param periodMinMS: The period
   */
  void
  setPeriodMinMS(const int& periodMinMS)
  {
    this->periodMinMS = periodMinMS;
  }

  /**
   * Returns the module time period in milliseconds
   *
   * @return int
   */
  int
  getPeriodMinMS() const
  {
    return periodMinMS;
  }

  /**
   * Returns the module run time in seconds
   *
   * @return float
   */
  float
  getModuleTime() const
  {
    return moduleTime;
  }

  /**
   * Sets the SharedMemory object
   *
   * @return void
   */
  void
  setLocalSharedMemory(const SharedMemoryPtr& sharedMemory)
  {
    this->sharedMemory = sharedMemory;
  }

  /**
   * Returns the pointer to SharedMemory object
   *
   * @return SharedMemoryPtr
   */
  SharedMemoryPtr
  getLocalSharedMemory() const
  {
    return sharedMemory;
  }

  //~thread related functions
  //FIXME: Need to define reason for this function
  void
  getInputMuxSelectLineString(string& out);

  /**
   * Generates the headers for the local memory variables of this module
   *
   * @param out the output string
   * @return void
   */
  virtual void
  getStringHeader(string& out);

protected:
  //! A queue for receiving requests
  ThreadSafeQueue<ModuleRequestPtr> inRequests;

private:
  /**
   * A pure virtual function that defines how the incoming module requests
   * are handled
   *
   * @return void
   */
  virtual void
  handleRequests() = 0;

  /**
   * A pure virtual function that defines the main processing loop of
   * the derived class
   *
   * @return void
   */
  virtual void
  mainRoutine() = 0;

  /**
   * A pure virtual function that defines the derived class access to
   * SharedMemory
   *
   * @return void
   */
  virtual void
  initMemoryConn() = 0;
  
  /**
   * Initializes the module and its variables
   *
   * @return void
   */
  virtual void
  init() = 0;

  /**
   * Sets the thread cycle time period
   *
   * @return void
   */
  virtual void
  setThreadPeriod() = 0;

  /**
   * Syncs the local memory variables of the module with the variables in
   * memory and completes the thread time period loop
   *
   * @return void
   */
  void onIterationComplete();

  /**
   * Checks whether the module should be suspended
   *
   * @return void
   */
  void
  checkSuspend();

  /**
   * Syncs the local input memory variables of the module with the
   * variables in memory
   *
   * @return void
   */
  void
  startUpInputSync();

  /**
   * Syncs the local output memory variables of the module with the
   * variables in memory
   *
   * @return void
   */
  void
  startUpOutputSync();

  /**
   * Sets the iteration start time for the given module
   *
   * @return void
   */
  void
  setIterationStartTime()
  {
    iterationStartTime = high_resolution_clock::now();
  }
  
  /**
   * Creates the module thread
   *
   * @return void
   */
  void
  createThread() throw (ThreadException);

  /**
   * Pthread callback wrapper function for initializing the thread
   *
   * @param void* pTr a pointer to self
   * @return void pointer
   */
  static void*
  threadFunc(void* pTr);

  //! A static vector of all the created base modules
  static vector<BaseModule*> runningModules;

  //! Time at the start of iteration
  high_resolution_clock::time_point iterationStartTime;

  //! Time period of the module in milliseconds
  int periodMinMS;

  //! Pointer to parent
  void* parent;

  //! Module identity
  unsigned moduleId;

  //! Module name
  string moduleName;

  //! Module time
  double moduleTime;

  //! Max requests possible in the queue
  size_t inRequestsMax;

  //! Pointer to local data memory object
  SharedMemoryPtr sharedMemory;
  
  //! Module suspension ordered by parent
  bool parentSuspendFlag;

  //! Module suspension ordered by itself
  bool suspendFlag;

  //! Module thread mutex
  pthread_mutex_t suspendMutex;

  //! Module thread condition variable
  pthread_cond_t resumeCond;

  //! Mutex conditional wait
  pthread_mutex_t condWaitMutex;

  //! Module thread id defined by pthread_t
  pthread_t threadId;
};

typedef boost::shared_ptr<BaseModule> BaseModulePtr;
