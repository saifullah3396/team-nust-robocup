/**
 * @file BehaviorManager/include/Behavior.h
 *
 * This file declares the classes Behavior and BehaviorException
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 10 Sep 2017
 */

#pragma once
#include <boost/make_shared.hpp>
#include <boost/filesystem.hpp>
#include "BehaviorManager/include/BehaviorConfig.h"
#include "BehaviorManager/include/BehaviorRequest.h"
#include "Utils/include/Exceptions/TNRSException.h"
#include "Utils/include/ConfigManager.h"
#include <jsoncpp/json/json.h>

namespace Utils{
  class JsonLogger;
}
class Behavior;
typedef boost::shared_ptr<Behavior> BehaviorPtr;
typedef boost::shared_ptr<Utils::JsonLogger> JsonLoggerPtr;

/**
 * Enumeration for possible types of behavior exceptions
 *
 * @enum BehaviorExceptionType
 */
DEFINE_ENUM_WITH_STRING_CONVERSIONS(
  BehaviorExceptionType,
  (EXC_INVALID_BEHAVIOR)
  (EXC_INVALID_BEHAVIOR_SETUP)
  (EXC_CHILD_ID_MISMATCH)
)

/**
 * @class BehaviorException
 * @brief Behavior exception management class
 */
class BehaviorException : public TNRSException
{
public:
  /**
   * Constructor
   *
   * @param behavior: In which the exception is raised
   * @param message: Explanatory message
   * @param bSysMsg: True if the system message (from strerror(errno))
   *   should be postfixed to the user provided message
   * @param type: Argument parser exception type
   */
  BehaviorException(
    Behavior* behavior,
    const string& message,
    const bool& bSysMsg,
    const BehaviorExceptionType& type
  ) throw ();

  /**
   * Destructor
   */
  ~BehaviorException() throw () {}

  string getExcPrefix()
    { return "Exception caught in behavior \n\t" + name + ";\t"; }

private:
  string name;
  BehaviorExceptionType type;
};

/**
 * @class Behavior
 * @brief The base class for all types of behaviors
 */
class Behavior
{
public:
  /**
   * Constructor
   *
   * @param name: Name of this behavior
   */
  Behavior(
    const BehaviorConfigPtr& config,
    const string& name = "Not assigned.") :
    name(name),
    config(config),
    initiated(false),
    inBehavior(false),
    lastChildReqId(0),
    runTime(0.f),
    childInParallel(false),
    logData(false)
  {
  }

  /**
   * Destructor
   */
  virtual ~Behavior()
  {
  }

  /**
   * Manages the initiation and update of this behavior
   */
  void manage()
  {
    if (!initiated) {
      // Setup a data logger if it is needed and not already initialized
      if (logData && !dataLogger) {
        // Setup log dirs 
        if (setupLogsDir())
          dataLogger = makeLogger();
        else
          logData = false;
      }
      initiate();
      initiated = true;
    } else {
      update();
      if (logData) {
        updateDataLogger();
      }
      //runTime = runTime + cycleTime;
    }
  }

  /**
   * Initiates the behavior. Must be defined in derived class.
   */
  virtual void initiate() = 0;

  /**
   * Updates the behavior. Must be defined in derived class.
   */
  virtual void update() = 0;

  /**
   * Performs behavior cleanup operations
   */
  virtual void finish() = 0;

  /**
   * This function is meant to load the static variables of this
   * behavior
   */
  virtual void loadExternalConfig() = 0;

  /**
   * Creates a json logger
   */ 
  virtual JsonLoggerPtr makeLogger();
  
  /**
   * Can be defined in the child classes to update logger in each 
   * update run as desired
   */ 
  virtual void updateDataLogger() {}

  /**
   * Reinitiates the behavior with the given configuration
   *
   * @param cfg: The new behavior configuration
   */
  virtual void reinitiate(const BehaviorConfigPtr& cfg) {}

  /**
   * Sets up a child behavior request
   *
   * @param config: The behavior config for requested child
   */
  void setupChildRequest(
    const BehaviorConfigPtr& config, 
    const bool& runInParallel = false)
  {
    this->childInParallel = runInParallel;
    childReq = boost::make_shared<BehaviorRequest>(config);
  }

  /**
   * Finishes the behavior if it reaches its maximum time limit
   */
  void checkBehaviorTimeLimit()
  {
    if (runTime > config->maxRuntime)
      kill();
  }

  /**
   * Kills the behavior with last child killed first
   */
  void kill()
  {
    //! Kill all the behaviors recursively from child to parent
    if (child)
      child->kill();
    finish();
  }
  
  /**
   * Kills the child
   */
  void killChild()
  {
    if (child)
      child->kill();
  }

  /**
   * Returns true if the behavior has been initiated
   *
   * @return bool
   */
  const bool& isInitiated() { return initiated; }

  /**
   * Returns true if the behavior is in running state
   *
   * @return bool
   */
  const bool& isRunning() const { return inBehavior; }
  
  /**
   * Gets the configuration of this behavior
   *
   * @return BehaviorConfigPtr
   */
  BehaviorConfigPtr getBehaviorConfig() { return config; }

  /**
   * Gets the referenced pointer to child. The reference is sent as 
   * this pointer is also updated if a new child request is found.
   *
   * @return BehaviorPtr
   */
  BehaviorPtr& getChild() { return child; }

  /**
   * Gets the child request pointer
   *
   * @return BehaviorRequestPtr
   */
  BehaviorRequestPtr& getChildRequest() { return childReq; }

  /**
   * Gets the last child request id handle
   */  
  unsigned& getLastChildReqId() { return lastChildReqId; }

  /**
   * Sets the last child configuration
   * 
   * @param cfg: The configuration of the last child accepted
   */ 
  void setLastChildCfg(const BehaviorConfigPtr& cfg) {
    lastChildCfg = cfg;
  }

  /**
   * Returns true if child should run in parallel
   * 
   * @return bool
   */ 
  bool setChildInParallel(const bool& childInParallel)
   { this->childInParallel = childInParallel; }

  /**
   * Returns true if child should run in parallel
   * 
   * @return bool
   */ 
  bool getChildInParallel() { return childInParallel; }

  /**
   * Gets the behavior name
   *
   * @return string
   */
  string getName() { return name; }
  
  /**
   * Gets the data logger
   *
   * @return JsonLoggerPtr
   */
  JsonLoggerPtr getDataLogger() { return dataLogger; }

  /**
   * Sets the parent of this behavior
   * 
   * @param parent: Parent
   */ 
  void setParent(const BehaviorPtr& parent)
   { this->parent = parent; }
   
  /**
   * Sets the data logger of this behavior from parent if it exists
   */ 
  void setLoggerFromParent()
   { this->dataLogger = parent->getDataLogger(); logData = true; }

  /**
   * Creates a directory for storing logs. Returns false if unsucessful
   * 
   * @return bool
   */
  bool setupLogsDir()
  {
    if (logData) {
      logsDirPath = ConfigManager::getLogsDirPath() + name + "/";
      if (!boost::filesystem::exists(logsDirPath)) {
        if(!boost::filesystem::create_directory(logsDirPath))
          return false;
      }
      
      int num = 0;
      using namespace boost::filesystem;
      for(directory_iterator it(logsDirPath); 
          it != directory_iterator(); ++it)
      {
         ++num;
      }
      logsDirPath += "log";
      logsDirPath += DataUtils::varToString(num);
      boost::filesystem::path dir(logsDirPath);
      if(!boost::filesystem::create_directory(dir))
        return false;
    }
  }

  //! Maximum time a behavior can have to perform setup
  static constexpr float maxBehaviorSetupTime = 6.f;

protected:
  //! Name of the behavior
  string name;

  //! Total behavior runtime
  float runTime;

  //! Whether behavior has been initiated
  bool initiated;

  //! Whether the behavior is running
  bool inBehavior;
  
  //! Whether the child should run in parallel or not
  bool childInParallel;

  //! Configuration of the behavior
  BehaviorConfigPtr config;

  //! Parent of this behavior
  BehaviorPtr parent;

  //! Child of this behavior
  BehaviorPtr child;

  //! Current child behavior request
  BehaviorRequestPtr childReq;

  //! Previous child request id
  unsigned lastChildReqId;

  //! Config of last child behavior accepted
  BehaviorConfigPtr lastChildCfg;
  
  //! Log directory path
  string logsDirPath;
  
  //! Whether to log data regarding the running behavior
  bool logData;

  //! Data logger for this behavior
  JsonLoggerPtr dataLogger;
};

typedef boost::shared_ptr<Behavior> BehaviorPtr;
