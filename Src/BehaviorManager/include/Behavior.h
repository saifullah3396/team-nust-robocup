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
#include "BehaviorManager/include/BehaviorConfig.h"
#include "BehaviorManager/include/BehaviorRequest.h"
#include "Utils/include/Exceptions/TNRSException.h"

class Behavior;
typedef boost::shared_ptr<Behavior> BehaviorPtr;

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
    runTime(0.f)
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
      initiate();
      initiated = true;
    } else {
      update();
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
   * Reinitiates the behavior with the given configuration
   *
   * @param cfg: The new behavior configuration
   */
  virtual void reinitiate(const BehaviorConfigPtr& cfg) {}

  /**
   * Runs the loadExternalConfig() function once for the behavior to
   * load static variables
   */
  void setupExternalConfig() {
    //! Whether the external configuration has been loaded once
    //! for the behavior
    static bool extCfgLoaded = false;
    if (!extCfgLoaded) {
      loadExternalConfig();
      extCfgLoaded = true;
    }
  }

  /**
   * Sets up a child behavior request
   *
   * @param config: The behavior config for requested child
   */
  void setupChildRequest(const BehaviorConfigPtr& config)
  {
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
   * Gets the behavior name
   *
   * @return string
   */
  string getName() { return name; }

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

  //! Configuration of the behavior
  BehaviorConfigPtr config;

  //! Child behavior
  BehaviorPtr child;

  //! Current child behavior request
  BehaviorRequestPtr childReq;

  //! Previous child request id
  unsigned lastChildReqId;

  //! Config of last child behavior accepted
  BehaviorConfigPtr lastChildCfg;
};

typedef boost::shared_ptr<Behavior> BehaviorPtr;
