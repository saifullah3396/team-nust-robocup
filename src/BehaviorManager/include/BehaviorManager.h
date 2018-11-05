/**
 * @file BehaviorManager/include/BehaviorManager.h
 *
 * This file declares the classes BehaviorManager and BManagerException
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "BehaviorManager/include/BehaviorInfo.h"
#include "Utils/include/Exceptions/TNRSException.h"

//! Forward declaration
class BehaviorManager;
class Behavior;
typedef boost::shared_ptr<Behavior> BehaviorPtr;
class BehaviorRequest;
typedef boost::shared_ptr<BehaviorRequest> BehaviorRequestPtr;
class BehaviorConfig;
typedef boost::shared_ptr<BehaviorConfig> BehaviorConfigPtr;

/**
 * Enumeration for possible types of behavior manager exceptions
 *
 * @enum BManagerExceptionType
 */
DEFINE_ENUM_WITH_STRING_CONVERSIONS(
  BManagerExceptionType,
  (EXC_BEHAVIOR_SETUP_FAILED)
)

/**
 * @class BManagerException
 * @brief BehaviorManager exception management class
 */
class BManagerException : public TNRSException
{
public:
  /**
   * Constructor
   *
   * @param behaviorManager: In which the exception is raised
   * @param message: Explanatory message
   * @param bSysMsg: True if the system message (from strerror(errno))
   *   should be postfixed to the user provided message
   * @param type: Argument parser exception type
   */
  BManagerException(
    BehaviorManager* behaviorManager,
    const string& message,
    const bool& bSysMsg,
    const BManagerExceptionType& type) throw ();

  /**
   * Destructor
   */
  ~BManagerException() throw () {}

  string getExcPrefix()
  { return "Exception caught in behavior manager \n\t" + name + ";\t"; }

private:
  string name;
  BManagerExceptionType type;
};

/**
 * @class BehaviorManager
 * @brief The base class for managing behaviors
 */
class BehaviorManager
{
public:
  /**
   * Constructor
   */
  BehaviorManager(const string& name) :
    name(name), killRequested(false)
  {
    behaviorInfo = boost::make_shared<BehaviorInfo>();
  }

  /**
   * Destructor
   */
  virtual ~BehaviorManager()
  {
  }

  /**
   * Updates the behavior manager
   */
  void update();

  /**
   * Manages a behavior request and updates the currBehavior with it
   *
   * @param req: The behavior request to be managed
   */
  void manageRequest(
    const BehaviorRequestPtr& req);
    
  /**
   * Manages a behavior request and updates the given behavior with it
   *
   * @param bPtr: The behavior to be updated
   * @param req: The behavior request to be managed
   */
  void manageRequest(
    BehaviorPtr& bPtr,
    const BehaviorRequestPtr& req);

  /**
   * Gets the behavior name
   *
   * @return string
   */
  string getName() { return name; }

  /**
   * Sets the current behavior info from the given input behavior
   * 
   * @param bPtr: The behavior whose info is to be shown
   */ 
  void setBehaviorInfo(const BehaviorPtr& bPtr);

  /**
   * Gets the info of the current running behavior
   *
   * @return BehaviorInfo
   */
  BehaviorInfo getBehaviorInfo() const { return *behaviorInfo; }

  /**
   * Sets killing the behavior in progress
   */ 
  void killBehavior() { killRequested = true; }
protected:
  /**
   * Makes a new behavior and assigns it to the input referenced
   * behavior
   *
   * @param bPtr: Behavior to be assigned a value
   * @param cfg: Configuration with which a new behavior is constructed
   *
   * @return Whether a behavior is constructed and assigned to bPtr
   */
  virtual bool makeBehavior(
    BehaviorPtr& bPtr, const BehaviorConfigPtr& cfg) = 0;

private:
  /**
   * Sets up a behavior based on the given behavior configuration
   *
   * @param bPtr: Behavior to be set up
   * @param cfg: Configuration with which the behavior is set up
   *
   * @return Whether the behavior is set up successfully
   */
  bool setupBehavior(
    BehaviorPtr& behavior,
    const BehaviorConfigPtr& cfg
  ) throw (BManagerException);

  /**
   * Updates the child of the behavior if present, otherwise the
   * behavior itself
   *
   * @param bPtr: Behavior which or whose child is to be updated
   */
  void updateBehavior(BehaviorPtr& bPtr);

  //! Name of this behavior manager
  string name;

  //! If a kill request is received
  bool killRequested;

  //! Pointer to the current running behavior
  BehaviorPtr currBehavior;
  
  //! Information about the current running behavior
  BehaviorInfoPtr behaviorInfo;
};
