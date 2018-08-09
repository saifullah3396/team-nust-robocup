/**
 * @file Utils/include/BehaviorConfig.h
 *
 * This file declares the base struct for the configuration of all the
 * behaviors.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 10 Sep 2017
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include "Utils/include/Exceptions/TNRSException.h"
#include "Utils/include/DataUtils.h"

using namespace Utils;

/**
 * Enumeration for all types of behavior base types
 *
 * @enum BaseBehaviorType
 */
DEFINE_ENUM_WITH_STRING_CONVERSIONS(
	BaseBehaviorType, (MOTION)(STATIC)(PLANNING)
)

/**
 * @struct BehaviorConfig
 * @brief The base struct for a behavior configuration
 */
struct BehaviorConfig
{
  BehaviorConfig(
    const unsigned& id, 
    const BaseBehaviorType& baseType,
    const float& maxRuntime,
    const int& type = -1) :
    id(id), baseType(baseType), maxRuntime(maxRuntime), type(type)
  {
  }
  
  virtual
  ~BehaviorConfig()
  {
  }

  virtual bool isValid() { return false; }
  unsigned id;
  int type;
  BaseBehaviorType baseType;
  float maxRuntime;
};

typedef boost::shared_ptr<BehaviorConfig> BehaviorConfigPtr;

DEFINE_ENUM_WITH_STRING_CONVERSIONS(
	BConfigExceptionType, 
	(EXC_INVALID_BCONFIG_PARAMETERS)
)

/**
 * @class BConfigException
 * @brief BehaviorConfig exception management class
 */
class BConfigException : public TNRSException
{
public:
  /**
   * Constructor
   * 
   * @param behaviorConfig in which the exception is raised
   * @param message explanatory message
   * @param bSysMsg true if the system message (from strerror(errno))
   *   should be postfixed to the user provided message
   * @param type Argument parser exception type
   */
  BConfigException(
    BehaviorConfig* config, 
		const string& message,
		const bool& bSysMsg, 
		const BConfigExceptionType& type) throw () :
    TNRSException(message, bSysMsg),
    config(config),
    type(type)
  {
  }

  /**
   * Destructor
   */
  ~BConfigException() throw () {}
  
  string getExcPrefix()
		{ 
      return 
        string("Exception caught in behavior config \n\tType: ") + 
        EnumToString(config->baseType) + 
        string(", Id: ") + 
        DataUtils::varToString(config->id) + 
        string(";\t"); 
    }

private:
  BehaviorConfig* config;
	BConfigExceptionType type;
};
