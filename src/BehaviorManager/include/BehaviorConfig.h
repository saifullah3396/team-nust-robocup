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
#include <jsoncpp/json/json.h>
#include "Utils/include/Exceptions/TNRSException.h"
#include "Utils/include/DataUtils.h"

using namespace Utils;

DEFINE_ENUM_WITH_STRING_CONVERSIONS(
	BConfigExceptionType, 
	(EXC_INVALID_BCONFIG_PARAMETERS)
  (EXC_INVALID_BCONFIG)
  (INVALID_JSON_INPUT)
)

class BehaviorConfig;

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
		const BConfigExceptionType& type) throw ();

  /**
   * Destructor
   */
  ~BConfigException() throw () {}
  
  string getExcPrefix();

private:
  BehaviorConfig* config;
	BConfigExceptionType type;
};

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
 * @brief A base struct for all types of behavior configurations
 */
struct BehaviorConfig
{
  /**
   * Constructor
   * 
   * @param id: Behavior id
   * @param baseType: Behavior baseType -> MOTION, STATIC, PLANNING
   * @param maxRuntime: Maximum runtime for behavior after which it should
   *   be killed
   * @param type: Sub type of the behavior
   */ 
  BehaviorConfig(
    const unsigned& id, 
    const BaseBehaviorType& baseType,
    const float& maxRuntime,
    const int& type = -1) :
    id(id), baseType(baseType), maxRuntime(maxRuntime), type(type)
  {
  }
  
  /**
   * Destructor
   */ 
  virtual
  ~BehaviorConfig()
  {
  }

  /**
   * Throws an exception if the configuration is invalid
   * 
   * @return void
   */ 
  virtual void validate() throw (BConfigException) {
    throw
      BConfigException(
        this,
        "Requested behavior configuration is invalid or undefined for use.",
        false,
        EXC_INVALID_BCONFIG
      );
  }
  
  /**
   * Assigns the configuration parameters using json object
   * 
   * @param obj: Input json object with info regarding the configuration
   * 
   * @return false if an exception is raised
   */
  virtual bool assignFromJson(const Json::Value& obj)
  {
    try {
      // Match cfg parameters with that from the requested cfg in 
      // json format
      if (
        obj.isNull() ||
        id != obj["id"].asUInt() ||
        type != obj["type"].asInt() || 
        baseType != (BaseBehaviorType) obj["baseType"].asUInt()
      ) {
        throw Json::Exception(
          "Invalid json object specified for given behavior config.");
      }
      // Reset max runtime from json cfg
      maxRuntime = obj.get("maxRuntime", maxRuntime).asFloat();
    } catch (Json::Exception& e) {
      cout 
        << "Exception caught in behavior config \n\tType: "
        << EnumToString(baseType)
        << ", Id: " 
        << DataUtils::varToString(id) 
        << ";\t";
      cout << e.what();
      return false;
    }
    return true;
  }
  
  /**
   * Makes a json object from the configuration parameters
   * 
   * @return Json::Value
   */
  virtual Json::Value getJson() { 
    Json::Value obj;
    try {
      obj["id"] = id;
      obj["type"] = type;
      obj["baseType"] = (unsigned)baseType;
      obj["maxRuntime"] = maxRuntime;
    } catch (Json::Exception& e) {
      cout 
        << "Exception caught in behavior config \n\tType: "
        << EnumToString(baseType)
        << ", Id: " 
        << DataUtils::varToString(id) 
        << ";\t";
      cout << e.what();
    }
    return obj;
  }
  
  //! Behavior id as defined for the given base type
  unsigned id;
  
  //! Behavior sub type
  int type;
  
  //! Behavior base type
  BaseBehaviorType baseType;
  
  //! Maximum run time of this behavior
  float maxRuntime;
};

typedef boost::shared_ptr<BehaviorConfig> BehaviorConfigPtr;
