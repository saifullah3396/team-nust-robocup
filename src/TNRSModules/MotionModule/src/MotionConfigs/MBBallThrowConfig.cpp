/**
 * @file MotionModule/src/MotionConfigs/MBBallThrowConfig.cpp
 *
 * This file implements the struct MBBallThrowConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "MotionModule/include/MotionConfigs/MBBallThrowConfig.h"

MBBallThrowConfig::MBBallThrowConfig(
  const MBBallThrowTypes& type,
  const float& timeToThrow,
  const bool& headTapToStart) :
  MBConfig(MBIds::BALL_THROW, 45.f, (int)type),
  timeToThrow(timeToThrow),
  headTapToStart(headTapToStart)
{
}

void MBBallThrowConfig::validate() throw (BConfigException) {
  if (timeToThrow <= 0.f) 
  { 
    throw 
      BConfigException(
        this, 
        "Invalid behavior configuration parameters passed.", 
        false, 
        EXC_INVALID_BCONFIG_PARAMETERS
      );
  }
}

bool MBBallThrowConfig::assignFromJson(const Json::Value& obj)
{
  if (!BehaviorConfig::assignFromJson(obj))
    return false;
  try {
    timeToThrow = obj.get("timeToThrow", -1.f).asFloat(); // An invalid default value -1
    headTapToStart = obj.get("headTapToStart", false).asBool();
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

Json::Value MBBallThrowConfig::getJson() 
{ 
  Json::Value obj = BehaviorConfig::getJson();
  try {
    obj["name"] = "MBBallThrow";
    obj["timeToThrow"] = timeToThrow;
    obj["headTapToStart"] = headTapToStart;
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

boost::shared_ptr<MBBallThrowConfig> 
  MBBallThrowConfig::makeFromJson(const Json::Value& obj)
{
  boost::shared_ptr<MBBallThrowConfig> config;
  try {
    if (!obj.isNull() &&
        obj["id"].asUInt() == (unsigned)MBIds::BALL_THROW &&
        obj["baseType"].asUInt() == (unsigned) BaseBehaviorType::MOTION &&
        obj["type"].asUInt() <= (unsigned)MBBallThrowTypes::COUNT
    ) {
      switch (obj["type"].asUInt()) {
        case (unsigned)MBBallThrowTypes::WB_BALL_THROW:
          config = 
            boost::make_shared<MBBallThrowConfig>(); break;
        
      }
      if (config->assignFromJson(obj)) {
        throw TNRSException(
          "Error creating a configuration from given Json object");
      }
    }
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught making a ball throw behavior config from json object;\t";
    cout << e.what();
    config.reset();
  } catch (TNRSException& e) {
    cout << e.what();
    config.reset();
  }
}
