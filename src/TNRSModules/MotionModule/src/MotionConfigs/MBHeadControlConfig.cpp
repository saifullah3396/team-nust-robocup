/**
 * @file MotionModule/src/MotionConfigs/MBHeadControlConfig.cpp
 *
 * This file implements the structs MBHeadControlConfig, 
 * HeadTargetSearchConfig and HeadTargetTrackConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "MotionModule/include/MotionConfigs/MBHeadControlConfig.h"
#include "MotionModule/include/HeadControl/HeadTargetTypes.h"


MBHeadControlConfig::MBHeadControlConfig(
  const MBHeadControlTypes& type, 
  const float& maxRunTime) :
  MBConfig(MBIds::HEAD_CONTROL, maxRunTime, (int)type)
{
}

bool MBHeadControlConfig::assignFromJson(const Json::Value& obj)
{
  if (!BehaviorConfig::assignFromJson(obj))
    return false;
  return true;
}

Json::Value MBHeadControlConfig::getJson() { 
  Json::Value obj = BehaviorConfig::getJson();
  return obj;
}

MBHeadControlConfigPtr MBHeadControlConfig::makeFromJson(const Json::Value& obj)
{
  boost::shared_ptr<MBHeadControlConfig> config;
  try {
    if (!obj.isNull() &&
        obj["id"].asUInt() == (unsigned)MBIds::HEAD_CONTROL &&
        obj["baseType"].asUInt() == (unsigned) BaseBehaviorType::MOTION &&
        obj["type"].asUInt() <= (unsigned)MBHeadControlTypes::COUNT
    ) {
      switch (obj["type"].asUInt()) {
        case (unsigned)MBHeadControlTypes::HEAD_TARGET_SEARCH:
          config = 
            boost::make_shared<HeadTargetSearchConfig>(); break;
        case (unsigned)MBHeadControlTypes::HEAD_TARGET_TRACK:
          config = 
            boost::make_shared<HeadTargetTrackConfig>(); break;
      }
      if (config->assignFromJson(obj)) {
        throw TNRSException(
          "Error creating a configuration from given Json object");
      }
    }
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught making a head control behavior config from json object;\t";
    cout << e.what();
    config.reset();
  } catch (TNRSException& e) {
    cout << e.what();
    config.reset();
  }
}
  
HeadTargetSearchConfig::HeadTargetSearchConfig(
  const HeadTargetTypes& headTargetType,
  const bool& scanLowerArea,
  const float& totalWaitTime,
  const float& scanMaxYaw,
  const float& scanMaxPitch) :
  MBHeadControlConfig(MBHeadControlTypes::HEAD_TARGET_SEARCH),
  headTargetType(headTargetType),
  totalWaitTime(totalWaitTime),
  scanMaxYaw(scanMaxYaw),
  scanMaxPitch(scanMaxPitch) 
{
}

void HeadTargetSearchConfig::validate() throw (BConfigException) 
{
}


bool HeadTargetSearchConfig::assignFromJson(const Json::Value& obj)
{
  MBHeadControlConfig::assignFromJson(obj);
  try {
    scanLowerArea = obj.get("scanLowerArea", false).asBool();
    scanMaxYaw = obj.get("scanMaxYaw", 1.f).asFloat();
    scanMaxPitch = obj.get("scanMaxPitch", 100.f * M_PI / 180).asFloat();
    totalWaitTime = obj.get("totalWaitTime", 16.f * M_PI / 180).asFloat();
    headTargetType = (HeadTargetTypes) obj.get("headTargetType", -1).asUInt();
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

Json::Value HeadTargetSearchConfig::getJson() 
{
  Json::Value obj = MBHeadControlConfig::getJson();
  try {
    obj["name"] = "HeadTargetSearch";
    obj["scanLowerArea"] = scanLowerArea;
    obj["scanMaxYaw"] = scanMaxYaw;
    obj["scanMaxPitch"] = scanMaxPitch;
    obj["totalWaitTime"] = totalWaitTime;
    obj["headTargetType"] = (unsigned) headTargetType;
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

HeadTargetTrackConfig::HeadTargetTrackConfig(
  const HeadTargetTypes& headTargetType,
  const float& maxRunTime) : // Basically stay non stop in track
  MBHeadControlConfig(
    MBHeadControlTypes::HEAD_TARGET_TRACK, maxRunTime), 
  headTargetType(headTargetType)
{
}

void HeadTargetTrackConfig::validate() throw (BConfigException) 
{
}

bool HeadTargetTrackConfig::assignFromJson(const Json::Value& obj)
{
  MBHeadControlConfig::assignFromJson(obj);
  try {
    headTargetType = 
      (HeadTargetTypes) obj.get("headTargetType", -1).asUInt();
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught in behavior config \n\tType: "
      << EnumToString(baseType)
      << ", Id: " 
      << DataUtils::varToString(id) 
      << ";\t";
    cout << e.what();
  }
}

Json::Value HeadTargetTrackConfig::getJson() 
{
  Json::Value obj = MBHeadControlConfig::getJson();
  try {
    obj["name"] = "HeadTargetTrack";
    obj["headTargetType"] = (unsigned) headTargetType;
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


