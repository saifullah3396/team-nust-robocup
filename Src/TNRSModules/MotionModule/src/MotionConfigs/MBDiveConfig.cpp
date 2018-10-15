/**
 * @file MotionModule/src/MotionConfigs/MBDiveConfig.cpp
 *
 * This file implements the structs MBDiveConfig and KFMDiveConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "MotionModule/include/MotionConfigs/MBDiveConfig.h"
#include "MotionModule/include/DiveModule/KeyFrameDiveTypes.h"

MBDiveConfig::MBDiveConfig(const MBDiveTypes& type) :
  MBConfig(MBIds::DIVE, 5.f, (int)type)
{
}

bool MBDiveConfig::assignFromJson(const Json::Value& obj)
{
  if (!BehaviorConfig::assignFromJson(obj))
    return false;
  return true;
}

Json::Value MBDiveConfig::getJson() { 
  Json::Value obj = BehaviorConfig::getJson();
  return obj;
}

MBDiveConfigPtr MBDiveConfig::makeFromJson(const Json::Value& obj)
{
  boost::shared_ptr<MBDiveConfig> config;
  try {
    if (!obj.isNull() &&
        obj["id"].asUInt() == (unsigned)MBIds::DIVE &&
        obj["baseType"].asUInt() == (unsigned) BaseBehaviorType::MOTION &&
        obj["type"].asUInt() <= (unsigned)MBDiveTypes::COUNT
    ) {
      switch (obj["type"].asUInt()) {
        case (unsigned)MBDiveTypes::KEY_FRAME_MOTION_DIVE:
          config = 
            boost::make_shared<KFMDiveConfig>(); break;
        
      }
            if (config->assignFromJson(obj)) {
        throw TNRSException(
          "Error creating a configuration from given Json object");
      }
    }
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught making a dive behavior config from json object;\t";
    cout << e.what();
    config.reset();
  } catch (TNRSException& e) {
    cout << e.what();
    config.reset();
  }
}

KFMDiveConfig::KFMDiveConfig(
  const KeyFrameDiveTypes& keyFrameDiveType) :
  MBDiveConfig(MBDiveTypes::KEY_FRAME_MOTION_DIVE),
  keyFrameDiveType(keyFrameDiveType)
{
}

void KFMDiveConfig::validate() throw (BConfigException) 
{
}

bool KFMDiveConfig::assignFromJson(const Json::Value& obj)
{
  MBDiveConfig::assignFromJson(obj);
  try {
    // Will throw if key doesn't exist
    keyFrameDiveType = 
      (KeyFrameDiveTypes) obj.get("keyFrameDiveType", -1).asUInt();
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

Json::Value KFMDiveConfig::getJson() 
{ 
  Json::Value obj = MBDiveConfig::getJson();
  try {
    obj["name"] = "KFMDiveConfig";
    obj["keyFrameDiveType"] = (unsigned) keyFrameDiveType;
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
