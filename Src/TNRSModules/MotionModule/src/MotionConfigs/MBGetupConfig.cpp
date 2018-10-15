/**
 * @file MotionModule/src/MotionConfigs/MBGetupConfig.cpp
 *
 * This file implements the structs MBGetupConfig and KFMGetupConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "MotionModule/include/GetupModule/KeyFrameGetupTypes.h"
#include "MotionModule/include/MotionConfigs/MBGetupConfig.h"


MBGetupConfig::MBGetupConfig(const MBGetupTypes& type) :
  MBConfig(MBIds::GETUP, 10.f, (int)type)
{
}

bool MBGetupConfig::assignFromJson(const Json::Value& obj)
{
  if (!BehaviorConfig::assignFromJson(obj))
    return false;
  return true;
}

Json::Value MBGetupConfig::getJson() { 
  Json::Value obj = BehaviorConfig::getJson();
  return obj;
}
  
MBGetupConfigPtr MBGetupConfig::makeFromJson(const Json::Value& obj)
{
  boost::shared_ptr<MBGetupConfig> config;
  try {
    if (!obj.isNull() &&
        obj["id"].asUInt() == (unsigned)MBIds::GETUP &&
        obj["baseType"].asUInt() == (unsigned)BaseBehaviorType::MOTION &&
        obj["type"].asUInt() <= (unsigned)MBGetupTypes::COUNT
    ) {
      switch (obj["type"].asUInt()) {
        case (unsigned)MBGetupTypes::KEY_FRAME_MOTION_GETUP:
          config = 
            boost::make_shared<KFMGetupConfig>(); break;
        
      }
      if (config->assignFromJson(obj)) {
        throw TNRSException(
          "Error creating a configuration from given Json object");
      }
    }
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught making a getup behavior config from json object;\t";
    cout << e.what();
    config.reset();
  } catch (TNRSException& e) {
    cout << e.what();
    config.reset();
  }
}

KFMGetupConfig::KFMGetupConfig(
  const KeyFrameGetupTypes& keyFrameGetupType) :
  MBGetupConfig(MBGetupTypes::KEY_FRAME_MOTION_GETUP),
  keyFrameGetupType(keyFrameGetupType)
{
}

void KFMGetupConfig::validate() throw (BConfigException) {
}

bool KFMGetupConfig::assignFromJson(const Json::Value& obj)
{
  MBGetupConfig::assignFromJson(obj);
  try {
    // Will throw if key doesn't exist
    keyFrameGetupType = 
      (KeyFrameGetupTypes) obj.get("keyFrameGetupType", -1).asUInt();
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
  
Json::Value KFMGetupConfig::getJson() { 
  Json::Value obj = MBGetupConfig::getJson();
  try {
    obj["name"] = "KFMGetupConfig";
    obj["keyFrameGetupType"] = (unsigned) keyFrameGetupType;
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

