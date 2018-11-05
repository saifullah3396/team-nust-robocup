/**
 * @file MotionModule/src/MotionConfigs/MBPostureConfig.cpp
 *
 * This file implements the structs MBPostureConfig 
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */
 
#include "MotionModule/include/MotionConfigs/MBPostureConfig.h"
#include "MotionModule/include/PostureModule/PostureDefinitions.h"
#include "Utils/include/PostureState.h"
#include "Utils/include/HardwareIds.h"

MBPostureConfig::MBPostureConfig(
  const VectorXf& jointsToReach,
  const float& timeToReachP,
  const MBPostureTypes type) :
  MBConfig(MBIds::POSTURE, 10.f, (int)type),
  jointsToReach(jointsToReach),
  timeToReachP(timeToReachP),
  targetPosture(PostureState::UNKNOWN)
{
}


MBPostureConfig::MBPostureConfig(
  const PostureState& targetPosture,
  const float& timeToReachP,
  const MBPostureTypes type) :
  MBConfig(MBIds::POSTURE, 10.f, (int)type),
  targetPosture(targetPosture),
  timeToReachP(timeToReachP)
{
  if ((unsigned)targetPosture >= (unsigned)PostureState::STATIC_POSTURES) 
    return;
  jointsToReach = 
    VectorXf::Map(
      &postureDefinitions[(unsigned)targetPosture][0],
      sizeof(postureDefinitions[(unsigned)targetPosture]) / 
      sizeof(postureDefinitions[(unsigned)targetPosture][0])
    );
}

void MBPostureConfig::validate() throw (BConfigException) 
{
  if (timeToReachP <= 0.f || // Undefined time given
      (unsigned)targetPosture >= 
      (unsigned)PostureState::STATIC_POSTURES || 
      jointsToReach.size() != NUM_JOINTS) 
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

boost::shared_ptr<MBPostureConfig> 
MBPostureConfig::makeFromJson(const Json::Value& obj)
{
  boost::shared_ptr<MBPostureConfig> config;
  try {
    if (!obj.isNull() &&
        obj["id"].asUInt() == (unsigned)MBIds::POSTURE &&
        obj["baseType"].asUInt() == (unsigned) BaseBehaviorType::MOTION &&
        obj["type"].asUInt() <= (unsigned)MBPostureTypes::COUNT
    ) {
      switch (obj["type"].asUInt()) {
        case (unsigned)MBPostureTypes::INTERP_TO_POSTURE:
          config = 
            boost::make_shared<MBPostureConfig>(
              VectorXf(), 2.f, MBPostureTypes::INTERP_TO_POSTURE); break;
        
      }
      if (config->assignFromJson(obj)) {
        throw TNRSException(
          "Error creating a configuration from given Json object");
      }
    }
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught making a posture behavior config from json object;\t";
    cout << e.what();
    config.reset();
  } catch (TNRSException& e) {
    cout << e.what();
    config.reset();
  }
}

bool MBPostureConfig::assignFromJson(const Json::Value& obj)
{
  BehaviorConfig::assignFromJson(obj);
  try {
    targetPosture = (PostureState) obj.get("targetPosture", -1).asUInt();
    timeToReachP = obj.get("timeToReachP", 0.f).asFloat();
    const Json::Value& joints = obj["jointsToReach"];
    jointsToReach.resize(joints.size());
    for (int i = 0; i < joints.size(); ++i) {
      jointsToReach[i] = joints[i].asFloat();
    }
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

Json::Value MBPostureConfig::getJson() 
{
  Json::Value obj = BehaviorConfig::getJson();
  try {
    obj["name"] = "MBPosture";
    obj["targetPosture"] = (unsigned) targetPosture;
    obj["timeToReachP"] = timeToReachP;
    Json::Value joints;
    for (int i = 0; i < joints.size(); ++i) {
      joints[i] = jointsToReach[i];
    }
    obj["jointsToReach"] = joints;
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
