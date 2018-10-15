/**
 * @file MotionModule/src/MotionConfigs/MBKickConfig.cpp
 *
 * This file implements the structs MBKickConfig, JSKickConfig, 
 * JSE2DImpKickConfig and JSOImpKickConfig
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "MotionModule/include/MotionConfigs/MBKickConfig.h"
#include "MotionModule/include/MotionConfigs/MBBalanceConfig.h"
#include "MotionModule/include/MotionConfigs/MBPostureConfig.h"

MBKickConfig::MBKickConfig(
  const MBKickTypes& type,
  const Point2f& ball, 
  const MBPostureConfigPtr& postureConfig) :
  MBConfig(MBIds::KICK, 20.f, (int)type),
  ball(ball),
  postureConfig(postureConfig)
{
}
  
MBKickConfig::MBKickConfig(
  const MBKickTypes& type,
  const Point2f& ball) :
  MBConfig(MBIds::KICK, 20.f, (int)type),
  ball(ball)
{
}

bool MBKickConfig::assignFromJson(const Json::Value& obj)
{
  if (!BehaviorConfig::assignFromJson(obj))
    return false;
  try {
    ball.x = obj["ball"][0].asFloat();
    ball.y = obj["ball"][1].asFloat();
    reqVel.x = obj["reqVel"][0].asFloat();
    reqVel.y = obj["reqVel"][1].asFloat();
    target.x = obj["target"][0].asFloat();
    target.y = obj["target"][1].asFloat();
    const Json::Value& cfgObj = obj["postureConfig"];
    if (!cfgObj.isNull()) {
      auto pcfg = MBPostureConfig::makeFromJson(cfgObj);
      if(pcfg->assignFromJson(cfgObj)) {
        postureConfig = pcfg;
      }
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

Json::Value MBKickConfig::getJson() 
{
  Json::Value obj = BehaviorConfig::getJson();
  try {
      obj["ball"].append(ball.x);
      obj["ball"].append(ball.y);
      obj["reqVel"].append(reqVel.x);
      obj["reqVel"].append(reqVel.y);
      obj["target"].append(target.x);
      obj["target"].append(target.y);
      if (postureConfig)
        obj["postureConfig"] = postureConfig->getJson();
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

MBKickConfigPtr MBKickConfig::makeFromJson(const Json::Value& obj)
{
  boost::shared_ptr<MBKickConfig> config;
  try {
    if (!obj.isNull() &&
        obj["id"].asUInt() == (unsigned)MBIds::KICK &&
        obj["baseType"].asUInt() == (unsigned) BaseBehaviorType::MOTION &&
        obj["type"].asUInt() <= (unsigned)MBKickTypes::COUNT
    ) {
      switch (obj["type"].asUInt()) {
        case (unsigned)MBKickTypes::JOINT_SPACE_OPT_IMP_KICK:
          config = 
            boost::make_shared<JSOImpKickConfig>(); break;
        case (unsigned)MBKickTypes::JOINT_SPACE_EST_2D_IMP_KICK:
          config = 
            boost::make_shared<JSE2DImpKickConfig>(); break;
      }
      if (config->assignFromJson(obj)) {
        throw TNRSException(
          "Error creating a configuration from given Json object");
      }
    }
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught making a kick behavior config from json object;\t";
    cout << e.what();
    config.reset();
  } catch (TNRSException& e) {
    cout << e.what();
    config.reset();
  }
}

JSKickConfig::JSKickConfig(
  const MBKickTypes& type,
  const Point2f& ball, 
  const boost::shared_ptr<MBPostureConfig>& postureConfig,
  const boost::shared_ptr<MBBalanceConfig>& balanceConfig,
  const float& minTimeToKick) :
  MBKickConfig(type, ball, postureConfig),
  balanceConfig(balanceConfig),
  minTimeToKick(minTimeToKick)
{
  target = Point2f(-1.f, -1.f);
  reqVel = Point2f(-1.f, -1.f);
}

JSKickConfig::JSKickConfig(
const MBKickTypes& type,
  const Point2f& ball, 
  const boost::shared_ptr<MBBalanceConfig>& balanceConfig,
  const float& minTimeToKick) :
  MBKickConfig(type, ball),
  balanceConfig(balanceConfig),
  minTimeToKick(minTimeToKick)
{
  target = Point2f(-1.f, -1.f);
  reqVel = Point2f(-1.f, -1.f);
}

JSKickConfig::JSKickConfig(
  const MBKickTypes& type,
  const Point2f& ball, 
  const float& minTimeToKick) :
  MBKickConfig(type, ball),
  minTimeToKick(minTimeToKick)
{
  target = Point2f(-1.f, -1.f);
  reqVel = Point2f(-1.f, -1.f);
}

bool JSKickConfig::assignFromJson(const Json::Value& obj)
{
  if (!MBKickConfig::assignFromJson(obj))
    return false;
  try {
    minTimeToKick = obj.get("minTimeToKick", -1.f).asFloat();
    const Json::Value& cfgObj = obj["balanceConfig"];
    if (!cfgObj.isNull()) {
      auto bcfg = MBBalanceConfig::makeFromJson(cfgObj);
      if(bcfg->assignFromJson(cfgObj)) {
        balanceConfig = bcfg;
      }
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

Json::Value JSKickConfig::getJson() { 
  Json::Value obj = MBKickConfig::getJson();
  try {
  obj["name"] = "JSKick"; // Reassign name
  obj["minTimeToKick"] = minTimeToKick; // Reassign name
  if (balanceConfig)
    obj["balanceConfig"] = balanceConfig->getJson();
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

JSOImpKickConfig::JSOImpKickConfig(
  const Point2f& ball, 
  const boost::shared_ptr<MBPostureConfig>& postureConfig,
  const boost::shared_ptr<MBBalanceConfig>& balanceConfig,
  const float& minTimeToKick) :
  JSKickConfig(
    MBKickTypes::JOINT_SPACE_OPT_IMP_KICK, 
    ball, 
    postureConfig,
    balanceConfig,
    minTimeToKick
  )
{
}

JSOImpKickConfig::JSOImpKickConfig(
  const Point2f& ball,  
  const boost::shared_ptr<MBBalanceConfig>& balanceConfig,
  const float& minTimeToKick) :
  JSKickConfig(
    MBKickTypes::JOINT_SPACE_OPT_IMP_KICK, 
    ball,
    balanceConfig,
  minTimeToKick
  )
{
}

JSOImpKickConfig::JSOImpKickConfig(
  const Point2f& ball,  
  const float& minTimeToKick) :
  JSKickConfig(
    MBKickTypes::JOINT_SPACE_OPT_IMP_KICK, 
    ball,
    minTimeToKick
  )
{
}

void JSOImpKickConfig::validate() throw (BConfigException) 
{
  if (ball.x < 0.f ||
      (reqVel.x == -1.f && target.x == -1.f) || 
      minTimeToKick <= 0.f)
  { 
    cout << "Parameters: \n" << endl;
    cout << "Ball: " << ball << endl;
    cout << "ReqVel: " << reqVel << endl;
    cout << "target: " << target << endl;
    cout << "minTimeToKick: " << minTimeToKick << endl;
    throw 
      BConfigException(
        this, 
        "Invalid behavior configuration parameters passed.", 
        false, 
        EXC_INVALID_BCONFIG_PARAMETERS
      );
  }
}

bool JSOImpKickConfig::assignFromJson(const Json::Value& obj)
{
  if (!JSKickConfig::assignFromJson(obj))
    return false;
  return true;
}

Json::Value JSOImpKickConfig::getJson() 
{ 
  Json::Value obj = JSKickConfig::getJson();
  try {
    obj["name"] = "JSOImpKick"; // Reassign name
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

JSE2DImpKickConfig::JSE2DImpKickConfig(
  const Point2f& ball, 
  const Point2f& ballVel,
  const double& timeUntilImpact,
  const high_resolution_clock::time_point& timeAtEstimation,
  const boost::shared_ptr<MBPostureConfig>& postureConfig,
  const boost::shared_ptr<MBBalanceConfig>& balanceConfig,
  const float& minTimeToKick) :
  JSKickConfig(
    MBKickTypes::JOINT_SPACE_EST_2D_IMP_KICK, 
    ball, 
    postureConfig,
    balanceConfig,
    minTimeToKick
  ),
  ballVel(ballVel),
  timeUntilImpact(timeUntilImpact),
  timeAtEstimation(timeAtEstimation)
{
}

JSE2DImpKickConfig::JSE2DImpKickConfig(
  const Point2f& ball,  
  const Point2f& ballVel,
  const double& timeUntilImpact,
  const high_resolution_clock::time_point& timeAtEstimation,
  const boost::shared_ptr<MBBalanceConfig>& balanceConfig,
  const float& minTimeToKick) :
  JSKickConfig(
    MBKickTypes::JOINT_SPACE_EST_2D_IMP_KICK, 
    ball,
    balanceConfig,
  minTimeToKick
  ),
  ballVel(ballVel),
  timeUntilImpact(timeUntilImpact),
  timeAtEstimation(timeAtEstimation)
{
}

JSE2DImpKickConfig::JSE2DImpKickConfig(
  const Point2f& ball,  
  const Point2f& ballVel,
  const double& timeUntilImpact,
  const high_resolution_clock::time_point& timeAtEstimation,
  const float& minTimeToKick) :
  JSKickConfig(
    MBKickTypes::JOINT_SPACE_EST_2D_IMP_KICK, 
    ball,
    minTimeToKick
  ),
  ballVel(ballVel),
  timeUntilImpact(timeUntilImpact),
  timeAtEstimation(timeAtEstimation)
{
}

JSE2DImpKickConfig::JSE2DImpKickConfig(
  const Point2f& ball,  
  const Point2f& ballVel,
  const double& timeUntilImpact,
  const float& minTimeToKick) :
  JSKickConfig(
    MBKickTypes::JOINT_SPACE_EST_2D_IMP_KICK, 
    ball,
    minTimeToKick
  ),
  ballVel(ballVel),
  timeUntilImpact(timeUntilImpact),
  timeAtEstimation(timeAtEstimation)
{
}

void JSE2DImpKickConfig::validate() throw (BConfigException) 
{
  if (ball.x < 0.f ||
      target.x == -1.f || 
      minTimeToKick <= 0.f || 
      timeUntilImpact < 0.5f) // Minimum time given to kick should be 0.5 secs
  { 
    cout << "Parameters: \n" << endl;
    cout << "Ball: " << ball << endl;
    cout << "BallVel: " << ballVel << endl;
    cout << "ReqVel: " << reqVel << endl;
    cout << "Target: " << target << endl;
    cout << "MinTimeToKick: " << minTimeToKick << endl;
    cout << "TimeUntilImpact: " << timeUntilImpact << endl;
    throw 
      BConfigException(
        this, 
        "Invalid behavior configuration parameters passed.", 
        false, 
        EXC_INVALID_BCONFIG_PARAMETERS
      );
  }
}

bool JSE2DImpKickConfig::assignFromJson(const Json::Value& obj)
{
  if (!JSKickConfig::assignFromJson(obj))
    return false;
  try {
    ballVel.x = obj["ballVel"][0].asFloat();
    ballVel.y = obj["ballVel"][1].asFloat();
    timeUntilImpact = obj.get("timeUntilImpact", 0.f).asFloat();
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

Json::Value JSE2DImpKickConfig::getJson() 
{ 
  Json::Value obj = JSKickConfig::getJson();
  try {
    obj["name"] = "JSE2DImpKick";
    obj["ballVel"].append(ballVel.x);
    obj["ballVel"].append(ballVel.y);
    obj["timeUntilImpact"] = timeUntilImpact;
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
