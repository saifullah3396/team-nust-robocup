/**
 * @file MotionModule/src/MotionConfigs/MBBalanceConfig.cpp
 *
 * This file defines the structs MBBalanceConfig, MPComControlConfig,
 * PIDComControlConfig, and ZmpControlConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#include "MotionModule/include/MotionConfigs/MBBalanceConfig.h"

MBBalanceConfig::MBBalanceConfig(
  const MBBalanceTypes& type,
  const unsigned& supportLeg) :
  MBConfig(MBIds::BALANCE, 45.f, (int)type), // Basically a lot of time
  supportLeg(supportLeg)
{
}
  
bool MBBalanceConfig::assignFromJson(const Json::Value& obj)
{
  if (!BehaviorConfig::assignFromJson(obj))
    return false;
  try {
    // Reset max runtime from json cfg
    supportLeg = obj.get("supportLeg", 0).asUInt(); // Default value is invalid
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

Json::Value MBBalanceConfig::getJson() { 
  Json::Value obj = BehaviorConfig::getJson();
  try {
    obj["supportLeg"] = supportLeg;
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

MBBalanceConfigPtr MBBalanceConfig::makeFromJson(const Json::Value& obj)
{
  boost::shared_ptr<MBBalanceConfig> config;
  try {
    if (!obj.isNull() &&
        obj["id"].asUInt() == (unsigned)MBIds::BALANCE &&
        obj["baseType"].asUInt() == (unsigned) BaseBehaviorType::MOTION &&
        obj["type"].asUInt() <= (unsigned)MBBalanceTypes::COUNT
    ) {
      switch (obj["type"].asUInt()) {
        case (unsigned)MBBalanceTypes::MP_COM_CONTROL:
          config = boost::make_shared<MPComControlConfig>(); break;
        case (unsigned)MBBalanceTypes::PID_COM_CONTROL:
          config = boost::make_shared<PIDComControlConfig>(); break;
        case (unsigned)MBBalanceTypes::ZMP_CONTROL:
          config = boost::make_shared<ZmpControlConfig>(); break;
      }
      if (config->assignFromJson(obj)) {
        throw TNRSException(
          "Error creating a configuration from given Json object");
      }
    }
  } catch (Json::Exception& e) {
    cout 
      << "Exception caught making a balance behavior config from json object;\t";
    cout << e.what();
    config.reset();
  } catch (TNRSException& e) {
    cout << e.what();
    config.reset();
  }
  return config;
}

MPComControlConfig::MPComControlConfig(
  const unsigned& supportLeg,
  const float& timeToReachB) :
  MBBalanceConfig(MBBalanceTypes::MP_COM_CONTROL, supportLeg),
  timeToReachB(timeToReachB)
{
}

void MPComControlConfig::validate() throw (BConfigException)
{
  if (
    supportLeg != CHAIN_L_LEG &&
    supportLeg != CHAIN_R_LEG || 
    timeToReachB <= 0.5f) 
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

bool MPComControlConfig::assignFromJson(const Json::Value& obj)
{
  return MBBalanceConfig::assignFromJson(obj);
}

Json::Value MPComControlConfig::getJson() 
{ 
  Json::Value obj = MBBalanceConfig::getJson();
  try {
    obj["name"] = "MPComControl";
    obj["timeToReachB"] = timeToReachB;
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

PIDComControlConfig::PIDComControlConfig(const unsigned& supportLeg) :
  MBBalanceConfig(MBBalanceTypes::PID_COM_CONTROL, supportLeg)
{
}

void PIDComControlConfig::validate() throw (BConfigException) 
{
  if (
    supportLeg != CHAIN_L_LEG &&
    supportLeg != CHAIN_R_LEG) 
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

bool PIDComControlConfig::assignFromJson(const Json::Value& obj)
{
  return MBBalanceConfig::assignFromJson(obj);
}


Json::Value PIDComControlConfig::getJson() 
{ 
  Json::Value obj = MBBalanceConfig::getJson();
  try {
    obj["name"] = "PIDComControl";
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

ZmpControlConfig::ZmpControlConfig(
  const unsigned& supportLeg) :
  MBBalanceConfig(MBBalanceTypes::ZMP_CONTROL, supportLeg)
{
}

ZmpControlConfig::ZmpControlConfig(
  const unsigned& supportLeg,
  const ZmpRefGeneratorPtr& refGenerator) :
  MBBalanceConfig(MBBalanceTypes::ZMP_CONTROL, supportLeg),
  refGenerator(refGenerator)
{
}

void ZmpControlConfig::validate() throw (BConfigException) 
{
  return;//@TODO REMOVE LATER
  if (!refGenerator) 
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

bool ZmpControlConfig::assignFromJson(const Json::Value& obj)
{
  return MBBalanceConfig::assignFromJson(obj);
}

Json::Value ZmpControlConfig::getJson() 
{ 
  Json::Value obj = MBBalanceConfig::getJson();
  try {
    obj["name"] = "ZmpControl";
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
