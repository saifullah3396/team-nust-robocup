/**
 * @file MotionModule/include/MotionConfigs/MBDiveConfig.h
 *
 * This file declares the structs MBDiveConfig and KFMDiveConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "MBConfig.h"
#include "MotionModule/include/DiveModule/KeyFrameDiveTypes.h"

/**
 * @struct MBDiveConfig
 * @brief Dive behavior configuration
 */
struct MBDiveConfig : MBConfig
{
  /**
   * Constructor
   * 
   * @param type: Type of the dive behavior
   */ 
  MBDiveConfig(const MBDiveTypes& type);
  
  /**
   * @derived
   */ 
  virtual bool assignFromJson(const Json::Value& obj);
  
  /**
   * @derived
   */
  virtual Json::Value getJson();
  
  /**
   * Makes an object of type this and returns it if valid
   */ 
  static boost::shared_ptr<MBDiveConfig> makeFromJson(const Json::Value& obj);
};
typedef boost::shared_ptr<MBDiveConfig> MBDiveConfigPtr;

/**
 * @struct KFMDiveConfig
 * @brief Key frame motion dive behavior configuration
 */
struct KFMDiveConfig : MBDiveConfig
{
  /**
   * Constructor
   * 
   * @param keyFrameDiveType: Type of the key frame dive
   */ 
  KFMDiveConfig(
    const KeyFrameDiveTypes& keyFrameDiveType = KeyFrameDiveTypes::IN_PLACE);
  
  /**
   * @derived
   */ 
  void validate() throw (BConfigException);
  
  /**
   * @derived
   */ 
  virtual bool assignFromJson(const Json::Value& obj);
  
  /**
   * @derived
   */
  virtual Json::Value getJson();
  
  //! Type of the key frame dive
  KeyFrameDiveTypes keyFrameDiveType;
};
typedef boost::shared_ptr<KFMDiveConfig> KFMDiveConfigPtr;
