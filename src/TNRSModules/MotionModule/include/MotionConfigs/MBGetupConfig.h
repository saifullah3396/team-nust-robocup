/**
 * @file MotionModule/include/MotionConfigs/MBGetupConfig.h
 *
 * This file defines the structs MBGetupConfig and KFMGetupConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "MotionModule/include/GetupModule/KeyFrameGetupTypes.h"
#include "MBConfig.h"

/**
 * @struct MBGetupConfig
 * @brief Getup behavior configuration
 */
struct MBGetupConfig : MBConfig
{
  /**
   * Constructor
   * 
   * @param type: Type of the getup behavior
   */ 
  MBGetupConfig(const MBGetupTypes& type);
  
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
  static boost::shared_ptr<MBGetupConfig> 
    makeFromJson(const Json::Value& obj);
};
typedef boost::shared_ptr<MBGetupConfig> MBGetupConfigPtr;

/**
 * @struct KFMGetupConfig
 * @brief Key frame motion getup from fall behavior configuration
 */
struct KFMGetupConfig : MBGetupConfig
{
  /**
   * Constructor
   * 
   * @param keyFrameGetupType: Type of they key frame getup motion
   */ 
  KFMGetupConfig(
    const KeyFrameGetupTypes& keyFrameGetupType = 
    (KeyFrameGetupTypes) 0);
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
  
  //! Type of the key frame getup motion
  KeyFrameGetupTypes keyFrameGetupType;
};
typedef boost::shared_ptr<KFMGetupConfig> KFMGetupConfigPtr;
