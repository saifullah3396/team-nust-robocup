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
  MBGetupConfig(const MBGetupTypes& type) :
    MBConfig(MBIds::GETUP, 10.f, (int)type)
  {
  }
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
    const KeyFrameGetupTypes& keyFrameGetupType) :
    MBGetupConfig(MBGetupTypes::KEY_FRAME_MOTION_GETUP),
    keyFrameGetupType(keyFrameGetupType)
  {
  }
  
  /**
   * Returns true if the configuration parameters are valid
   * 
   * @return bool
   */ 
  bool isValid() {
    return true;
  }
  
  //! Type of the key frame getup motion
  KeyFrameGetupTypes keyFrameGetupType;
};
typedef boost::shared_ptr<KFMGetupConfig> KFMGetupConfigPtr;
