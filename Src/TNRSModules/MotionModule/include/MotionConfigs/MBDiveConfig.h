/**
 * @file MotionModule/include/MotionConfigs/MBDiveConfig.h
 *
 * This file defines the structs MBDiveConfig and KFMDiveConfig
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
  MBDiveConfig(const MBDiveTypes& type) :
    MBConfig(MBIds::DIVE, 5.f, (int)type)
  {
  }
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
    const KeyFrameDiveTypes& keyFrameDiveType = KeyFrameDiveTypes::IN_PLACE) :
    MBDiveConfig(MBDiveTypes::KEY_FRAME_MOTION_DIVE),
    keyFrameDiveType(keyFrameDiveType)
  {
  }
  
  /**
   * Returns true if the configuration is valid
   */ 
  bool isValid() {
    return true;
  }
  
  //! Type of the key frame dive
  KeyFrameDiveTypes keyFrameDiveType;
};
typedef boost::shared_ptr<KFMDiveConfig> KFMDiveConfigPtr;
