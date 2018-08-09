/**
 * @file MotionModule/include/MotionConfigs/MBBallThrowConfig.h
 *
 * This file defines the struct MBBallThrowConfig
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "MBConfig.h"

/**
 * @struct MBBallThrowConfig
 * @brief Ball throw behavior configuration
 */
struct MBBallThrowConfig : MBConfig
{
  /**
   * @param type: Type of the ball throw behavior
   * @param timeToThrow: Total ball throw execution time
   * @param headTapToStart: Whether to execute behavior on head tap
   * @param 
   */ 
  MBBallThrowConfig(
    const MBBallThrowTypes& type = MBBallThrowTypes::WB_BALL_THROW,
    const float& timeToThrow = 2.f,
    const bool& headTapToStart = false) :
    MBConfig(MBIds::BALL_THROW, 45.f, (int)type),
    timeToThrow(timeToThrow),
    headTapToStart(headTapToStart)
  {
  }

  /**
   * Returns true if the configuration parameters are valid
   * 
   * @return bool
   */
  bool isValid() {
    try {
      if (timeToThrow <= 0.f) 
      { 
        throw 
          BConfigException(
            this, 
            "Invalid behavior configuration parameters passed.", 
            false, 
            EXC_INVALID_BCONFIG_PARAMETERS
          );
      }
      return true;
    } catch (BConfigException& e) {
      cout << e.what();
      return false;
    }
  }

  //! Total ball throw execution time
  float timeToThrow;
  
  //! Whether to execute behavior on head tap
  bool headTapToStart;
};
typedef boost::shared_ptr<MBBallThrowConfig> MBBallThrowConfigPtr;
