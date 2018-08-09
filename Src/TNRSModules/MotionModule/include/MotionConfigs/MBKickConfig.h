/**
 * @file MotionModule/include/MotionConfigs/MBKickConfig.h
 *
 * This file defines the structs MBKickConfig, JSKickConfig, 
 * JSE2DImpKickConfig and JSOImpKickConfig
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 3 April 2018
 */

#pragma once

#include "MBConfig.h"
#include "MBBalanceConfig.h"
#include "MBPostureConfig.h"

/**
 * @struct MBKickConfig
 * @brief Kick behavior configuration
 */
struct MBKickConfig : MBConfig
{
  /**
   * Constructor
   * 
   * @param type: Type of the kick behavior
   * @param ball: Initial ball position
   * @param postureConfig: If passed the robot first goes to the desired
   *   posture
   */ 
  MBKickConfig(
    const MBKickTypes& type,
    const Point2f& ball, 
    const MBPostureConfigPtr& postureConfig) :
    MBConfig(MBIds::KICK, 20.f, (int)type),
    ball(ball),
    postureConfig(postureConfig)
  {
  }
  
  /**
   * Constructor
   * 
   * @param type: Type of the kick behavior
   * @param ball: Initial ball position
   */ 
  MBKickConfig(
    const MBKickTypes& type,
    const Point2f& ball) :
    MBConfig(MBIds::KICK, 20.f, (int)type),
    ball(ball)
  {
  }
  
  //! Target of the kick in robot frame
  Point2f target;
  
  //! Required velocity of the end-effector
  Point2f reqVel;
  
  //! Initial position of the ball
  Point2f ball;
  
  //! Posture configuration
  MBPostureConfigPtr postureConfig;
};
typedef boost::shared_ptr<MBKickConfig> MBKickConfigPtr;

/**
 * @struct JSImpKickConfig
 * @brief Joint space base kick behavior configuration
 */
struct JSKickConfig : MBKickConfig
{
  /**
   * Constructor
   * 
   * @param type: Type of the kick behavior
   * @param ball: Initial ball position
   * @param postureConfig: If passed the robot first goes to the desired
   *   posture
   * @param balanceConfig: If passed the balancer config is used for 
   *   shifting balance of the robot to the support leg
   * @param minTimeToKick: Minimum possible time for overall kick 
   *   trajectory. This is important if we don't want the kick to be
   *   too fast even if we're using time optimization
   */
  JSKickConfig(
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
  
  /**
   * Constructor
   * 
   * @param type: Type of the kick behavior
   * @param ball: Initial ball position
   * @param balanceConfig: If passed the balancer config is used for 
   *   shifting balance of the robot to the support leg
   * @param minTimeToKick: Minimum possible time for overall kick 
   *   trajectory. This is important if we don't want the kick to be
   *   too fast even if we're using time optimization
   */
  JSKickConfig(
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
  
  /**
   * Constructor
   * 
   * @param type: Type of the kick behavior
   * @param ball: Initial ball position
   * @param minTimeToKick: Minimum possible time for overall kick 
   *   trajectory. This is important if we don't want the kick to be
   *   too fast even if we're using time optimization
   */
  JSKickConfig(
	const MBKickTypes& type,
    const Point2f& ball, 
	const float& minTimeToKick) :
    MBKickConfig(type, ball),
    minTimeToKick(minTimeToKick)
  {
    target = Point2f(-1.f, -1.f);
    reqVel = Point2f(-1.f, -1.f);
  }
  
  //! Minimum possible time for overall kick trajectory.
  float minTimeToKick;
  
  //! Balance configuration
  MBBalanceConfigPtr balanceConfig; 
};
typedef boost::shared_ptr<JSKickConfig> JSKickConfigPtr;

/**
 * @struct JSOImpKickConfig
 * @brief Joint space optimized impulse kick behavior configuration
 */
struct JSOImpKickConfig : JSKickConfig
{
  /**
   * Constructor
   * 
   * @param ball: Initial ball position
   * @param postureConfig: If passed the robot first goes to the desired
   *   posture
   * @param balanceConfig: If passed the balancer config is used for 
   *   shifting balance of the robot to the support leg
   * @param minTimeToKick: Minimum possible time for overall kick 
   *   trajectory. This is important if we don't want the kick to be
   *   too fast even if we're using time optimization
   */
  JSOImpKickConfig(
    const Point2f& ball, 
    const boost::shared_ptr<MBPostureConfig>& postureConfig,
    const boost::shared_ptr<MBBalanceConfig>& balanceConfig,
    const float& minTimeToKick = 1.f) :
    JSKickConfig(
      MBKickTypes::JOINT_SPACE_OPT_IMP_KICK, 
      ball, 
      postureConfig,
      balanceConfig,
      minTimeToKick
    )
  {
  }
  
  /**
   * Constructor
   * 
   * @param ball: Initial ball position
   * @param balanceConfig: If passed the balancer config is used for 
   *   shifting balance of the robot to the support leg
   * @param minTimeToKick: Minimum possible time for overall kick 
   *   trajectory. This is important if we don't want the kick to be
   *   too fast even if we're using time optimization
   */
  JSOImpKickConfig(
    const Point2f& ball,  
    const boost::shared_ptr<MBBalanceConfig>& balanceConfig,
    const float& minTimeToKick = 1.f) :
    JSKickConfig(
      MBKickTypes::JOINT_SPACE_OPT_IMP_KICK, 
      ball,
      balanceConfig,
	  minTimeToKick
    )
  {
  }
  
  /**
   * Constructor
   * 
   * @param ball: Initial ball position
   * @param minTimeToKick: Minimum possible time for overall kick 
   *   trajectory. This is important if we don't want the kick to be
   *   too fast even if we're using time optimization
   */
  JSOImpKickConfig(
    const Point2f& ball,  
    const float& minTimeToKick = 1.f) :
    JSKickConfig(
      MBKickTypes::JOINT_SPACE_OPT_IMP_KICK, 
      ball,
      minTimeToKick
    )
  {
  }
  
  /**
   * Returns true if the configuration is valid
   * 
   * @return bool
   */ 
  bool isValid() {
    try {
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
      return true;
    } catch (BConfigException& e) {
      cout << e.what();
      return false;
    }
  }
};
typedef boost::shared_ptr<JSOImpKickConfig> JSOImpKickConfigPtr;

/**
 * @struct JSE2DImpKickConfig
 * @brief Joint space estimated 2D impulse kick behavior configuration
 */
struct JSE2DImpKickConfig : JSKickConfig
{
  /**
   * Constructor
   * 
   * @param ball: Initial ball position
   * @param ballVel: Estimated initial ball velocity which ball reaches
   *   the initial position
   * @param timeUntilImpact: Estimated time until ball reaches the 
   *   initial position
   * @param postureConfig: If passed the robot first goes to the desired
   *   posture
   * @param balanceConfig: If passed the balancer config is used for 
   *   shifting balance of the robot to the support leg
   * @param minTimeToKick: Minimum possible time for overall kick 
   *   trajectory. This is important if we don't want the kick to be
   *   too fast even if we're using time optimization
   */
  JSE2DImpKickConfig(
    const Point2f& ball, 
	const Point2f& ballVel,
	const float& timeUntilImpact,
    const boost::shared_ptr<MBPostureConfig>& postureConfig,
	const boost::shared_ptr<MBBalanceConfig>& balanceConfig,
	const float& minTimeToKick = 1.f) :
    JSKickConfig(
      MBKickTypes::JOINT_SPACE_EST_2D_IMP_KICK, 
      ball, 
      postureConfig,
      balanceConfig,
      minTimeToKick
    ),
    ballVel(ballVel),
    timeUntilImpact(timeUntilImpact)
  {
  }
  
  /**
   * Constructor
   * 
   * @param ball: Initial ball position
   * @param ballVel: Estimated initial ball velocity which ball reaches
   *   the initial position
   * @param timeUntilImpact: Estimated time until ball reaches the 
   *   initial position
   * @param balanceConfig: If passed the balancer config is used for 
   *   shifting balance of the robot to the support leg
   * @param minTimeToKick: Minimum possible time for overall kick 
   *   trajectory. This is important if we don't want the kick to be
   *   too fast even if we're using time optimization
   */
  JSE2DImpKickConfig(
    const Point2f& ball,  
    const Point2f& ballVel,
	const float& timeUntilImpact,
	const boost::shared_ptr<MBBalanceConfig>& balanceConfig,
	const float& minTimeToKick = 1.f) :
    JSKickConfig(
      MBKickTypes::JOINT_SPACE_EST_2D_IMP_KICK, 
      ball,
      balanceConfig,
	  minTimeToKick
    ),
    ballVel(ballVel),
    timeUntilImpact(timeUntilImpact)
  {
  }
  
  /**
   * Constructor
   * 
   * @param ball: Initial ball position
   * @param ballVel: Estimated initial ball velocity which ball reaches
   *   the initial position
   * @param timeUntilImpact: Estimated time until ball reaches the 
   *   initial position
   * @param minTimeToKick: Minimum possible time for overall kick 
   *   trajectory. This is important if we don't want the kick to be
   *   too fast even if we're using time optimization
   */
  JSE2DImpKickConfig(
    const Point2f& ball,  
    const Point2f& ballVel,
	const float& timeUntilImpact,
	const float& minTimeToKick = 1.f) :
    JSKickConfig(
      MBKickTypes::JOINT_SPACE_EST_2D_IMP_KICK, 
      ball,
      minTimeToKick
    ),
    ballVel(ballVel),
    timeUntilImpact(timeUntilImpact)
  {
  }
  
  /**
   * Returns true if the configuration is valid
   * 
   * @return bool
   */ 
  bool isValid() {
    try {
      if (ball.x < 0.f ||
          (reqVel.x == -1.f && target.x == -1.f) || 
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
      return true;
    } catch (BConfigException& e) {
      cout << e.what();
      return false;
    }
  }
  
  //! Estimated initial ball velocity which ball reaches the initial position
  Point2f ballVel;
  
  //! Estimated time until ball reaches the initial position
  float timeUntilImpact;
};
typedef boost::shared_ptr<JSE2DImpKickConfig> JSE2DImpKickConfigPtr;
