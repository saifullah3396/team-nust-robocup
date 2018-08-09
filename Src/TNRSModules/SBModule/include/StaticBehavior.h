/**
 * @file SBModule/include/StaticBehavior.h
 *
 * This file declares the class StaticBehavior
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

#include "BehaviorManager/include/Behavior.h"
#include "SBModule/include/SBModule.h"
#include "SBModule/include/SBConfigs.h"
#include "SBModule/include/StaticBehaviorIds.h"

static const string stiffnessJointNames[24] =
  { "HeadYaw", "HeadPitch", "LShoulderPitch", "LShoulderRoll",
    "LElbowYaw", "LElbowRoll", "LWristYaw", "RShoulderPitch",
    "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw",
    "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch",
    "LAnklePitch", "LAnkleRoll", "RHipYawPitch", "RHipRoll",
    "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll"
  };

/**
 * @class StaticBehavior
 * @brief A base class for all kinds of static behaviors
 */
class StaticBehavior : public Behavior, public MemoryBase
{
public:
  /**
   * Constructor
   *
   * @param sbModule: Pointer to base static behaviors module
   * @param config: Configuration of the behavior
   * @param name: Name of the behavior
   */
  StaticBehavior(
		SBModule* sbModule,
		const BehaviorConfigPtr& config,
		const string& name = "Not assigned.") :
    Behavior(config, name),
    MemoryBase(sbModule),
    sbModule(sbModule)
  {
    motionProxy = sbModule->getSharedMotionProxy();
    cycleTime = sbModule->getPeriodMinMS() / 1000.f;
  }

  /**
   * Destructor
   */
  virtual ~StaticBehavior()
  {
  }

protected:
  /**
   * Interpolates stiffness using naoqi stiffness interpolation
   *
   * @param ids: Ids of all the joints under consideration
   * @param timeLists: Times for interpolation of each joint
   * @param stiffnessLists: Stiffnesses for interpolation of each joint
   * @param postCommand: Whether this command is to be run in a
   *   separate thread or as a blocking call
   */
  void naoqiStiffnessInterpolation(
    const vector<unsigned>& ids,
    const AL::ALValue& timeLists,
    const AL::ALValue& stiffnessLists,
    const bool& postCommand);

  //! Cycle time of this static behavior
  float cycleTime;

  //! Naoqi motion proxy object
  ALMotionProxyPtr motionProxy;

  //! Pointer to base static module
  SBModule* sbModule;
};
