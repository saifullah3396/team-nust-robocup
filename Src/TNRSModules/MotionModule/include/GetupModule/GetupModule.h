/**
 * @file MotionModule/include/GetupModule/GetupModule.h
 *
 * This file declares the class GetupModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 26 Dec 2017
 */

#pragma once

#include "MotionModule/include/GetupModule/KeyFrameGetupTypes.h"
#include "MotionModule/include/MotionBehavior.h"
#include "MotionModule/include/MotionConfigs/MBGetupConfig.h"

/**
 * @class GetupModule
 * @brief The class for generating motions for standing up if a robot
 *   is fallen.
 */
template<typename Scalar>
class GetupModule : public MotionBehavior<Scalar>
{
public:
  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   * @param name: Name of the behavior
   */
  GetupModule(
    MotionModule* motionModule,
    const BehaviorConfigPtr& config,
		const string& name = "Not assigned.") :
    MotionBehavior<Scalar>(motionModule, config, name)
  {
  }

  /**
   * Destructor
   */
  ~GetupModule()
  {
  }
  
  /**
   * Returns its own child based on the given type
   * 
   * @param motionModule: Pointer to base motion module
   * @param cfg: Config of the requested behavior
   * 
   * @return BehaviorConfigPtr
   */
  static boost::shared_ptr<GetupModule<Scalar> > getType(
    MotionModule* motionModule, const BehaviorConfigPtr& cfg);

  /**
   * Derived from Behavior. Child type may or may not use the same 
   * behavior config as parent.
   */
  virtual void loadExternalConfig() {}
  
protected:
  //! The final posture state after getting up
  PostureState endPosture;
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<GetupModule<MType> > GetupModulePtr;
