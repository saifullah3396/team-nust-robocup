/**
 * @file MotionModule/BalanceModule/Types/ZmpControl.h
 *
 * This file declares the class ZmpControl
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include "MotionModule/include/BalanceModule/BalanceModule.h"
#include "MotionModule/include/BalanceModule/ZmpPreviewController.h"
#include "MotionModule/include/BalanceModule/ZmpRefGenerator.h"
#include "MotionModule/include/MotionConfigs/MBBalanceConfig.h"

template <typename Scalar>
class MotionTask;
typedef boost::shared_ptr<MotionTask<MType> > TaskPtr;

template <typename Scalar>
class ZmpControl : public BalanceModule<Scalar>
{
public:
  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   */
  ZmpControl(
    MotionModule* motionModule,
    const BehaviorConfigPtr& config);

  /**
   * Default destructor for this class.
   */
  ~ZmpControl();
  
  /**
   * Derived from Behavior
   */ 
  void initiate();
  void reinitiate(const BehaviorConfigPtr& cfg);
  void update();
  void finish();
  void loadExternalConfig();
private:
  /**
   * Tracks the zmp by following center of mass trajectories resulting from
   * preview controller.
   */ 
  void trackZmp();

  /**
	 * Returns the cast of config to ZmpControlConfigPtr
	 */
  ZmpControlConfigPtr getBehaviorCast();
  
  //! Preview controller
  vector<ZmpPreviewController<Scalar>*> controllers;
  
  //! Desired zmp xy-coordinate reference relative to support leg.
  ZmpRef<Scalar> zmpRef;
  
  //! Zmp reference generator
  ZmpRefGenerator<Scalar>* refGenerator;
  
  //! Number of zmp reference previews required
  static unsigned nPreviews;
  
  //! Fixed com height for inverted-cart table model
  static Scalar comHeight;

  //! Tasks for solving inverse kinematics
  vector<TaskPtr> tasks;

  //! Log for storing center of mass data
  fstream comLog;

  //! Log for storing zmp ref data
  fstream zmpRegLog;

  //! Execution time of behavior
  Scalar execTime;

  enum Tasks {
    COM_TASK,
    POSTURE_TASK,
    CONTACT_TASK,
    TORSO_TASK,
    NUM_TASKS
  };
};

typedef boost::shared_ptr<ZmpControl<MType> > ZmpControlPtr;
