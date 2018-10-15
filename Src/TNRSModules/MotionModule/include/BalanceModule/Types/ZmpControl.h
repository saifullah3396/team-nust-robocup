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
    const BehaviorConfigPtr& config) :
    BalanceModule<Scalar>(motionModule, config, "ZmpControl"),
    execTime(Scalar(0.0))
  {
    controllers.resize(2); // x-y dimensions
  }

  /**
   * Default destructor for this class.
   */
  ~ZmpControl()
  {
    for (size_t i = 0; i < controllers.size(); ++i)
      delete controllers[i];
  }
  
  /**
   * Derived from Behavior
   */ 
  void initiate();
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
  
  /**
   * Computes next com state based on the preview controller
   */
  ComState<Scalar> computeNextComState();
  
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

  //! Log for storing center of mass data
  fstream comLog;

  //! Log for storing zmp ref data
  fstream zmpRegLog;

  //! Execution time of behavior
  Scalar execTime;
};

typedef boost::shared_ptr<ZmpControl<MType> > ZmpControlPtr;
