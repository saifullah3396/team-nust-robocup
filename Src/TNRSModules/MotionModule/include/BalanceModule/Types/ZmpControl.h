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

class ZmpControl : public BalanceModule
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
    BalanceModule(motionModule, config, "ZmpControl")
  {
    controllers.resize(2); // x-y dimensions
  }

  /**
   * Default destructor for this class.
   */
  ~ZmpControl()
  {
    for (int i = 0; i < controllers.size(); ++i)
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
	 * Returns the cast of config to ZmpControlConfigPtr
	 */
  ZmpControlConfigPtr getBehaviorCast();
  
  /**
   * Computes next com state based on the preview controller
   */
  ComState computeNextComState();
  
  //! Preview controller
  vector<ZmpPreviewController*> controllers;
  
  //! Desired zmp xy-coordinate reference relative to support leg.
  ZmpRef zmpRef;
  
  //! Zmp reference generator
  ZmpRefGenerator* refGenerator;
  
  //! Number of zmp reference previews required
  static unsigned nPreviews;
  
  //! Fixed com height for inverted-cart table model
  static float comHeight;
};

typedef boost::shared_ptr<ZmpControl> ZmpControlPtr;
