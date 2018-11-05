/**
 * @file SBModule/include/WhistleDetector/WhistleDetector.h
 *
 * This file declares the class WhistleDetector
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

#include "SBModule/include/WhistleDetector/SoundConfig.h"
#include "SBModule/include/WhistleDetector/ALSARecorder.h"
#include "SBModule/include/WhistleDetector/STFT.h"
#include "Utils/include/ConfigMacros.h"
#include "SBModule/include/StaticBehavior.h"
/**
 * @class WhistleDetector
 * @brief Base class for behaviors to detect the whistle
 */ 
class WhistleDetector : public StaticBehavior
{
public:

  /**
   * Constructor
   * 
   * @param sbModule: Pointer to base static behaviors module
   * @param config: Configuration of the behavior
   * @param name: Name of the behavior
   */
  WhistleDetector(
	SBModule* sbModule,
	const BehaviorConfigPtr& config,
	const string& name = "WhistleDetector") :
    StaticBehavior(sbModule, config, name),
    execTime(0.f)
  {
  }

  /**
   * Destructor
   */
  virtual ~WhistleDetector()
  {
  }
  
  /**
   * Returns its own child based on the given type
   * 
   * @param sbModule: Pointer to base static behaviors module
   * @param cfg: Config of the requested behavior
   * 
   * @return BehaviorConfigPtr
   */
  static boost::shared_ptr<WhistleDetector> getType(
    SBModule* sbModule, const BehaviorConfigPtr& type);
    
  //! Child type may or may not use the same behavior config as parent
  virtual void loadExternalConfig() {}

protected:
 /**
  * Returns the casts of config to SBWDConfigPtr
  */ 
  SBWDConfigPtr getBehaviorCast();

  /**
   * Action to perform when the whistle is detected
   */ 
  void whistleAction();

  //! Time as in how long should we check for whistle
  float timeToDetect;
  
  //! Behavior time after initiation
  float execTime;
};

typedef boost::shared_ptr<WhistleDetector> WhistleDetectorPtr;
