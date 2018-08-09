/**
 * @file PlanningModule/PlanningBehaviors/DatasetExtractor.h
 *
 * This file declares the class DatasetExtractor.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017 
 */

#pragma once

#include "ConfigManager/include/ConfigMacros.h"
#include "PlanningModule/include/PlanningBehavior.h"

/** 
 * @class DatasetExtractor
 * @brief The class for extracting and saving video data from the robot.
 */
class DatasetExtractor : public PlanningBehavior
{
public:
  /**
   * Default constructor for this class.
   * 
   * @param planningModule: pointer to parent.
   */
  DatasetExtractor(PlanningModule* planningModule) :
    PlanningBehavior(planningModule, "DatasetExtractor")
  {
    loadInitialConfig();
    behaviorState = headTapWait;//startup;
  }

  /**
   * Default destructor for this class.
   */
  ~DatasetExtractor()
  {
  }
  ;

  void
  initiate();
  void
  update();
  void
  finishBehaviorSafely();
  void
  loadInitialConfig();
  void
  setBehaviorConfig(boost::shared_ptr<BehaviorConfig> behaviorConfig);
  boost::shared_ptr<PBDatasetExtractorConfig>
  getBehaviorCast();
private:
  void
  startupAction();
  void 
  headTapWaitAction();
  void
  videoStartAction();
  void
  videoStopAction();

  unsigned behaviorState;
  enum BehaviorState
  {
    startup,
    headTapWait,
    videoStart,
    videoStop
  };
};

typedef boost::shared_ptr<DatasetExtractor> DatasetExtractorPtr;
