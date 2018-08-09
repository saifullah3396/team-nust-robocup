/**
 * @file VisionModule/include/FeatureExtraction//GoalExtraction.h
 *
 * This file declares the class for goal extraction from
 * the image.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 22 Aug 2017 
 */

#pragma once

#include "VisionModule/include/FeatureExtraction/FeatureExtraction.h"
#include "VisionModule/include/FeatureExtraction/FieldExtraction.h"
#include "VisionModule/include/FeatureExtraction/GoalPost.h"
#include "VisionModule/include/FeatureExtraction/RobotExtraction.h"
#include "Utils/include/Landmark.h"
#include "Utils/include/ObstacleType.h"

/**
 * @class GoalExtraction
 * @brief The class for extracting goal from the input image.
 */
class GoalExtraction : public FeatureExtraction, public DebugBase
{
INIT_DEBUG_BASE_(
  //! Option to send total module time.
  (int, sendTime, 0),
  //! Option to draw the goal posts base points.
  (int, drawGoalPostBase, 0),
)

public:

  /**
   * Default constructor for this class.
   *
   * @param visionModule: Pointer to parent VisionModule.
   */
  GoalExtraction(VisionModule* visionModule) :
    FeatureExtraction(visionModule), DebugBase("GoalExtraction", this)
  {
    initDebugBase();
    int tempSendTime;
    int tempDrawGoalPostBase;
    houghSettings.resize(3);
    GET_CONFIG(
      "VisionDebug",
      (int, GoalExtraction.sendTime, tempSendTime), (int, GoalExtraction.drawGoalPostBase, tempDrawGoalPostBase), (int, GoalExtraction.hltThreshold, houghSettings[0]), (int, GoalExtraction.hltLineLength, houghSettings[1]), (int, GoalExtraction.hltLineGap, houghSettings[2]), )
    SET_DVAR(int, sendTime, tempSendTime);
    SET_DVAR(int, drawGoalPostBase, tempDrawGoalPostBase);
    fieldExt = GET_FEATURE_EXT_CLASS(FieldExtraction, FIELD);
    robotExt = GET_FEATURE_EXT_CLASS(RobotExtraction, ROBOT);
    regionSeg = GET_FEATURE_EXT_CLASS(RegionSegmentation, REGION_SEGMENTATION);
  }

  /**
   * Default destructor for this class.
   */
  ~GoalExtraction()
  {
  }

  //! Main image processing routine for this feature extraction class.
  void
  processImage();

  /**
   * Returns the best goal posts extracted from the image.
   *
   * @return vector<Point>
   */
  vector<vector<Point2f> >
  getBestGoalPosts()
  {
    return bestGoalPosts;
  }

private:
  void
  classifyPosts(vector<ScannedRegionPtr>& verGoalRegions);
  bool
  scanForPosts(vector<ScannedLinePtr>& verGoalLines);
  void
  refreshGoalPosts();
  void
  findBestPosts();
  void
  updateGoalInfo();
  void
  findGoalSide(GoalInfo& goalInfo);
  void
  addGoalPost(const GoalPostPtr& goalPost);
  void
  addGoalObstacle(const GoalPostPtr& goalPost);
  void
  addGoalLandmark(const GoalPostPtr& goalPost);

  //! Best goals extracted.
  vector<vector<Point2f> > bestGoalPosts;

  //! Vector of detected goal posts along with history information
  vector<GoalPostPtr> goalPosts;

  //! Field Extraction module object
  boost::shared_ptr<FieldExtraction> fieldExt;

  //! Robot Extraction module object
  boost::shared_ptr<RobotExtraction> robotExt;

  //! Region Segmentation module object
  boost::shared_ptr<RegionSegmentation> regionSeg;

  vector<int> houghSettings;

  static constexpr float refreshTime = 1.f;
  typedef vector<GoalPostPtr>::iterator GPIter;
};
