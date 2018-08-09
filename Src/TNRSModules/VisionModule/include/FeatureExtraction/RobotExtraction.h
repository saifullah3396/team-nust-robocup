/**
 * @file VisionModule/include/FeatureExtraction/RobotExtraction.h
 *
 * This file declares the class for field corners extraction from
 * the image.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 24 Aug 2017
 */

#pragma once

#include "VisionModule/include/FeatureExtraction/FeatureExtraction.h"
#include "VisionModule/include/FeatureExtraction/FieldExtraction.h"
#include "Utils/include/ObstacleType.h"

/**
 * @struct RobotRegion
 * @brief Holds information about the region that defines a robot.
 */
struct RobotRegion
{
  RobotRegion(const boost::shared_ptr<ScannedRegion>& sr, const bool& ourTeam) :
    ourTeam(ourTeam), sr(sr), refresh(true), fallen(false),
      posFromJersey(false), minDistValidation(0.3f)
  {
  }

  Point2f world;
  bool refresh;
  float timeDetected;
  bool ourTeam;
  bool fallen;
  bool posFromJersey;
  boost::shared_ptr<ScannedRegion> sr;
  float minDistValidation; // 30 cms
};
typedef boost::shared_ptr<RobotRegion> RobotRegionPtr;

/**
 * @class RobotExtraction
 * @brief The class for extracting robots from the input image.
 */
class RobotExtraction : public FeatureExtraction, public DebugBase
{
INIT_DEBUG_BASE_(
  //! Option to send total module time.
  (int, sendTime, 0),
  //! Option to draw the bounding boxes for extracted robots.
  (int, drawRobotBoxes, 0),
  //! Option to draw the bounding boxes for extracted robots.
  (int, drawJerseyWithColor, 0),
)

public:

  /**
   * Default constructor for this class.
   *
   * @param visionModule: Pointer to parent VisionModule.
   */
  RobotExtraction(VisionModule* visionModule) :
    FeatureExtraction(visionModule), DebugBase("RobotExtraction", this)
  {
    initDebugBase();
    fieldExt = GET_FEATURE_EXT_CLASS(FieldExtraction, FIELD);
    regionSeg = GET_FEATURE_EXT_CLASS(RegionSegmentation, REGION_SEGMENTATION);
    int tempSendTime;
    int tempDrawRobotBoxes;
    int tempDrawJerseyWithColor;
    GET_CONFIG(
      "VisionDebug",
      (int, RobotExtraction.sendTime, tempSendTime), (int, RobotExtraction.drawRobotBoxes, tempDrawRobotBoxes), (int, RobotExtraction.drawJerseyWithColor, tempDrawJerseyWithColor), )
    SET_DVAR(int, sendTime, tempSendTime);
    SET_DVAR(int, drawRobotBoxes, tempDrawRobotBoxes);
    SET_DVAR(int, drawJerseyWithColor, tempDrawJerseyWithColor);

    string classifierFile = ConfigManager::getConfigDirPath() + string(
      "/Classifiers/topRobot.xml");

    if (!classifier.load(classifierFile)) {
      //CommModule::addToLogMsgQueue("Robot classifier not found.");
#ifdef MODULE_IS_REMOTE
      ERROR("Robot classifier not found.")
#endif
    }
  }

  /**
   * Default destructor for this class.
   */
  ~RobotExtraction()
  {
  }

  //! Main image processing routine for this feature extraction class.
  void
  processImage();

  vector<RobotRegionPtr>&
  getRobotRegions()
  {
    return outputRegions;
  }
  vector<Rect>&
  getStrayRegions()
  {
    return strayRegions;
  }

private:
  void
  updateRobotsInfo();
  void
  refreshRobotRegions();
  void
  findRobotRegions();
  void
  findStrayRegions();
  void
  classifyRobots();
  void
  filterLinesBelowField();

  void
  linkRegions(vector<ScannedRegionPtr>& resultRegions,
    vector<ScannedRegionPtr>& regions, const unsigned& xTol,
    const unsigned& yTol);

  void
  filterJerseys(const vector<Point>& border,
    vector<ScannedLinePtr>& jerseyLines);

  void
  filterRegions(vector<ScannedRegionPtr>& regions, const unsigned& threshold);

  vector<RobotRegionPtr> robotRegions;
  vector<RobotRegionPtr> outputRegions;
  vector<ScannedLinePtr> verRobotLines;
  vector<ScannedLinePtr> horRobotLines;
  vector<Point> border;

  //! Vector of regions that are not part of lines.
  vector<Rect> strayRegions;

  //! OpenCv Cascade classifier for robots.
  CascadeClassifier classifier;

  //! Field Extraction module object.
  boost::shared_ptr<FieldExtraction> fieldExt;

  //! Lines Extraction module object.
  boost::shared_ptr<RegionSegmentation> regionSeg;

  typedef vector<RobotRegionPtr>::iterator RRIter;static constexpr float refreshTime = 1.f;
};
