/**
 * @file VisionModule/include/FeatureExtraction//LinesExtraction.h
 *
 * This file declares the class for field lines extraction from
 * the image.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 22 Aug 2017
 */

#pragma once

#include <Eigen/Dense>
#include "VisionModule/include/FeatureExtraction/Circle.h"
#include "VisionModule/include/FeatureExtraction/FittedLine.h"
#include "VisionModule/include/FeatureExtraction/DirectEllipseFit.h"
#include "VisionModule/include/FeatureExtraction/FeatureExtraction.h"
#include "VisionModule/include/FeatureExtraction/FieldExtraction.h"
#include "VisionModule/include/FeatureExtraction/GoalExtraction.h"
#include "VisionModule/include/FeatureExtraction/RobotExtraction.h"
#include "VisionModule/include/FeatureExtraction/RANSACEllipseFit.h"

/**
 * @class LineExtraction
 * @brief The class for extracting lines from the input image.
 */
class LinesExtraction : public FeatureExtraction, public DebugBase
{
INIT_DEBUG_BASE_(
  //! Option to send total module time.
  (int, sendTime, 0),
  //! Option to draw hough lines algorithm output on the image.
  (int, drawHoughLines, 0),
  //! Option to draw fitted lines on the image.
  (int, drawFittedLines, 0),
  //! Option to draw paired up lines on the image.
  (int, drawPairedLines, 0),
  //! Option to draw the fitted ellipse on the image.
  (int, drawEllipseFit, 0),
)

public:

  /**
   * Default constructor for this class.
   *
   * @param visionModule: Pointer to parent VisionModule.
   */
  LinesExtraction(VisionModule* visionModule) :
    FeatureExtraction(visionModule), DebugBase("LinesExtraction", this)
  {
    initDebugBase();
    fieldExt = GET_FEATURE_EXT_CLASS(FieldExtraction, FIELD);
    goalExt = GET_FEATURE_EXT_CLASS(GoalExtraction, GOAL);
    regionSeg = GET_FEATURE_EXT_CLASS(RegionSegmentation, REGION_SEGMENTATION);
    robotExt = GET_FEATURE_EXT_CLASS(RobotExtraction, ROBOT);
    int tempSendTime;
    int tempDrawHoughLines;
    int tempDrawFittedLines;
    int tempDrawPairedLines;
    int tempDrawEllipseFit;
    houghSettings.resize(3);
    //RANSACParams rParams;
    GET_CONFIG(
      "VisionDebug",
      (int, LinesExtraction.hltThreshold, houghSettings[0]), (int, LinesExtraction.hltLineLength, houghSettings[1]), (int, LinesExtraction.hltLineGap, houghSettings[2]), (int, LinesExtraction.sendTime, tempSendTime), (int, LinesExtraction.drawHoughLines, tempDrawHoughLines), (int, LinesExtraction.drawFittedLines, tempDrawFittedLines), (int, LinesExtraction.drawEllipseFit, tempDrawEllipseFit),
      //(int, RANSACEllipseFit.n, rParams.n),
      //(int, RANSACEllipseFit.iter, rParams.iter),
      //(int, RANSACEllipseFit.minPoints, rParams.minPoints),
      //(int, RANSACEllipseFit.distThresh, rParams.distThresh),
      )
    SET_DVAR(int, sendTime, tempSendTime);
    SET_DVAR(int, drawHoughLines, tempDrawHoughLines);
    SET_DVAR(int, drawFittedLines, tempDrawFittedLines);
    SET_DVAR(int, drawPairedLines, tempDrawPairedLines);
    SET_DVAR(int, drawEllipseFit, tempDrawEllipseFit);
    //ransacEllipseFit = 
    //  boost::make_shared<RANSACEllipseFit>(rParams, getImageWidth(), getImageHeight());
    //whiteEdges = Mat(Size(getImageWidth(), getImageHeight()), CV_8U, Scalar(0));
    olImage = Mat(Size(1000, 700), CV_8UC3, Scalar(0));
  }

  /**
   * Default destructor for this class
   */
  ~LinesExtraction()
  {
  }

  //! Main image processing routine for this feature extraction class
  void
  processImage();

  static bool
  checkEllipse(Ellipse& e);

  Point2f
  findIntersection(const boost::shared_ptr<FittedLine>& l1,
    const boost::shared_ptr<FittedLine>& l2);
private:
  bool
  scanForEdges(vector<vector<ScannedEdgePtr> >& connectedEdges);
  void
  findConnectedEdges(vector<vector<ScannedEdgePtr> >& connectedEdges,
    vector<ScannedEdgePtr>& scannedEdges, const unsigned& xTol,
    const unsigned& yTol, const bool& verticalScan);
  void
  findLinesFromEdges(vector<vector<ScannedEdgePtr> >& connectedEdges,
    vector<FittedLinePtr>& worldLines);
  void
  filterLines(vector<FittedLinePtr>& worldLines, vector<Point2f>& circlePoints);
  void
  findFeatures(vector<FittedLinePtr>& worldLines);
  void
  computeLandmark(const Point2f& inter, const Point2f& unitToBaseLine,
    const unsigned& type);
  void
  computeCircleLandmark(const Circle& c,
    const vector<FittedLinePtr>& worldLines);
  bool
  findCircle(Circle& circleOutput, vector<Point2f>& circlePoints);
  bool
  findCircleLineIntersection(const Circle& c, const FittedLinePtr& wl,
    vector<Point2f>& intersections);
  void
  addLineLandmarks(vector<FittedLinePtr>& worldLines);
  vector<int> houghSettings;

  //Mat whiteEdges;
  Mat olImage;
  //! Field Extraction module object
  boost::shared_ptr<FieldExtraction> fieldExt;

  //! Goal Extraction module object
  boost::shared_ptr<GoalExtraction> goalExt;

  //! Robot Extraction module object
  boost::shared_ptr<RobotExtraction> robotExt;

  //! Lines Extraction module object
  boost::shared_ptr<RegionSegmentation> regionSeg;

  //! RANSAC Ellipse fit module object
  boost::shared_ptr<RANSACEllipseFit> ransacEllipseFit;

  typedef vector<FittedLinePtr>::iterator FlIter;
};
