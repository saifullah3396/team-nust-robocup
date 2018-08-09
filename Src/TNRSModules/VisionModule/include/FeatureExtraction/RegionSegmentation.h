/**
 * @file FeatureExtraction/RegionSegmentation.h
 *
 * This file declares the class for segmenting the image into different
 * regions for further processing.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author AbdulRehman
 * @date 5 Mar 2018
 */

#pragma once

#include "VisionModule/include/FeatureExtraction/FeatureExtraction.h"
#include "VisionModule/include/ColorHandler.h"

/*class ScanHorizontal : public ParallelLoopBody
 {
 public:
 ScanHorizontal(const Mat &img) : image(image)
 {
 }

 virtual void operator ()(const Range& range) const
 {
 for (int r = range.start; r < range.end; r++)
 {
 int i = r / image.cols;
 int j = r % image.cols;
 image.ptr<uchar>(i)[j] = 128;
 }
 }

 ScanHorizontal& operator=(const ScanHorizontal&) {
 return *this;
 };

 private:
 Mat &image;
 };*/

class FieldExtraction;
class BallExtraction;
class GoalExtraction;

/**
 * @class RegionSegmentation
 * @brief The class for extracting field from the input image.
 */
class RegionSegmentation : public FeatureExtraction, public DebugBase
{
INIT_DEBUG_BASE_(
  //! Option to send total module time.
  (int, sendTime, 0),
  //! Option to draw regions on incoming image.
  (int, drawRegions, 0),
)

public:
  /**
   * Constructor
   *
   * @param visionModule: pointer to parent VisionModule
   */
  RegionSegmentation(VisionModule* visionModule);

  /**
   * Destructor
   */
  ~RegionSegmentation()
  {
  }

  //! Derived from FeatureExtraction
  void
  processImage();

  void
  setFieldExtraction(const boost::shared_ptr<FieldExtraction>& fieldExt)
  {
    this->fieldExt = fieldExt;
  }

  void
  setBallExtraction(const boost::shared_ptr<BallExtraction>& ballExt)
  {
    this->ballExt = ballExt;
  }

  void
  setGoalExtraction(const boost::shared_ptr<GoalExtraction>& goalExt)
  {
    this->goalExt = goalExt;
  }

  vector<Point>&
  getBorderPoints()
  {
    return borderPoints;
  }
  int&
  getFieldAvgHeight()
  {
    return avgHeight;
  }
  int&
  getFieldMinBestHeight()
  {
    return minBestHeight;
  }

  //vector<ScannedLinePtr>& getVerGoalLines() { return verGoalLines; } 
  //vector<ScannedLinePtr>& getVerBallLines() { return verBallLines; } 
  //vector<ScannedLinePtr>& getHorBallLines() { return horBallLines; }
  vector<ScannedLinePtr>&
  getVerRobotLines()
  {
    return verRobotLines;
  }
  vector<ScannedLinePtr>&
  getHorRobotLines()
  {
    return horRobotLines;
  }
  vector<ScannedLinePtr>&
  getVerJerseyLinesOurs()
  {
    return verJerseyLinesOurs;
  }
  vector<ScannedLinePtr>&
  getHorJerseyLinesOurs()
  {
    return horJerseyLinesOurs;
  }
  vector<ScannedLinePtr>&
  getVerJerseyLinesOpps()
  {
    return verJerseyLinesOpps;
  }
  vector<ScannedLinePtr>&
  getHorJerseyLinesOpps()
  {
    return horJerseyLinesOpps;
  }

private:
  int scanStepLow;
  int scanStepHigh;
  int vScanLimitIdx;
  int hScanLimitIdx;
  duration<double> timeSpan;

  vector<ScannedLinePtr> horRobotLines;
  //vector<ScannedLinePtr> horBallLines;
  vector<ScannedLinePtr> horJerseyLinesOurs;
  vector<ScannedLinePtr> horJerseyLinesOpps;

  //vector<ScannedLinePtr> verGoalLines;
  vector<ScannedLinePtr> verRobotLines;
  //vector<ScannedLinePtr> verBallLines;
  vector<ScannedLinePtr> verJerseyLinesOurs;
  vector<ScannedLinePtr> verJerseyLinesOpps;

  vector<Point> borderPoints;
  int avgHeight;
  int minBestHeight;

  //! Field Extraction module object
  boost::shared_ptr<FieldExtraction> fieldExt;

  //! Ball Extraction module object
  boost::shared_ptr<BallExtraction> ballExt;

  //! Goal Extraction module object
  boost::shared_ptr<GoalExtraction> goalExt;
};
