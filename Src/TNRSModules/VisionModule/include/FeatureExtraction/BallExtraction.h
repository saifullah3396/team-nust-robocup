/**
 * @file FeatureExtraction/BallExtraction.h
 *
 * This file declares the class for ball extraction from the image.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#pragma once

#include "CommModule/include/CommModule.h"
#include "VisionModule/include/FeatureExtraction/FeatureExtraction.h"
#include "VisionModule/include/FeatureExtraction/FieldExtraction.h"
#include "VisionModule/include/FeatureExtraction/BallTracker.h"

typedef boost::shared_ptr<Rect> RectPtr;

/**
 * @class BallExtraction
 * @brief The class for ball extraction.
 */
class BallExtraction : public FeatureExtraction, public DebugBase
{
INIT_DEBUG_BASE_(
  //! Option to send total module time.
  (int, sendTime, 0),
  //! Option to draw extracted ball contour.
  (int, drawBallContour, 0),
)

public:
  /**
   * Default constructor for this class.
   *
   * @param visionModule: Pointer to parent VisionModule.
   */
  BallExtraction(VisionModule* visionModule) :
    FeatureExtraction(visionModule), DebugBase("BallExtraction", this),
      gridSize(160), ballType(1) // For checkered ball, see config VisionDebug.ini.
  {
    initDebugBase();
    int tempSendTime;
    int tempDrawBallContour;
    GET_CONFIG(
      "EnvProperties",
      (int, ballType, ballType),
      (float, ballRadius, ballRadius),
      (float, coeffSF, coeffSF),
      (float, coeffRF, coeffRF),
    )
    
    GET_CONFIG(
      "VisionDebug",
      (float, BallExtraction.ballRadiusMin, ballRadiusMin),
      (float, BallExtraction.ballRadiusMax, ballRadiusMax),
      (int, BallExtraction.sendTime, tempSendTime), 
      (int, BallExtraction.drawBallContour, tempDrawBallContour), 
      (int, BallExtraction.scanStepLow, scanStepLow), 
      (int, BallExtraction.scanStepHigh, scanStepHigh), 
    )
      
    SET_DVAR(int, sendTime, tempSendTime);
    SET_DVAR(int, drawBallContour, tempDrawBallContour);

    string classifierFile = ConfigManager::getConfigDirPath() + string(
      "/Classifiers/ballClassifier.xml");

    if (!cascade.load(classifierFile)) {
      //CommModule::addToLogMsgQueue("Ball classifier not found.");
#ifdef MODULE_IS_REMOTE
      ERROR("Ball classifier not found.")
#endif
    }
    fieldExt = GET_FEATURE_EXT_CLASS(FieldExtraction, FIELD);
    regionSeg = GET_FEATURE_EXT_CLASS(RegionSegmentation, REGION_SEGMENTATION);
    ballTracker = boost::make_shared < BallTracker > (visionModule);
    PRINT("BallTracker constructed")
    ballTracker->init();
    PRINT("BallTracker initiated")
    regionsDist.resize(NUM_CAMS);
    regionsDist[0] = 50;
    regionsDist[1] = 200;
    int gridDivX = 640 / gridSize;
    int gridDivY = 480 / gridSize;
    for (int x = 0; x < gridDivX; ++x) {
      for (int y = 0; y < gridDivY; ++y) {
        topXYPairs.push_back(pair<int, int>(x, y));
      }
    }
    gridDivX = 160 / (gridSize / 2);
    gridDivY = 120 / (gridSize / 2);
    for (int x = 0; x < gridDivX; ++x) {
      for (int y = 0; y < gridDivY; ++y) {
        bottomXYPairs.push_back(pair<int, int>(x, y));
      }
    }
  }

  /**
   * Default desstructor for this class.
   */
  ~BallExtraction()
  {
  }

  /**
   * Main image processing routine for this feature extraction class.
   */
  void
  processImage();

  /**
   * A function that resets the class members.
   */
  void
  reset()
  {
  }

  /**
   * Sets the image (upper/lower) which is to be processed.  
   */
  void
  setImage();

  /**
   * Returns the state of the ball if it is found or not
   */
  bool
  getBallFound()
  {
    return ballTracker->getBallFound();
  }
private:
  /**
   * Applies the classifier to given image and stores ball bouding box
   * in detectionResult.
   * 
   * @param origRect: The rect defining the cropped region in original
   *   image
   * @param croppedImage: Image under consideration.
   */
  void
  applyClassifier(const Rect& origRect, Mat croppedImage);

  /**
   * Simple ball detector which checks whether the ball is actually 
   * present in the predicted position
   * 
   * @param origRect: The rect defining the cropped region in original
   *   image
   * @param croppedImage: Image under consideration.
   */
  void
  simpleDetect(const Rect& origRect, Mat croppedImage);

  void 
  drawState(const Mat& state, const Scalar& color = Scalar(255,255,255));
  float 
  getCurrentFriction();
  unsigned
  getBallFrameInNextCycle(const BallInfo& ballInfo);
  void
  findBallUpperCam();
  void
  findBallUpperCam(const Rect& roi);
  void
  findBallLowerCam();
  void
  findBallLowerCam(const Rect& roi);
  void
  scanRandom(vector<RectPtr>& boundRects, const vector<int>& pairIndices,
    const int& iters);
  void
  scanRandom(vector<RectPtr>& boundRects, const Rect& roi);
  void
  filterRegions(vector<RectPtr>& boundRects, const float& threshold);
  void
  classifyRegions(vector<RectPtr>& boundRects);
  void
  simBallDetector(const Rect& origRect);
  void
  getPredRoi(Rect& predRoi, const Mat& predState);

  /** 
   * Updates ballInfo variable in the memory.
   */
  void
  updateBallInfo();

  //! Balls found in current iteration
  vector<Rect> foundBall;

  //! Ball radius in xyz frame
  float ballRadius;

  //! Upper radius threshold.
  float ballRadiusMax;

  //! Lower radius threshold.
  float ballRadiusMin;

  //! OpenCv Cascade classifier for ball.
  CascadeClassifier cascade;

  //! Field Extraction module object.
  boost::shared_ptr<FieldExtraction> fieldExt;

  //! Field Extraction module object.
  boost::shared_ptr<RegionSegmentation> regionSeg;

  //! Ball tracker class object.
  boost::shared_ptr<BallTracker> ballTracker;

  //! Distance threshold for combining two regions
  vector<int> regionsDist;

  //! Ball scan step for lower cam
  int scanStepLow;
  int scanStepHigh;

  //! Type of the ball
  unsigned ballType;
  
  //! Friction coefficients
  float coeffSF;
  float coeffRF;

  vector<pair<int, int> > topXYPairs;
  vector<pair<int, int> > bottomXYPairs;
  vector<int> xySeen;
  int gridSize;
};
