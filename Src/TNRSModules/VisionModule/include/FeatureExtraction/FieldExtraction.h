/**
 * @file VisionModule/include/FeatureExtraction/FieldExtraction.h
 *
 * This file declares the class for field extraction from the image.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author AbdulRehman
 * @date 22 Aug 2017
 */

#pragma once

#include "VisionModule/include/FeatureExtraction/FeatureExtraction.h"
#include "VisionModule/include/FeatureExtraction/RegionSegmentation.h" 
#include "VisionModule/include/FeatureExtraction/FittedLine.h"
#include "Utils/include/DataUtils.h"
#include "VisionModule/include/ColorHandler.h"

/**
 * @class FieldExtraction
 * @brief The class for extracting field from the input image.
 */
class FieldExtraction : public FeatureExtraction, public DebugBase
{
INIT_DEBUG_BASE_(
  //! Option to send total module time.
  (int, sendTime, 0),
  //! Option to draw border on the output image.
  (int, drawBorder, 0),
  //! Option to draw border lines on the output image.
  (int, drawBorderLines, 0),
)

public:
  /**
   * Default constructor for this class.
   *
   * @param visionModule: Pointer to parent VisionModule.
   */
  FieldExtraction(VisionModule* visionModule) :
    FeatureExtraction(visionModule), DebugBase("FieldExtraction", this)
  {
    initDebugBase();
    int tempSendTime;
    int tempDrawBorder;
    int tempDrawBorderLines;
    GET_CONFIG(
      "VisionDebug",
      (int, FieldExtraction.otherPixelThreshold, otherPixelThreshold), (int, FieldExtraction.otherCountThreshold, otherCountThreshold), (int, FieldExtraction.sendTime, tempSendTime), (int, FieldExtraction.drawBorder, tempDrawBorder), (int, FieldExtraction.drawBorderLines, tempDrawBorderLines), )
    SET_DVAR(int, sendTime, tempSendTime);
    SET_DVAR(int, drawBorder, tempDrawBorder);
    SET_DVAR(int, drawBorderLines, tempDrawBorderLines);
    regionSeg = GET_FEATURE_EXT_CLASS(RegionSegmentation, REGION_SEGMENTATION);
    fieldHeight = 0;
    fieldFound = false;
    mask = Mat(Size(getImageWidth(), getImageHeight()), CV_8UC1, Scalar(0));
  }

  /**
   * Default destructor for this class.
   */
  ~FieldExtraction()
  {
  }

  /**
   * Returns the vector of extracted field border points.
   *
   * @return vector<Point>
   */
  vector<Point>&
  getBorder()
  {
    return border;
  }

  /**
   * Returns the extracted field border lines.
   *
   * @return vector<Vec4i>
   */
  vector<Vec4f>&
  getBorderLines()
  {
    return borderLines;
  }

  /**
   * Returns the extracted field border lines in world.
   *
   * @return vector<FittedLinePtr>
   */
  vector<FittedLinePtr>&
  getBorderLinesWorld()
  {
    return borderLinesWorld;
  }

  /**
   * Returns the minimum field height.
   *
   * @return int
   */
  int&
  getFieldHeight()
  {
    return fieldHeight;
  }

  /**
   * Returns the mask of the extracted field.
   *
   * @return Mat
   */
  Mat&
  getMask()
  {
    return mask;
  }

  /**
   * Returns the rectangle of the extracted field.
   *
   * @return Rect
   */
  Rect&
  getFieldRect()
  {
    return fieldRect;
  }

  bool&
  isFound()
  {
    return fieldFound;
  }

  //! Main image processing routine for this feature extraction class.
  void
  processImage();

private:
  //! Threshold for removing pixels other than green
  int otherPixelThreshold;

  //! This threshold defines the amount of pixels that if present in
  //! one column consequently will reset the max height of the field.
  int otherCountThreshold;

  //! An offset for manually increasing the field border points heights.
  int upperOffset;

  //! Field minimum height
  int fieldHeight;

  //! Whether the field is found or not
  bool fieldFound;

  //! Field border.
  vector<Point> border;

  //! Field border lines in image.
  vector<Vec4f> borderLines;

  //! Field border lines in world.
  vector<FittedLinePtr> borderLinesWorld;

  Rect fieldRect;
  Mat mask;

  boost::shared_ptr<RegionSegmentation> regionSeg;
};
