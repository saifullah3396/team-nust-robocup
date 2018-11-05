/**
 * @file VisionModule/ImagePreprocessor.h
 *
 * This file declares a class for performing various image preprocessing
 * steps such as noise removal, color conversion, filteration, which
 * is the same for all classes.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#pragma once

#include "TNRSBase/include/MemoryBase.h"
#include "VisionModule/include/VisionModule.h"
#include "Utils/include/VisionUtils.h"

/*class MatGenerator : public ParallelLoopBody
 {
 public:
 MatGenerator(const Mat &image) : image(image)
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

 MatGenerator& operator=(const MatGenerator&) {
 return *this;
 };

 private:
 Mat &image;
 };*/

/**
 * @class ImagePreprocessor
 * @brief This file declares a class for performing various image
 *   preprocessing steps such as noise removal, color conversion,
 *   filteration, which is the same for all classes.
 */
class ImagePreprocessor
{
public:
  /**
   * The default constructor for image processing class.
   */
  ImagePreprocessor(VisionModule* visionModule) :
    camModule(visionModule->getCameraModule())
  {
    cams = camModule->getCameraPtrs();
  }

  /**
   * The default destructor for image processing class.
   */
  ~ImagePreprocessor()
  {
  }

  /**
   * The main processing function.
   */
  void
  update()
  {
#ifdef MODULE_IS_REMOTE
    //for (int i = 0; i < NUM_CAMS; ++i) {
    //  cvtColor(cams[i]->image, cams[i]->image, COLOR_BGR2YUV);
    //}
#endif
  }

private:
  //! CameraModule ptr
  CameraModulePtr camModule;

  //! Vector of cameras.
  vector<CameraPtr> cams;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<ImagePreprocessor> ImagePreprocessorPtr;

