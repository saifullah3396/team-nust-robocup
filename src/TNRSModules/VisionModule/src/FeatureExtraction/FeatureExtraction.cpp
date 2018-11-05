/**
 * @file FeatureExtraction/FeatureExtraction.cpp
 *
 * This file implements the base class for feature extraction
 * from the image. All the functions and algorithms for detecting
 * features from the image will be defined under this module.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 22 Aug 2017
 */

#include "VisionModule/include/FeatureExtraction/FeatureExtraction.h"

Rect FeatureExtraction::footArea = Rect();
vector<int> FeatureExtraction::imageWidth = vector<int>(NUM_CAMS);
vector<int> FeatureExtraction::imageHeight = vector<int>(NUM_CAMS);
vector<uint8_t*> FeatureExtraction::image = vector<uint8_t*>(NUM_CAMS);
vector<Mat> FeatureExtraction::grayImage = vector < Mat > (NUM_CAMS);
vector<Mat> FeatureExtraction::imageMat = vector < Mat > (NUM_CAMS);
vector<Mat> FeatureExtraction::bgrMat = vector < Mat > (NUM_CAMS);
Colors FeatureExtraction::ourColor = Colors::WHITE;
Colors FeatureExtraction::oppColor = Colors::WHITE;
bool FeatureExtraction::blackJerseyExists = false;
vector<CameraPtr> FeatureExtraction::cams = vector < CameraPtr > (NUM_CAMS);
CameraModulePtr FeatureExtraction::camModule;
CameraTransformPtr FeatureExtraction::cameraTransform;
ColorHandlerPtr FeatureExtraction::colorHandler;
vector<KnownLandmarkPtr> FeatureExtraction::knownLandmarks;
vector<UnknownLandmarkPtr> FeatureExtraction::unknownLandmarks;

FeatureExtraction::FeatureExtraction(VisionModule* visionModule) :
  MemoryBase(visionModule), visionModule(visionModule), currentImage(TOP_CAM)
{
  cycleTime = visionModule->getPeriodMinMS() / ((float) 1000);
}

FeatureExtraction::~FeatureExtraction()
{
  for (int i = 0; i < NUM_CAMS; ++i)
    delete image[i];
}

void
FeatureExtraction::setup(VisionModule* visionModule)
{
  camModule = visionModule->getCameraModule();
  cameraTransform = visionModule->getCameraTransform();
  colorHandler = visionModule->getColorHandler();
  cams = camModule->getCameraPtrs();
  for (int i = 0; i < NUM_CAMS; ++i) {
    imageWidth[i] = cams[i]->width;
    imageHeight[i] = cams[i]->height;
    cams[i]->getImagePtr(image[i]);
    grayImage[i] = Mat(Size(imageWidth[i], imageHeight[i]), CV_8U);
    imageMat[i] = Mat(Size(imageWidth[i], imageHeight[i]), CV_8UC2);
    bgrMat[i] = Mat(Size(imageWidth[i], imageHeight[i]), CV_8UC3);
  }
  // For lower camera only.
  // As defined in 
  // https://pdfs.semanticscholar.org/3742/0e2c3fc50e89e14008e7ce584ddce52cce06.pdf
  footArea.y = 0.625 * imageHeight[1];
  footArea.height = (1 - 0.625) * imageHeight[1];
  footArea.x = 0.125 * imageWidth[1] / 2;
  // Because yuv422 when converted to uv image has half the width 
  footArea.width = (0.875 - 0.125) * imageWidth[1] / 2;
}

void
FeatureExtraction::setupImagesAndHists()
{
  for (int i = 0; i < NUM_CAMS; ++i) {
    imageMat[i].data = image[i];
#ifdef MODULE_IS_REMOTE
    cvtColor(imageMat[i], bgrMat[i], COLOR_YUV2BGR_YUY2);
#endif
    if (i == BOTTOM_CAM) {
      Mat yuv422Ch[2];
      split(imageMat[i], yuv422Ch);
      // Set lower cam gray-scale image
      grayImage[i] = yuv422Ch[0];
      // Prepare to get lower cam histograms
      Mat roi = yuv422Ch[1].reshape(2)(footArea);
      Mat binary;
      inRange(roi, Scalar(124, 124), // Lower range for white without a doubt
      Scalar(132, 132), // Upper range for white without a doubt
      binary);
      bitwise_not(binary, binary);
      // Inverse binary to mask the image for histogram calculations
      colorHandler->computeUVHist(roi, binary, false);
      //Mat shaped = yuv422Ch[1].reshape(2);
      //Mat uvCh[2];
      //split(shaped, uvCh);
      //Mat u = Mat(Size(imageWidth[i], imageHeight[i]), CV_8UC1);
      //Mat v = Mat(Size(imageWidth[i], imageHeight[i]), CV_8UC1);
      //resize(uvCh[0], u, u.size());
      //resize(uvCh[1], v, v.size());
      //imshow("uvCh[0]", uvCh[0]);
      //imshow("uvCh[1]", uvCh[1]);
      //imshow("v", u);
      //imshow("u", v);
      //std::vector<cv::Mat> array_to_merge;
      //array_to_merge.push_back(grayImage[i]);
      //array_to_merge.push_back(u);
      //array_to_merge.push_back(v);
      //cv::Mat color
      //cv::merge(array_to_merge, color);
      //imshow("color", color);
      //waitKey(0);
      //cvtColor(bgrMat[i], color, COLOR_RGB2YUV);
      //imshow("color2", color);
      //waitKey(0);
    } else {
      cvtColor(imageMat[i], grayImage[i], COLOR_YUV2GRAY_YUY2);
    }
  }
}

void
FeatureExtraction::updateColorInfo(const Colors& ourColor,
  const Colors& oppColor, const bool& blackJerseyExists)
{
  FeatureExtraction::ourColor = ourColor;
  FeatureExtraction::oppColor = oppColor;
  FeatureExtraction::blackJerseyExists = blackJerseyExists;
}
