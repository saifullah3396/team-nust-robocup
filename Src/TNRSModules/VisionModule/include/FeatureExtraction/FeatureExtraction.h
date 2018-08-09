/**
 * @file FeatureExtraction/FeatureExtraction.h
 *
 * This file declares the base class for feature extraction
 * from the image. All the functions and algorithms for detecting
 * features from the image will be defined under this module.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 22 Aug 2017
 */

#pragma once

#include <boost/circular_buffer.hpp>
#include <chrono>
#include "CommModule/include/CommRequest.h"
#include "VisionModule/include/FeatureExtraction/ScannedCurve.h"
#include "VisionModule/include/FeatureExtraction/ScannedEdge.h"
#include "VisionModule/include/FeatureExtraction/ScannedLine.h"
#include "VisionModule/include/FeatureExtraction/ScannedRegion.h"
#include "TNRSBase/include/MemoryBase.h"
#include "LocalizationModule/include/FieldLandmarkIds.h"
#include "VisionModule/include/CameraModule/CameraModule.h"
#include "VisionModule/include/CameraTransform.h"
#include "VisionModule/include/ColorHandler.h"
#include "VisionModule/include/VisionModule.h"
#include "Utils/include/Landmark.h"
#include "Utils/include/Obstacle.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/VisionUtils.h"

template<typename T>
  Point_<T>
  findIntersection(const Vec<T, 4>& l1, const Vec<T, 4>& l2)
  {
    auto p1 = Point_ < T > (l1[0], l1[1]);
    auto p2 = Point_ < T > (l1[2], l1[3]);
    auto p3 = Point_ < T > (l2[0], l2[1]);
    auto p4 = Point_ < T > (l2[2], l2[3]);
    auto p1p2 = p1 - p2;
    auto p3p4 = p3 - p4;
    auto det = p1p2.x * p3p4.y - p1p2.y * p3p4.x;
    auto c1 = p1.x * p2.y - p1.y * p2.x;
    auto c2 = p3.x * p4.y - p3.y * p4.x;
    if (det != 0.0) {
      //! det == 0 -> Lines are parallel
      auto pX = (c1 * p3p4.x - p1p2.x * c2) / det;
      auto pY = (c1 * p3p4.y - p1p2.y * c2) / det;
      return Point_ < T > (pX, pY);
    } else {
      return Point_ < T > (-1, -1);
    }
  }

/*bool isStraight(boost::shared_ptr<ScannedCurve> sc) {
 auto last = sc->upper.back();
 float dX = (last.x - sc->upper[0].x);
 float dY = (last.y - sc->upper[0].y);
 float m = dY / dX;
 float diff = 0;
 cout << "start" << endl;
 cout << "slope: " << m << endl;
 for (int i = 1; i < sc->upper.size() - 1; ++i)
 {
 float idX = last.x - sc->upper[i].x;
 float idY = last.y - sc->upper[i].y;
 cout << "idX: " << abs(idX) << endl;
 if(abs(idX) < 0.001)
 continue;
 float iM = idY / idX;
 diff = diff + abs((iM - m) / m);
 cout << "diffcurr: " << abs((iM - m) / m) << endl;
 }
 diff /= (sc->upper.size() - 1);
 cout << "diff: " << diff << endl;
 if(diff < 0.15)
 return true;
 return false;
 }*/

using namespace std::chrono;
using namespace Utils;

#define GET_FEATURE_EXT_CLASS(name, index) \
	boost::static_pointer_cast<name>(visionModule->getFeatureExtClass(index));

class MatGenerator : public ParallelLoopBody
{
public:
  MatGenerator(Mat& image, const uint8_t* in, const int& imageWidth,
    const int& imageHeight) :
    image(image), in(in), imageWidth(imageWidth), imageHeight(imageHeight)
  {
  }

  virtual void
  operator ()(const Range& range) const
  {
    for (int r = range.start; r < range.end; r++) {
      int i = r / (imageWidth * 3);
      int j = r % (imageWidth * 3);
      if (j % 3 == 0) image.ptr < uchar > (i)[j] =
        in[(j / 3 + i * imageWidth) << 1];
      else if (j % 3 == 1) image.ptr < uchar > (i)[j] =
        in[(((j / 3 + i * imageWidth) >> 1) << 2) + 1];
      else if (j % 3 == 2) image.ptr < uchar > (i)[j] =
        in[((j / 3 + i * imageWidth) << 1) | 3];
    }
  }

  MatGenerator&
  operator=(const MatGenerator&)
  {
    return *this;
  }
  ;

private:
  Mat& image;
  const uint8_t* in;
  int imageWidth;
  int imageHeight;
};

class FeatureExtraction : public MemoryBase
{
public:

  /**
   * Default constructor for this class.
   *
   * @param visionModule: Pointer to parent VisionModule
   */
  FeatureExtraction(VisionModule* visionModule);

  /**
   * Default destructor for this class
   */
  ~FeatureExtraction();

  /**
   * Processes the input image and extracts the required features
   */
  virtual void
  processImage() = 0;

  //! Sets up the initial image containers
  static void
  setup(VisionModule* visionModule);

  //! Sets up the gray scale images from input images
  static void
  setupImagesAndHists();

  //! Sets the camera index for the given class
  void
  setCurrentImage(const unsigned& imageIndex);

  //! Sets the camera index for the given class
  unsigned&
  getCurrentImage()
  {
    return currentImage;
  }

  //! Gets the bgr image matrix
  static Mat&
  getBgrMat(const int& index)
  {
    return bgrMat[index];
  }

  static void
  updateColorInfo(const Colors& ourColor, const Colors& oppColor,
    const bool& blackJerseyExists);

  static vector<KnownLandmarkPtr>
  getKnownLandmarks() {
    return knownLandmarks;
  }
  
  static vector<UnknownLandmarkPtr>
  getUnknownLandmarks() {
    return unknownLandmarks;
  }

  static void
  clearLandmarks() {
    knownLandmarks.clear();
    unknownLandmarks.clear();
  }

protected:
  inline Mat&
  getGrayImage()
  {
    return grayImage[currentImage];
  }

  inline int&
  getImageWidth()
  {
    return imageWidth[currentImage];
  }

  inline int&
  getImageHeight()
  {
    return imageHeight[currentImage];
  }

  /**
   * Gets the y component at the required y-x index of the image.
   */
  inline uint8_t
  getY(const int32_t& x, const int32_t& y)
  {
    return image[currentImage][(x + y * getImageWidth()) << 1];
  }

  /**
   * Gets the u component at the required x-y index of the image.
   */
  inline uint8_t
  getU(const int32_t& x, const int32_t& y)
  {
    return image[currentImage][(((x + y * getImageWidth()) >> 1) << 2) + 1];
  }

  /**
   * Gets the v component at the required x-y index of the image.
   */
  inline uint8_t
  getV(const int32_t& x, const int32_t& y)
  {
    return image[currentImage][((x + y * getImageWidth()) << 1) | 3];
  }

  /**
   * Gets the yuv color at the required x-y index of the image.
   */
  inline TNColor
  getYUV(const int32_t& x, const int32_t& y)
  {
    return TNColor(
      (int) image[currentImage][(x + y * getImageWidth()) << 1],
      (int) image[currentImage][(((x + y * getImageWidth()) >> 1) << 2) + 1],
      (int) image[currentImage][((x + y * getImageWidth()) << 1) | 3]);
  }

  /**
   * Gets the yuv color at the required x-y index of the image.
   */
  static TNColor
  getYUV(const int& index, const int32_t& x, const int32_t& y)
  {
    return TNColor(
      (int) image[index][(x + y * imageWidth[index]) << 1],
      (int) image[index][(((x + y * imageWidth[index]) >> 1) << 2) + 1],
      (int) image[index][((x + y * imageWidth[index]) << 1) | 3]);
  }

  inline Mat
  makeYuvMat()
  {
    Mat yuv = Mat(Size(getImageWidth(), getImageHeight()), CV_8UC3);
    uchar* p;
    for (int y = 0; y < getImageHeight(); ++y) {
      for (int x = 0; x < getImageWidth() * 3; x = x + 3) {
        p = yuv.ptr < uchar > (y);
        auto yuvp = getYUV(x / 3, y);
        p[x] = yuvp.y;
        p[x + 1] = yuvp.u;
        p[x + 2] = yuvp.v;
      }
    }
    return yuv;
  }

  static void
  computeUVHist(const Mat& uv422, const bool& drawHists);

  static inline void
  drawRegion(Mat& img, const ScannedRegionPtr& region, const Scalar& color =
    Scalar(0, 0, 0))
  {
    rectangle(img, region->rect, color, 1);
  }

  static inline void
  drawRegions(Mat& img, const vector<ScannedRegionPtr>& regions,
    const Scalar& color = Scalar(0, 0, 0))
  {
    for (int i = 0; i < regions.size(); ++i) {
      if (regions[i]) rectangle(img, regions[i]->rect, color, 1);
      //imshow("img", img);
      //waitKey(0);
    }
  }

  static inline void
  findRegions(vector<ScannedRegionPtr>& regions,
    vector<ScannedLinePtr>& scannedLines, const unsigned& xTol,
    const unsigned& yTol, const bool& horizontal)
  {
    typedef vector<ScannedLinePtr>::iterator slIter;
    slIter iter = scannedLines.begin();
    while (iter != scannedLines.end()) {
      if (*iter) ++iter;
      else iter = scannedLines.erase(iter);
    }

    sort(
      scannedLines.begin(),
      scannedLines.end(),
      [](const ScannedLinePtr& sl1, const ScannedLinePtr& sl2)
      { return sl1->p1.x < sl2->p1.x;});

    ScannedLinePtr pred;
    for (int i = 0; i < scannedLines.size(); ++i) {
      ScannedLinePtr sl = scannedLines[i];
      sl->closestDist = 1000;
      sl->bestNeighbor.reset();
      if (pred) {
        sl->pred = pred;
      }
      pred = sl;
    }

    int maxDist = sqrt(yTol * yTol + xTol * xTol);
    for (int i = 0; i < scannedLines.size(); ++i) {
      ScannedLinePtr sl = scannedLines[i];
      ScannedLinePtr neighbor = sl->pred;
      while (neighbor) {
        int diffY;
        if (horizontal) {
          diffY = abs(neighbor->p2.y - sl->p2.y);
        } else {
          int diff1 = abs(neighbor->p1.y - sl->p1.y);
          int diff2 = abs(neighbor->p2.y - sl->p2.y);
          diffY = diff1 < diff2 ? diff1 : diff2;
        }
        if (diffY < yTol) {
          int diffX;
          if (horizontal) {
            int diff1 = abs(neighbor->p1.x - sl->p1.x);
            int diff2 = abs(neighbor->p2.x - sl->p2.x);
            diffX = diff1 < diff2 ? diff1 : diff2;
          } else {
            diffX = neighbor->p2.x - sl->p2.x;
          }
          int dist = sqrt(diffX * diffX + diffY * diffY);
          if (dist < maxDist) {
            if (dist < sl->closestDist) {
              sl->closestDist = dist;
              sl->bestNeighbor = neighbor;
            }
          }
        }
        neighbor = neighbor->pred;
      }
    }
    reverse(scannedLines.begin(), scannedLines.end());
    vector<ScannedLinePtr> chainParents;
    for (int i = 0; i < scannedLines.size(); ++i) {
      ScannedLinePtr sl = scannedLines[i];
      if (sl->searched) continue;
      chainParents.push_back(sl);
      ScannedLinePtr neighbor = sl->bestNeighbor;
      int chain = 0;
      chain++;
      while (neighbor) {
        if (neighbor->searched) break;
        neighbor->searched = true;
        neighbor = neighbor->bestNeighbor;
        ++chain;
      }
      sl->chainLength = chain;
      sl->searched = true;
    }
    sort(
      chainParents.begin(),
      chainParents.end(),
      [](const ScannedLinePtr& sl1, const ScannedLinePtr& sl2)
      { return sl1->chainLength > sl2->chainLength;});

    for (int i = 0; i < chainParents.size(); ++i) {
      ScannedLinePtr sl = chainParents[i];
      if (sl->chainLength > 0) {
        auto sc = boost::make_shared<ScannedCurve>();
        sc->upper.push_back(sl->p1);
        sc->lower.push_back(sl->p2);
        ScannedLinePtr neighbor = sl->bestNeighbor;
        while (neighbor != 0) {
          sc->upper.push_back(neighbor->p1);
          sc->lower.push_back(neighbor->p2);
          neighbor = neighbor->bestNeighbor;
        }
        reverse(sc->lower.begin(), sc->lower.end());
        sc->upper.insert(sc->upper.end(), sc->lower.begin(), sc->lower.end());
        //VisionUtils::drawPoints(sc->upper, bgrMat[0]);
        //VisionUtils::displayImage(bgrMat[0], "AS");
        //waitKey(0);
        ScannedRegionPtr sr = boost::make_shared < ScannedRegion > (sc->upper);
        regions.push_back(sr);
      }
    }
  }

  /**
   * Vector of input image data pointers
   *
   * @var vector<uint8_t*>
   */
  static vector<uint8_t*> image;

  /**
   * Vector of gray scale image matrix of input images
   *
   * @var vector<Mat>
   */
  static vector<Mat> grayImage;

  /**
   * Vector of yuv422 image matrix of input images
   *
   * @var vector<Mat>
   */
  static vector<Mat> imageMat;

  /**
   * Vector of bgr image matrix of input images
   *
   * @var vector<Mat>
   */
  static vector<Mat> bgrMat;

  /**
   * Vector of input image widths
   *
   * @var vector<int>
   */
  static vector<int> imageWidth;

  /**
   * Vector of input image heights
   *
   * @var vector<int>
   */
  static vector<int> imageHeight;

  //! Vector for storing known landmarks observation
  static vector<KnownLandmarkPtr> knownLandmarks;
  
  //! Vector for storing unknown landmarks observation
  static vector<UnknownLandmarkPtr> unknownLandmarks;
  
  /**
   * Current image index
   */
  unsigned currentImage;

  //! Feature extraction iteration start time
  clock_t iterationStartTime;

  //! Rect defining the lower cam foot area for the robot.
  static Rect footArea;

  //! Pointer to color handler class.
  static ColorHandlerPtr colorHandler;

  //! Pointer to image transform class.
  static CameraTransformPtr cameraTransform;

  //! Our team color
  static Colors ourColor;

  //! Opponent team color
  static Colors oppColor;

  //! If a black team exists in game
  static bool blackJerseyExists;

  //! Cycle time of vision module
  float cycleTime;

  //! VisionModule pointer
  VisionModule* visionModule;
private:
  //! Pointer to camera module class.
  static CameraModulePtr camModule;

  //! Pointers to camera objects.
  static vector<CameraPtr> cams;
};

typedef boost::shared_ptr<FeatureExtraction> FeatureExtractionPtr;
