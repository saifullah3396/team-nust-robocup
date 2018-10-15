/**
 * @file CameraModule/CameraModule.h
 *
 * This file declares the class CameraModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#pragma once

#include <Eigen/Dense>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/Camera.h"
#include "Utils/include/VisionUtils.h"
#include "Utils/include/FileUtils.h"
#include "VisionModule/include/VisionModule.h"

/**
 * @class CameraModule
 * @brief This BaseModule that updates camera information to local
 *   shared memory
 */
class CameraModule
{
public:
  /**
   * Initializes the camera module
   *
   * @param visionModule pointer to the base class which is in
   *   this case the VisionModule
   */
  CameraModule(VisionModule* visionModule);

  /**
   * Destructor
   */
  ~CameraModule();

#ifdef MODULE_IS_REMOTE
  /**
   * Retrieves image from the camera
   *
   * @param index the camera index
   * @param saveImages whether to save images in respective robot dir
   * @param useLoggedImages whether to use logged images for processing
   * @return void
   */
  void updateImage(
    const unsigned& index,
    const int& saveImages = false,
    const bool& useLoggedImages = false);
#else
  /**
   * Retrieves image from the camera
   *
   * @param index the camera index
   * @param saveImages whether to save images in respective robot dir
   * @return void
   */
  void updateImage(
    const unsigned& index,
    const int& saveImages = false);
#endif
  void 
  recordVideo(const unsigned& index);
  
  void 
  stopRecording();

  /**
   * Releases the image from the camera
   *
   * @param index the camera index
   * @return void
   */
  void
  releaseImage(const unsigned& index);

  /**
   * Gets the pointer to given camera.
   *
   * @return a pointer to required camera.
   */
  CameraPtr
  getCameraPtr(const unsigned& index)
  {
    return cams[index];
  }

  /**
   * Gets a vector of pointers to both cameras.
   *
   * @return a vector of pointers to both cameras.
   */
  vector<CameraPtr>&
  getCameraPtrs()
  {
    return cams;
  }

  static inline void
  bgrToYuv422(uint8_t* out, const Mat& in, const int& width, const int& height)
  {
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        int r = in.at < Vec3b > (y, x)[2];
        int g = in.at < Vec3b > (y, x)[1];
        int b = in.at < Vec3b > (y, x)[0];
        out[x * 2 + y * width * 2] = (uint8_t)(
          0.299 * r + 0.587 * g + 0.114 * b);
        if (x % 2 == 0) {
          out[1 + x * 2 + y * width * 2] = (uint8_t)(
            -0.169 * r - 0.331 * g + 0.499 * b + 128);
        } else {
          out[1 + x * 2 + y * width * 2] = (uint8_t)(
            0.498 * r - 0.419 * g - 0.0813 * b + 128);
        }
      }
    }
  }

  static inline void
  yuv422ToBgr(Mat& out, const uint8_t * const in, const int& width,
    const int& height)
  {
    out = Mat(Size(width, height), CV_8UC3);
    for (int py = 0; py < height; py++) {
      int cbLast = in[(0 + py * width) * 2 + 1] & 255;
      int crLast = in[(0 + py * width) * 2 + 3] & 255;
      for (int px = 0; px < width; px++) {
        int y = in[(px + py * width) * 2] & 255;
        if ((px & 1) == 0) {
          cbLast = in[(px + py * width) * 2 + 1] & 255;
        } else {
          crLast = in[(px + py * width) * 2 + 1] & 255;
        }
        int cb = cbLast;
        int cr = crLast;
        out.at < Vec3b > (py, px)[2] = VisionUtils::clip(
          y + 1.402 * (cr - 128) + 2);
        out.at < Vec3b > (py, px)[1] = VisionUtils::clip(
          y - 0.344 * (cb - 128) - 0.714 * (cr - 128));
        out.at < Vec3b > (py, px)[0] = VisionUtils::clip(
          y + 1.772 * (cb - 128) + 2);
      }
    }
  }

private:
  /**
   * Sets up the camera input streaming configuration and image format
   * configuration
   *
   * @return bool true if camera setup is successful
   */
  bool
  setupCameras();

  //! Pointer to NaoQi video device proxy
  ALVideoDeviceProxyPtr camProxy;

  //! Vector of cameras
  vector<CameraPtr> cams;
  
  //! Opencv video writer objects for top and bottom cams
  vector<VideoWriter*> videoWriter;
};
