/**
 * @file Utils/includes/Camera.h
 *
 * This file defines the struct Camera
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#pragma once

#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <boost/shared_ptr.hpp>

using namespace std;
using namespace cv;

/**
 * @struct Camera
 * @brief This struct defines a camera with specific properties
 */
struct Camera
{
  /**
   * Constructor
   */
  Camera()
  {
    name = "";
    resolution = -1;
    colorSpace = -1;
    fps = -1;
    clientName = "";
    fovX = 0;
    fovY = 0;
    focalX = 0;
    focalY = 0;
    centerOffX = 0;
    centerOffY = 0;
    distCoeffs = (Mat1d(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  ~Camera()
  {
    delete image;
  }

  /**
   * Prints the configuration parameters
   *
   * @return void
   */
  void
  printConfig()
  {
    cout << "name: " << name << endl;
    cout << "resolution: " << resolution << endl;
    cout << "colorSpace: " << colorSpace << endl;
    cout << "fps: " << fps << endl;
    cout << "clientName: " << clientName << endl;
    cout << "fovX: " << fovX << endl;
    cout << "fovY: " << fovY << endl;
    cout << "focalX: " << focalX << endl;
    cout << "focalY: " << focalY << endl;
    cout << "centerOffX: " << centerOffX << endl;
    cout << "centerOffY: " << centerOffY << endl;
  }

  void
  getImagePtr(uint8_t*& image)
  {
    image = this->image;
  }

  //! Name of the camera
  string name;

  //! Camera subscribed client name
  string clientName;

  //! Resolution of the camera
  uint8_t resolution;

  //! Colorspace definition of the camera
  uint8_t colorSpace;

  //! Frames per second for the video
  uint16_t fps;

  //! Camera image
  uint8_t* image;

  //! Image width
  uint32_t width;

  //! Image height
  uint32_t height;

  //! Camera field of view X
  float fovX;

  //! Camera field of view Y
  float fovY;

  //! Camera focal length X
  float focalX;

  //! Camera focal length Y
  float focalY;

  //! Camera center offset X
  float centerOffX;

  //! Camera center offset Y
  float centerOffY;

  //! Distortion coefficients
  Mat distCoeffs;
};

typedef boost::shared_ptr<Camera> CameraPtr;
