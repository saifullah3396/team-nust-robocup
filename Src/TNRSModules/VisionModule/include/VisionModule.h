/**
 * @file VisionModule/VisionModule.h
 *
 * This file declares a class for vision planning.
 * All the functions and algorithms for image processing, object
 * detection and classification, and colorspaces will be defined
 * under this module.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <Eigen/Dense>
#include <alproxies/alvideodeviceproxy.h>
#include "TNRSBase/include/DebugBase.h"
#include "TNRSBase/include/BaseIncludes.h"
#include "VisionModule/include/VisionModuleIds.h"
#include "VisionModule/include/ColorHandler.h"
#include "Utils/include/BallInfo.h"
#include "Utils/include/GoalInfo.h"
#include "Utils/include/PlanningState.h"
#include "Utils/include/RoboCupGameControlData.h"
#include "Utils/include/Obstacle.h"
#include "Utils/include/DebugUtils.h"

class CameraModule;
class ImagePreprocessor;
class CameraTransform;
class FeatureExtraction;
typedef boost::shared_ptr<CameraModule> CameraModulePtr;
typedef boost::shared_ptr<ImagePreprocessor> ImagePreprocessorPtr;
typedef boost::shared_ptr<CameraTransform> CameraTransformPtr;
typedef boost::shared_ptr<FeatureExtraction> FeatureExtractionPtr;
typedef boost::shared_ptr<AL::ALVideoDeviceProxy> ALVideoDeviceProxyPtr;

/**
 * @class VisionModule
 * @brief A class for vision and feature extraction. All the functions
 *   and algorithms for image processing, object detection and
 *   classification, and colorspaces will be defined under this module.
 */
class VisionModule : public BaseModule, public DebugBase
{
  /**
   * Debug variables for this module
   */ 
  INIT_DEBUG_BASE_(
    //! Option to choose which output image should be sent for debugging.
    (int, debug, 0),
    //! Option to enable any kind of debugging.
    (int, debugImageIndex, 0),
  )

  /**
   * Output connector and variables for this module
   */ 
  CREATE_INPUT_CONNECTOR(VisionInput,
    (int, visionThreadPeriod),
    (Matrix4f, upperCamInFeet),
    (Matrix4f, lowerCamInFeet),
    (unsigned, planningState),
    (RoboCupGameControlData, gameData),
    (bool, robotOnSideLine),
  )

  /**
   * Output connector and variables for this module
   */ 
  CREATE_OUTPUT_CONNECTOR(VisionOutput,
    (BallInfo, ballInfo),
    (GoalInfo, goalInfo),
    (bool, landmarksFound),
    (ObsObstacles, obstaclesObs),
  )

public:
  /**
   * Initializes the vision module with its thread.
   *
   * @param processingModule: Pointer to base class which is in
   *   this case the ProcessingModule.
   * @param camProxy: Pointer to NaoQi's camera proxy.
   */
  VisionModule(void* processingModule, const ALVideoDeviceProxyPtr& camProxy);

  /**
   * Defualt destructor for this class.
   */
  ~VisionModule()
  {
    delete genericInputConnector;
    delete genericOutputConnector;
  }

  /**
   * Derived from BaseModule
   */
  void
  init();

  /**
   * Derived from BaseModule
   */
  void
  handleRequests();

  /**
   * Derived from BaseModule
   */
  void
  mainRoutine();

  /**
   * Derived from BaseModule
   */
  void
  initMemoryConn();

  /**
   * Derived from BaseModule
   */
  void
  setThreadPeriod();

  /**
   * Returns the pointer to the ALVideoDeviceProxyPtr.
   *
   * @return Pointer to ALVideoDeviceProxyPtr
   */
  ALVideoDeviceProxyPtr
  getCamProxy()
  {
    return camProxy;
  }

  /**
   * Returns the pointer to the CameraModule class.
   *
   * @return Pointer to CameraModule
   */
  CameraModulePtr
  getCameraModule()
  {
    return cameraModule;
  }

  /**
   * Returns the pointer to the required feature extraction class.
   *
   * @return Pointer to FeatureExtraction
   */
  FeatureExtractionPtr
  getFeatureExtClass(const unsigned &index)
  {
    return featureExt[index];
  }

  /**
   * Returns the pointer to the image transform class.
   *
   * @return Pointer to CameraTransform
   */
  CameraTransformPtr
  getCameraTransform()
  {
    return cameraTransform;
  }

  /**
   * Returns the pointer to the ColorHandler class.
   *
   * @return Pointer to ColorHandler
   */
  ColorHandlerPtr
  getColorHandler()
  {
    return colorHandler;
  }

private:
  /**
   * Sets up the field points and projects them on the field
   */ 
  void setupFieldProjection();

  //! Runs the vision module if true
  bool runVision;

  //! Flag to use logged images or realtime images
  bool useLoggedImages;

  //! Project field on the images
  bool projectField;

  //! Field points in world coordinates
  vector<Point3f> fieldPoints;

  //! Logs images for the respective camera in respective robot dir
  vector<bool> logImages;

  //! Starts video writer for the respective camera if true
  vector<bool> writeVideo;

  //! Pointer to a vector of feature extraction classes.
  vector<FeatureExtractionPtr> featureExt;

  //! Pointer to CameraModule class.
  CameraModulePtr cameraModule;

  //! Pointer to image preprocessing class.
  ImagePreprocessorPtr imagePreprocessor;

  //! Pointer to camera transform class.
  CameraTransformPtr cameraTransform;

  //! Pointer to ColorHandler class.
  ColorHandlerPtr colorHandler;

  //! Pointer to NaoQi video device proxy
  ALVideoDeviceProxyPtr camProxy;
};
