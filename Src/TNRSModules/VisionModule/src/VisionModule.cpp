/**
 * @file VisionModule/VisionModule.cpp
 *
 * This file implements a class for vision planning.
 * All the functions and algorithms for image processing, object
 * detection and classification, and colorspaces will be defined
 * under this module.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include <boost/exception/diagnostic_information.hpp> 
#include <Eigen/Dense>
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "VisionModule/include/CameraModule/CameraModule.h" 
#include "VisionModule/include/FeatureExtraction/BallExtraction.h"
#include "VisionModule/include/FeatureExtraction/FieldExtraction.h"
#include "VisionModule/include/FeatureExtraction/RegionSegmentation.h"
#include "VisionModule/include/FeatureExtraction/FeatureExtraction.h"
#include "VisionModule/include/FeatureExtraction/GoalExtraction.h"
#include "VisionModule/include/FeatureExtraction/LinesExtraction.h"
#include "VisionModule/include/FeatureExtraction/RobotExtraction.h"
#include "VisionModule/include/ImagePreprocessor.h"
#include "VisionModule/include/CameraTransform.h"
#include "VisionModule/include/VisionModule.h"
#include "VisionModule/include/VisionRequest.h"
#include "LocalizationModule/include/LocalizationRequest.h"

VisionModule::VisionModule(void* processingModule,
  const ALVideoDeviceProxyPtr& camProxy) :
  BaseModule(processingModule, (unsigned) TNSPLModules::VISION, "VisionModule"),
  DebugBase("VisionModule", this), 
  camProxy(camProxy),
  runVision(false),
  useLoggedImages(false),
  projectField(false)
{
  logImages = vector<bool>(NUM_CAMS, false);
  writeVideo = vector<bool>(NUM_CAMS, false);
}

void
VisionModule::setThreadPeriod()
{
  setPeriodMinMS(IVAR(int, VisionModule::visionThreadPeriod));
}

void
VisionModule::initMemoryConn()
{
  ASSERT_MSG(sharedMemory, "Shared Memory not found.");
  genericInputConnector = 
    new InputConnector(this, getModuleName() + "InputConnector");
  genericOutputConnector = 
    new OutputConnector(this, getModuleName() + "OutputConnector");
  genericInputConnector->initConnector();
  genericOutputConnector->initConnector();
}

void
VisionModule::init()
{
  PRINT("Initializing Vision Debug Base.")
  initDebugBase();
  PRINT("Initializing Camera Module.")
  cameraModule = boost::shared_ptr < CameraModule > (new CameraModule(this));
  PRINT("Initializing Image Preprocessor.")
  imagePreprocessor =
    boost::shared_ptr < ImagePreprocessor > (new ImagePreprocessor(this));
  PRINT("Initializing Camera Transform.")
  cameraTransform = boost::shared_ptr < CameraTransform > (new CameraTransform(
    this));
  PRINT("Initializing Color Handler.")
  colorHandler = boost::shared_ptr < ColorHandler > (new ColorHandler());
  RoboCupGameControlData gameData =
    IVAR(RoboCupGameControlData, VisionModule::gameData);
  unsigned ourTeam = gameData.teams[0].teamColour;
  unsigned oppTeam = gameData.teams[1].teamColour;
  Colors ourColor = Colors::YELLOW, oppColor = Colors::BLUE;
  if (ourTeam == TEAM_BLACK) ourColor = Colors::BLACK;
  else if (ourTeam == TEAM_BLUE) ourColor = Colors::BLUE;
  else if (ourTeam == TEAM_RED) ourColor = Colors::RED;
  else if (ourTeam == TEAM_YELLOW) ourColor = Colors::YELLOW;

  if (oppTeam == TEAM_BLACK) oppColor = Colors::BLACK;
  else if (oppTeam == TEAM_BLUE) oppColor = Colors::BLUE;
  else if (oppTeam == TEAM_RED) oppColor = Colors::RED;
  else if (oppTeam == TEAM_YELLOW) oppColor = Colors::YELLOW;
  bool blackJerseyExists =
    ourColor == Colors::BLACK || oppColor == Colors::BLACK;
  FeatureExtraction::setup(this);
  ourColor = Colors::BLUE;
  oppColor = Colors::YELLOW;
  FeatureExtraction::updateColorInfo(ourColor, oppColor, blackJerseyExists);

  featureExt.resize(NUM_FE_MODULES);
  featureExt[REGION_SEGMENTATION] =
    boost::shared_ptr < RegionSegmentation > (new RegionSegmentation(this));
  featureExt[FIELD] =
    boost::shared_ptr < FieldExtraction > (new FieldExtraction(this));
  featureExt[ROBOT] =
    boost::shared_ptr < RobotExtraction > (new RobotExtraction(this));
  featureExt[GOAL] = boost::shared_ptr < GoalExtraction > (new GoalExtraction(
    this));
  featureExt[BALL] = boost::shared_ptr < BallExtraction > (new BallExtraction(
    this));
  featureExt[LINES] =
    boost::shared_ptr < LinesExtraction > (new LinesExtraction(this));
  try {
    boost::static_pointer_cast < RegionSegmentation > (featureExt[REGION_SEGMENTATION])->setFieldExtraction(
      boost::static_pointer_cast < FieldExtraction > (featureExt[FIELD]));
    boost::static_pointer_cast < RegionSegmentation > (featureExt[REGION_SEGMENTATION])->setBallExtraction(
      boost::static_pointer_cast < BallExtraction > (featureExt[BALL]));
    boost::static_pointer_cast < RegionSegmentation > (featureExt[REGION_SEGMENTATION])->setGoalExtraction(
      boost::static_pointer_cast < GoalExtraction > (featureExt[GOAL]));
  } catch (boost::exception &e) {
    std::cerr << boost::diagnostic_information(e);
  }
  int tempDebug;
  int tempDebugImageIndex;
  GET_CONFIG(
    "VisionDebug",
    (int, VisionModule.debug, tempDebug), 
    (int, VisionModule.debugImageIndex, tempDebugImageIndex),
  )
  SET_DVAR(int, debug, tempDebug);
  SET_DVAR(int, debugImageIndex, tempDebugImageIndex);
}

void VisionModule::handleRequests()
{
  while (!inRequests.isEmpty()) {
    auto request = inRequests.queueFront();
    if (boost::static_pointer_cast <VisionRequest>(request)) {
      auto reqId = request->getId();
      if (reqId == (unsigned)VisionRequestIds::SWITCH_VISION) {
        auto sv = boost::static_pointer_cast <SwitchVision>(request);
        runVision = sv->state;
      } else if (reqId == (unsigned)VisionRequestIds::SWITCH_VIDEO_WRITER) {
        auto svw = boost::static_pointer_cast <SwitchVideoWriter>(request);
        writeVideo[svw->camIndex] = svw->state;
      } else if (reqId == (unsigned)VisionRequestIds::SWITCH_FIELD_PROJECTION) {
        auto sfp = boost::static_pointer_cast <SwitchFieldProjection>(request);
        projectField = sfp->state;
      } else if (reqId == (unsigned)VisionRequestIds::SWITCH_LOG_IMAGES) {
        auto sli = boost::static_pointer_cast <SwitchLogImages>(request);
        logImages[sli->camIndex] = sli->state;
      } else if (reqId == (unsigned)VisionRequestIds::SWITCH_USE_LOGGED_IMAGES) {
        auto uli = boost::static_pointer_cast <SwitchUseLoggedImages>(request);
        useLoggedImages = uli->state;
      }
    }
    inRequests.popQueue();
  }
}

void
VisionModule::mainRoutine()
{
  OVAR(ObsObstacles, VisionModule::obstaclesObs).data.clear();
  // Execution of this module is decided by planning module.
  if (runVision) {
    for (int i = 0; i < NUM_CAMS; ++i) {
      cameraModule->updateImage(i, logImages[i], useLoggedImages);
    }
  /* Video recording process is too computationally expensive for the robot
   * int& writeVideo = IVAR(int, VisionModule::writeVideo);
  if (writeVideo == 0 || writeVideo == 1) {
    cameraModule->recordVideo(writeVideo);
    return;
  } else if (writeVideo == 2) {
    for (int i = 0; i < 2; ++i)
      cameraModule->recordVideo(i);
    return;	
  } else if (writeVideo == -1) {
    cameraModule->stopRecording();
    return;
  }*/
    //cameraTransform->update();
    //FeatureExtraction::setupImagesAndHists();
    //if (colorHandler->fieldHistFormed()) {
    FeatureExtraction::clearLandmarks();
    //featureExt[REGION_SEGMENTATION]->processImage();
    //featureExt[FIELD]->processImage();
    //featureExt[ROBOT]->processImage();
    //featureExt[GOAL]->processImage();
    //featureExt[BALL]->processImage();
    //featureExt[LINES]->processImage();
    KnownLandmarksUpdatePtr klu = 
      boost::make_shared<KnownLandmarksUpdate>(
        FeatureExtraction::getKnownLandmarks());
    UnknownLandmarksUpdatePtr ulu = 
      boost::make_shared<UnknownLandmarksUpdate>(
        FeatureExtraction::getUnknownLandmarks());
    BaseModule::publishModuleRequest(klu);
    BaseModule::publishModuleRequest(ulu);
    if (klu->landmarks.size() > 0 || ulu->landmarks.size() > 0)
      OVAR(bool, VisionModule::landmarksFound) = true;
    else 
      OVAR(bool, VisionModule::landmarksFound) = false;
    //}
    OVAR(ObsObstacles, VisionModule::obstaclesObs).assignId();
    if (projectField)
      setupFieldProjection();
    VisionUtils::displayImage(FeatureExtraction::getBgrMat(0), "top");
    VisionUtils::displayImage(FeatureExtraction::getBgrMat(1), "bottom");
  }
}

void VisionModule::setupFieldProjection()
{
  static bool fieldPointsSaved = false;
  if (!fieldPointsSaved) {
    float x = 4.5f;
    float y = -3.f;
    for (int i = 0; i < 13; ++i) {
      fieldPoints.push_back(Point3f(x, y, 0.0));
      y += 0.5f;
    }
    x = 4.f;
    y = -3.f;
    for (int i = 0; i < 8; ++i) {
      fieldPoints.push_back(Point3f(x, y, 0.0));
      fieldPoints.push_back(Point3f(x, -y, 0.0));
      x -= 0.5f;
    }
    float xEllipse = 0;
    float ellipseRadius = 0.75;
    for (int i = 0; i < 25; ++i) {
      xEllipse += 0.75 / 25.0;
      //cout << "xEllipse: " << xEllipse << endl;
      float yEllipse =
      sqrt(ellipseRadius * ellipseRadius - xEllipse * xEllipse);
      fieldPoints.push_back(Point3f(xEllipse, yEllipse, 0.0));
      fieldPoints.push_back(Point3f(xEllipse, -yEllipse, 0.0));
    }
    fieldPointsSaved = true;
  }
  vector<Point2f> imagePs;
  cameraTransform->worldToImage(0, fieldPoints, imagePs);
  cameraTransform->worldToImage(1, fieldPoints, imagePs);
  VisionUtils::drawPoints(imagePs, FeatureExtraction::getBgrMat(0));
  VisionUtils::drawPoints(imagePs, FeatureExtraction::getBgrMat(1));
}
