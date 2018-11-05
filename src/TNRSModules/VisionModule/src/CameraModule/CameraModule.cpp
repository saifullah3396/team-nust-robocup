/**
 * @file CameraModule/CameraModule.cpp
 *
 * This file implements the class CameraModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#include "VisionModule/include/CameraModule/CameraModule.h"
#include "Utils/include/HardwareIds.h"

string
strDate()
{
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(buffer, sizeof(buffer), "%d-%m-%Y %I:%M:%S", timeinfo);
  string str(buffer);
  return str;
}

CameraModule::CameraModule(VisionModule* visionModule) :
  camProxy(visionModule->getCamProxy())
{
  if (camProxy) {
    if (setupCameras()) ;
    LOG_INFO("Cameras setup complete.")
  } else {
    ERROR("Could not get access to ALVideoDeviceProxy.")
  }
}

CameraModule::~CameraModule()
{
  for (size_t i = 0; i < NUM_CAMS; ++i)
    camProxy->unsubscribe(cams[i]->name);
}

bool
CameraModule::setupCameras()
{
  try {
    cams = vector < CameraPtr > (NUM_CAMS);
    videoWriter = vector < VideoWriter* > (NUM_CAMS);
    camProxy->unsubscribe("visionTop_0");
    camProxy->unsubscribe("visionBottom_0");
    for (size_t i = 0; i < NUM_CAMS; ++i) {
      cams[i] = CameraPtr(new Camera());
      int autoExposition, autoWhiteBalance, brightness, contrast, saturation,
        exposure, hue, gain, exposureAlgorithm, sharpness, whiteBalance,
        backlightCompensation;
      string cfgFile;
      if (i == TOP_CAM) {
        cfgFile = "UpperCamera";
      } else {
        cfgFile = "LowerCamera";
      }
      GET_CONFIG(
        cfgFile,
        (string, Subscription.name, cams[i]->name), (int, Subscription.resolution, cams[i]->resolution), (int, Subscription.colorSpace, cams[i]->colorSpace), (int, Subscription.fps, cams[i]->fps), (float, Intrinsic.fovX, cams[i]->fovX), (float, Intrinsic.fovY, cams[i]->fovY), (float, Intrinsic.focalX, cams[i]->focalX), (float, Intrinsic.focalY, cams[i]->focalY), (float, Intrinsic.centerOffX, cams[i]->centerOffX), (float, Intrinsic.centerOffY, cams[i]->centerOffY), (int, Calibration.autoExposition, autoExposition), (int, Calibration.autoWhiteBalance, autoWhiteBalance), (int, Calibration.brightness, brightness), (int, Calibration.contrast, contrast), (int, Calibration.saturation, saturation), (int, Calibration.exposure, exposure), (int, Calibration.hue, hue), (int, Calibration.gain, gain), (int, Calibration.exposureAlgorithm, exposureAlgorithm), (int, Calibration.sharpness, sharpness), (int, Calibration.whiteBalance, whiteBalance), (int, Calibration.backlightCompensation, backlightCompensation), (double, Distortion.a, cams[i]->distCoeffs.at<double>(0,0)), (double, Distortion.b, cams[i]->distCoeffs.at<double>(0,1)), (double, Distortion.c, cams[i]->distCoeffs.at<double>(0,2)), (double, Distortion.d, cams[i]->distCoeffs.at<double>(0,3)), (double, Distortion.e, cams[i]->distCoeffs.at<double>(0,4)), )
      AL::ALValue size = camProxy->resolutionToSizes(cams[i]->resolution);
      cams[i]->width = (int) size[0];
      cams[i]->height = (int) size[1];
      cams[i]->clientName = camProxy->subscribeCamera(
        cams[i]->name,
        i,
        cams[i]->resolution,
        cams[i]->colorSpace,
        cams[i]->fps);
      //cams[i]->focalX =
      //  (((float) size[0]) * 0.5) / tan(cams[i]->fovX * 0.5 * M_PI / 180);
      //cams[i]->focalY =
      //  (((float) size[1]) * 0.5) / tan(cams[i]->fovY * 0.5 * M_PI / 180);
      LOG_INFO("Subscribed Camera:" + cams[i]->clientName)
      cams[i]->image = new uint8_t[cams[i]->width * cams[i]->height * 2];
      camProxy->setAllParametersToDefault(i);
      if (autoExposition != -1000) camProxy->setCameraParameter(
        cams[i]->clientName,
        AL::kCameraAutoExpositionID,
        autoExposition);
      if (autoWhiteBalance != -1000) camProxy->setCameraParameter(
        cams[i]->clientName,
        AL::kCameraAutoWhiteBalanceID,
        autoWhiteBalance);
      if (brightness != -1000) camProxy->setCameraParameter(
        cams[i]->clientName,
        AL::kCameraBrightnessID,
        brightness);
      if (contrast != -1000) camProxy->setCameraParameter(
        cams[i]->clientName,
        AL::kCameraContrastID,
        contrast);
      if (saturation != -1000) camProxy->setCameraParameter(
        cams[i]->clientName,
        AL::kCameraSaturationID,
        saturation);
      if (exposure != -1000) camProxy->setCameraParameter(
        cams[i]->clientName,
        AL::kCameraExposureID,
        exposure);
      if (hue != -1000) camProxy->setCameraParameter(
        cams[i]->clientName,
        AL::kCameraHueID,
        hue);
      if (gain != -1000) camProxy->setCameraParameter(
        cams[i]->clientName,
        AL::kCameraGainID,
        gain);
      if (exposureAlgorithm != -1000) camProxy->setCameraParameter(
        cams[i]->clientName,
        AL::kCameraExposureAlgorithmID,
        exposureAlgorithm);
      if (sharpness != -1000) camProxy->setCameraParameter(
        cams[i]->clientName,
        AL::kCameraSharpnessID,
        sharpness);
      if (whiteBalance != -1000) camProxy->setCameraParameter(
        cams[i]->clientName,
        AL::kCameraWhiteBalanceID,
        whiteBalance);
      if (backlightCompensation != -1000) camProxy->setCameraParameter(
        cams[i]->clientName,
        AL::kCameraBacklightCompensationID,
        backlightCompensation);
    }
  } catch (const exception& e) {
    ERROR("Camera subscription not possible.")
    ERROR(e.what())
    return false;
  }
  return true;
}

#ifdef MODULE_IS_REMOTE
void CameraModule::updateImage(
  const unsigned& index,
  const int& saveImages,
  const bool& useLoggedImages)
{
  string cameraClient = cams[index]->clientName;
  try {
    /**
     * This image accessing function takes too much time.
     * Need some better way of getting images from the robot, probably,
     * making our own buffers and accessing it in raw format directly
     * from the system.
     */
    if(!useLoggedImages) {
      //high_resolution_clock::time_point tEnd, tStart;
      //tStart = high_resolution_clock::now();
      AL::ALValue img = camProxy->getImageRemote(cameraClient);
      //tEnd = high_resolution_clock::now();
      //duration<double> time_span = tEnd - tStart;
      //LOG_INFO("Time cam " << cameraClient << "  " << time_span.count());
      //!int width = (int) img[0];
      //!int height = (int) img[1];
      //!int nbLayers = (int) img[2];
      //!int colorSpace = (int) img[3];
      //!image[4] is the number of seconds, image[5] the
      //!number of microseconds.
      //!The pointer to the image data and its size:
      /*Mat bgr = Mat(Size(640,480), CV_8UC3);
       memcpy (bgr.data, (uint8_t*) img[6].GetBinary(), img[6].getSize());
       VisionUtils::displayImage(bgr, "InputBgr");
       Mat yuv;
       cvtColor(bgr, yuv, COLOR_BGR2YUV);
       VisionUtils::displayImage(yuv, "InputYuv1");*/
      //cout << yuv.at<Vec3b>(0, 0) << endl;
      //cout << yuv.at<Vec3b>(1, 0) << endl;
      //cout << yuv.at<Vec3b>(2, 0) << endl;
      //VisionUtils::bgrToYuv422(cams[index]->image, img, img.cols, img.rows);
      memcpy (cams[index]->image, (uint8_t*) img[6].GetBinary(), img[6].getSize());
      if(saveImages) {
        string imageStr;
        if (index == TOP_CAM) {
          imageStr = ConfigManager::getLogsDirPath() +
          "Images/Top/" + strDate() + ".jpg";
        } else {
          imageStr = ConfigManager::getLogsDirPath() +
          "Images/Bottom/" + strDate() + ".jpg";
        }
        cout << imageStr << endl;
        Mat yuv = Mat(Size(img[0], img[1]), CV_8UC2);
        yuv.data = cams[index]->image;
        Mat bgr;
        cvtColor(yuv, bgr, COLOR_YUV2BGR_YUY2);
        imwrite(imageStr, bgr);
      }
      camProxy->releaseImage(cameraClient);
    } else {
      string name;
      if (index == TOP_CAM) {
        GET_CONFIG(
          "VisionDebug",
          (string, InputImage.testImageTop, name),
        )
      } else {
        GET_CONFIG(
          "VisionDebug",
          (string, InputImage.testImageBot, name),
        )
      }
      Mat img = imread(ROOT_DIR + string("/ProcessingChilds/VisionModule/Logs/") + name);
      //Mat yuv;
      //cvtColor(img, yuv, COLOR_RGB2YUV);
      //VisionUtils::displayImage(yuv, "InputYuv1");
      //cout << yuv.at<Vec3b>(0, 0) << endl;
      //cout << yuv.at<Vec3b>(1, 0) << endl;
      //cout << yuv.at<Vec3b>(2, 0) << endl;
      bgrToYuv422(cams[index]->image, img, img.cols, img.rows);
      //VisionUtils::displayImage(cams[index]->image, "cams[index]->image");
    }
  } catch (const exception& e) {
    ERROR(e.what())
  }
}
#else
void CameraModule::updateImage(
  const unsigned& index)
{
  string cameraClient = cams[index]->clientName;
  try {
    AL::ALImage* img;
    img = (AL::ALImage*) camProxy->getImageLocal(cameraClient);
    memcpy(cams[index]->image, (uint8_t*) img->getData(), img->getSize());
    camProxy->releaseImage(cameraClient);
  } catch (const exception& e) {
    ERROR(e.what())
  }
}
#endif

void
CameraModule::releaseImage(const unsigned& index)
{
  string cameraClient = cams[index]->clientName;
  camProxy->releaseImage(cameraClient);
}

void CameraModule::recordVideo(const unsigned& index)
{
  static string logsDir = ConfigManager::getLogsDirPath();
  //! VideoWriter object;
  if (!videoWriter[index]) {
		try {
			string vidName;
			if (index == TOP_CAM) {
				vidName = logsDir + "Video/Top/";
			} else if (index == BOTTOM_CAM) {
				vidName = logsDir + "Video/Bottom/";
			}
			int cnt = FileUtils::getFileCnt(vidName);
			vidName += "vid-";
			vidName += DataUtils::varToString(cnt+1);
			vidName += ".avi";
			videoWriter[index] = new VideoWriter();
			videoWriter[index]->open(
				vidName, 
				CV_FOURCC('M', 'P', 'E', 'G'), 
				(int)cams[index]->fps, 
				Size((int)cams[index]->width, (int)cams[index]->height)
			);
			if (!videoWriter[index]->isOpened()) {
				delete videoWriter[index];
				throw "Unable to open video: \n\t" + vidName + " for write.";
			}
		} catch (exception &e) {
			ERROR(e.what());
		}
  } else {
		Mat yuv422 = 
			Mat(
				Size((int)cams[index]->width, (int)cams[index]->height), 
				CV_8UC2
			);
		yuv422.data = cams[index]->image;
		Mat bgr;
		cvtColor(yuv422, bgr, COLOR_YUV2BGR_YUY2);
		cout << "Writing..." << endl;
		videoWriter[index]->write(bgr);
	}
}

void CameraModule::stopRecording()
{
	for (int i = 0; i < videoWriter.size(); ++i) {
		if (videoWriter[i]) {
			videoWriter[i]->release();
			delete videoWriter[i];
		}
	}
}
