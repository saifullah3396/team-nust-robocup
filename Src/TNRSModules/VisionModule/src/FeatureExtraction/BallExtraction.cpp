/**
 * @file FeatureExtraction/BallExtraction.cpp
 *
 * This file implements the class for ball extraction from the image.
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017  
 */

#include "VisionModule/include/FeatureExtraction/BallExtraction.h"

void 
BallExtraction::drawState(const Mat& state, const Scalar& color)
{
  Point2f orig;
  orig.x = state.at<float>(6);
  orig.y = state.at<float>(7);
  float w = state.at<float>(10);
  float h = state.at<float>(11);
  rectangle(
    bgrMat[currentImage], 
    Rect(Point2f(orig.x - w / 2, orig.y - h / 2), Size(w, h)), 
    color, 
    3
  );
}

float 
BallExtraction::getCurrentFriction()
{
  //TODO: Define when to consider ball static phase while tracking
  if (false) // ballStaticPhase
    return coeffSF;
  else
    return coeffRF;
}

unsigned
BallExtraction::getBallFrameInNextCycle(const BallInfo& ballInfo)
{
  // Just an estimated state
  Point3f ballWorldRel = 
    Point3f(ballInfo.posRel.x, ballInfo.posRel.y, ballRadius);
	auto otherCam = currentImage == TOP_CAM ? BOTTOM_CAM : TOP_CAM;
  Point2f ballInOther;
  cameraTransform->worldToImage(otherCam, ballWorldRel, ballInOther);
  if (ballInOther.x > 0 && 
      ballInOther.x < imageWidth[otherCam] &&
      ballInOther.y > 0 &&
      ballInOther.y < imageHeight[otherCam]
     ) 
  {
    //PRINT("ballInCurrent: " << ballInfo.posImage)
    //PRINT("ballInOther: " << ballInOther)
    //! See if the ball is more visible in the other cam
    float yBoundary = 
			ballInfo.posImage.y > getImageHeight() / 2 ? 
			getImageHeight() - ballInfo.posImage.y : ballInfo.posImage.y;
		yBoundary /= getImageHeight();
		float yBoundaryInOther = 
			ballInOther.y > imageHeight[otherCam] / 2 ? 
			imageHeight[otherCam] - ballInOther.y  : ballInOther.y;
		yBoundaryInOther /= imageHeight[otherCam];
		//cout << "yBoundaryInOther: " << yBoundaryInOther << endl;
		//cout << "yBoundary: " << yBoundary << endl;
    if (yBoundaryInOther > yBoundary) {
			return otherCam;
		} else {
			return currentImage;	
		}
  } else {
    return currentImage;
  }
}

void
BallExtraction::updateBallInfo()
{
  Mat ballState = ballTracker->getEstimatedState();
  BallInfo ballInfo;
  ballInfo.found = ballTracker->getBallFound();
  if (ballInfo.found) 
    drawState(ballState, Scalar(255,0,0));
  ballInfo.camera = currentImage;
  ballInfo.posRel.x = ballState.at<float>(0);
  ballInfo.posRel.y = ballState.at<float>(1);
  ballInfo.velRel.x = ballState.at<float>(2);
  ballInfo.velRel.y = ballState.at<float>(3);
  ballInfo.accRel.x = ballState.at<float>(4);
  ballInfo.accRel.y = ballState.at<float>(5);
  ballInfo.posImage.x = ballState.at<float>(6);
  ballInfo.posImage.y = ballState.at<float>(7);
  //Point2f nextImagePos;
  //nextImagePos.x = ballInfo.posImage.x + ballState.at<float>(8) * cycleTime;
  //nextImagePos.x = ballInfo.posImage.y + ballState.at<float>(9) * cycleTime;
  //line(bgrMat[currentImage], ballInfo.posImage, nextImagePos, Scalar(255,0,0));
  ballInfo.bboxWidth = ballState.at<float>(10);
  ballInfo.bboxHeight = ballState.at<float>(11);
  ballInfo.ballAge = ballTracker->getTimeSinceLost();
  ballInfo.cameraNext = getBallFrameInNextCycle(ballInfo);
  /*cout << "Ball Position: " << ballInfo.posRel << endl;
  cout << "Ball posImage: " << ballInfo.posImage << endl;
  cout << "Ball Camera: " << ballInfo.camera << endl;
  cout << "Ball Camera Next frame: " << ballInfo.cameraNext << endl;
  cout << "Ball Found: " << ballInfo.found << endl;
  cout << "Ball W: " << ballInfo.width << endl;
  cout << "Ball H: " << ballInfo.height << endl;*/
  OVAR(BallInfo, VisionModule::ballInfo) = ballInfo;
}

void
BallExtraction::simpleDetect(const Rect& origRect, Mat croppedImage)
{
  vector < vector<Point> > contours;
  vector < Vec4i > hierarchy;
  Mat black;
  threshold(croppedImage, black, 50, 255, 1);
  Mat canny;
  Canny(black, canny, 100, 255, 3);
  findContours(
    canny,
    contours,
    hierarchy,
    CV_RETR_TREE,
    CV_CHAIN_APPROX_SIMPLE,
    Point(0, 0));
  if (contours.size() < 5) return;
  vector < Point > result;
  vector < Point > pts;
  for (size_t i = 0; i < contours.size(); i++)
    for (size_t j = 0; j < contours[i].size(); j++)
      pts.push_back(contours[i][j]);
  Rect ball = boundingRect(pts);
  Point center;
  center.x = ball.x + ball.width / 2 - croppedImage.rows / 2;
  center.y = ball.y + ball.height / 2 - croppedImage.cols / 2;
  ball.x = origRect.x + center.x;
  ball.y = origRect.y + center.y;
  ball.width = origRect.width;
  ball.height = origRect.height;
  foundBall.push_back(ball);
  //polylines(croppedImage, result, false, Scalar(255), 2);
  //for( int i = 0; i < contours.size(); i++ )
  //{
  //  drawContours(croppedImage, contours, i, Scalar(0), 2, 8, hierarchy, 0, Point() );
  //}
  //vector<Rect> ball;
  //rectangle(croppedImage, ball, Scalar(0), 3);
  //imshow("croppedImage" , croppedImage);
  //waitKey(0);
}

void
BallExtraction::simBallDetector(const Rect& origRect)
{
  if (origRect.width < 5 || origRect.height < 5) return;
  Rect scaled = origRect;
	int factor = 2;
	scaled = scaled - Point((scaled.width * factor) / 2, (scaled.height * factor) / 2);
	scaled += Size(scaled.width * factor, scaled.height * factor);
	scaled = scaled & Rect(0, 0, getImageWidth(), getImageHeight());
  Mat cropped = bgrMat[currentImage](scaled), redImage;
  inRange(cropped, Scalar(0, 0, 150), Scalar(60, 60, 255), redImage);
  vector < vector<Point> > contours;
  vector < Vec4i > hierarchy;
  findContours(
    redImage,
    contours,
    hierarchy,
    CV_RETR_TREE,
    CV_CHAIN_APPROX_SIMPLE,
    Point(0, 0));
  if (contours.empty()) return;
  vector < Point > possibleBall;
  float maxArea = 0;
  for (size_t i = 0; i < contours.size(); i++) {
    float area = contourArea(contours[i]);
    if (area > 10) {
      if (area > maxArea) {
        maxArea = area;
        possibleBall = contours[i];
      }
    }
  }
  if (maxArea > 0) {
    //cout << "max area  > 0 " << endl;
    Rect ball = boundingRect(possibleBall);
    ball.x = scaled.x + ball.x;
    ball.y = scaled.y + ball.y;
    foundBall.push_back(ball);
    /*rectangle(
      bgrMat[currentImage], 
      ball, 
      Scalar(0,0,0), 
      3
    );
    for( int i = 0; i < contours.size(); i++ )
    {
      drawContours(cropped, contours, i, Scalar(0), 2, 8, hierarchy, 0, Point() );
    }*/
  }

  //VisionUtils::displayImage(bgrMat[currentImage], "bgrMat[currentImage]1");
  //VisionUtils::displayImage(cropped, "croppedImage");
  //VisionUtils::displayImage(redImage, "redImage");
}

void
BallExtraction::applyClassifier(const Rect& origRect, Mat croppedImage)
{
  if (croppedImage.cols < 5 || croppedImage.rows < 5) return;

  float ratio = 1.f;
  if (croppedImage.rows > 35) {
    ratio = 35.f / (float) croppedImage.rows;
    resize(croppedImage, croppedImage, cv::Size(), ratio, ratio);
  }

  vector < Rect > ball;
  cascade.detectMultiScale(croppedImage, ball, 1.1, 1, 0 | CASCADE_SCALE_IMAGE);
  for (int i = 0; i < ball.size(); ++i) {
    ball[i] = Rect(
      origRect.x + ball[i].x / ratio,
      origRect.y + ball[i].y / ratio,
      ball[i].width / ratio,
      ball[i].height / ratio);
    foundBall.push_back(ball[i]);
  }
}

void
BallExtraction::setImage()
{
	auto& ballInfo = OVAR(BallInfo, VisionModule::ballInfo);
	//cout << "Camera: " << ballInfo.camera << endl;
	//cout << "CameraNext: " << ballInfo.cameraNext << endl;
	if (ballInfo.cameraNext != ballInfo.camera) {
		//cout << "camera changed" << endl;
		Mat ballState = ballTracker->getEstimatedState();
		//cout << "ballStatePrev: " << ballState << endl;
		//! rel position in real world
		Point3f ballWorldRel = 
			Point3f(ballState.at<float>(0), ballState.at<float>(1), ballRadius);
		//! ball in camera next
		Point2f ballInOther;
		cameraTransform->worldToImage(
			ballInfo.cameraNext, ballWorldRel, ballInOther);
		
		//! Position is shifted from the previous cam to new cam
		ballState.at<float>(6) = ballInOther.x;
		ballState.at<float>(7) = ballInOther.y;
		
		//! Reassigning ball width and height in new cam
		//! Velocity is scaled as well
		float camRatio = imageWidth[ballInfo.cameraNext] / (float)imageWidth[ballInfo.camera];
		//cout << "ballInfo.camera: " << ballInfo.camera << endl;
		//cout << "ballInfo.cameraNext: " << ballInfo.cameraNext << endl;
		//cout << "camRatio: " << camRatio << endl;
		ballState.at<float>(8) *= camRatio;
		ballState.at<float>(9) *= camRatio;
		ballState.at<float>(10) *= camRatio;
		ballState.at<float>(11) *= camRatio;
		//cout << "ballStateNew: " << ballState << endl;
		ballTracker->reset(ballInfo.cameraNext, ballState);
		currentImage = ballInfo.cameraNext;
    xySeen.clear();
	} else {
		if (!ballTracker->getBallFound()) {
			if (currentImage == TOP_CAM) {
				currentImage = BOTTOM_CAM;
			} else {
				currentImage = TOP_CAM;
			}
			xySeen.clear();
			ballTracker->reset(currentImage);
		}
	}
}

void
BallExtraction::scanRandom(vector<RectPtr>& boundRects,
  const vector<int>& pairIndices, const int& iters)
{
  vector < pair<int, int> > pairs;
  pairs = currentImage == TOP_CAM ? topXYPairs : bottomXYPairs;
  for (int i = 0; i < pairIndices.size(); ++i) {
    if (i >= iters) break;
    auto index = pairIndices[i];
    auto x = pairs[index].first * gridSize;
    auto y = pairs[index].second * gridSize;
    xySeen.push_back(index);
    Mat cropped = getGrayImage()(
      Rect(x, y, gridSize, gridSize) & Rect(
        0,
        0,
        getImageWidth(),
        getImageHeight()));
    //rectangle(bgrMat[currentImage], Rect(x, y, gridSize, gridSize) & Rect(0, 0, getImageWidth(), getImageHeight()), Scalar(255,255,255), 1);
    Mat black;
    threshold(cropped, black, 50, 255, 1);
    if (countNonZero(black) <= 0) continue;
    dilate(black, black, Mat(), Point(-1, -1));
    vector < vector<Point> > contours;
    vector < Vec4i > hierarchy;
    //auto offset = Point(0, fHeight);
    //auto offset = Point(x, y >= fHeight ? y : fHeight);
    auto offset = Point(x, y);
    findContours(
      black,
      contours,
      hierarchy,
      CV_RETR_EXTERNAL,
      CV_CHAIN_APPROX_SIMPLE,
      offset);
    for (int i = 0; i < contours.size(); i++) {
      Rect boundRect = boundingRect(contours[i]);
      float ratio = boundRect.width / (float) boundRect.height;
      if (ratio < 0.333333333 || ratio > 3.0) continue;
      //drawContours(bgrMat[currentImage], contours, i, Scalar(0,255,0), 2, 8, hierarchy, 0, Point() );
      boundRects.push_back(boost::make_shared < Rect > (boundRect));
      //rectangle(bgrMat[currentImage], *boundRects.back(), Scalar(255,255,255), 1);
    }
  }
}

void
BallExtraction::scanRandom(vector<RectPtr>& boundRects, const Rect& roi)
{
	Rect scaled = roi;
	int factor = 1;
	scaled = scaled - Point((scaled.width * factor) / 2, (scaled.height * factor) / 2);
	scaled += Size(scaled.width * factor, scaled.height * factor);
	scaled = scaled & Rect(0, 0, getImageWidth(), getImageHeight());
    //rectangle(bgrMat[currentImage], scaled, Scalar(0,0,0), 1);
  Mat cropped = getGrayImage()(scaled);
  Mat black;
  threshold(cropped, black, 50, 255, 1);
  if (countNonZero(black) <= 0) return;
  dilate(black, black, Mat(), Point(-1, -1));
  vector < vector<Point> > contours;
  vector < Vec4i > hierarchy;
  auto offset = Point(scaled.x, scaled.y);
  findContours(
    black,
    contours,
    hierarchy,
    CV_RETR_EXTERNAL,
    CV_CHAIN_APPROX_SIMPLE,
    offset);
  for (int i = 0; i < contours.size(); i++) {
    Rect boundRect = boundingRect(contours[i]);
    float ratio = boundRect.width / (float) boundRect.height;
    if (ratio < 0.333333333 || ratio > 3.0) continue;
    if (boundRect.width > roi.width / 2 ||
				boundRect.width < roi.width / 10 || 
				boundRect.height > roi.height / 2 ||
				boundRect.height < roi.height / 10) 
		{ 
			continue;
    }
    boundRects.push_back(boost::make_shared < Rect > (boundRect));
  }
}

void
BallExtraction::filterRegions(vector<RectPtr>& boundRects,
  const float& threshold)
{
  for (size_t j = 0; j < boundRects.size(); ++j) {
    if (!boundRects[j]) continue;
    auto r1 = *boundRects[j];
    for (size_t k = 0; k < boundRects.size(); ++k) {
      if (!boundRects[k]) continue;
      if (j != k) {
        auto r2 = *boundRects[k];
        Point c1 = Point(r1.x + r1.width / 2, r1.y + r1.height / 2);
        Point c2 = Point(r2.x + r2.width / 2, r2.y + r2.height / 2);
        if (norm(c1 - c2) < threshold) {
          r1 = r1 | r2;
          *boundRects[j] = r1;
          boundRects[k].reset();
        }
      }
    }
  }
}

void
BallExtraction::classifyRegions(vector<RectPtr>& boundRects)
{
  for (size_t j = 0; j < boundRects.size(); ++j) {
    if (!boundRects[j]) continue;
    //rectangle(bgrMat[currentImage], *boundRects[j], Scalar(255,255,255), 1);
    Rect r = *boundRects[j];
    float wHRatio = r.width / (float) r.height;
    Point newCenter = Point(r.x + r.width / 2, r.y + r.height / 2);
    r = Rect(
      newCenter - Point(r.width / wHRatio / 2, r.height / 2),
      Size(r.width / wHRatio, r.height));
    r -= Point(r.width / 4, r.height / 4);
    r += Size(r.width / 2, r.height / 2);
    r = r & Rect(0, 0, getImageWidth(), getImageHeight());
    *boundRects[j] = r;
    //rectangle(bgrMat[currentImage], *boundRects[j], Scalar(255,255,255), 1);
    Mat croppedImage = getGrayImage()(r);
    //VisionUtils::displayImage(croppedImage, "cropped");
    //waitKey(0);
    applyClassifier(r, croppedImage);
  }
}

void
BallExtraction::findBallUpperCam()
{
  auto fHeight = fieldExt->getFieldHeight();
  //if (fHeight == 0) return;
  srand(visionModule->getModuleTime() * 100);
  vector<int> pairIndices;
  int gridStartY = fHeight / gridSize;
  for (int i = 0; i < topXYPairs.size(); ++i) {
    //cout << "i: " << i << endl;
    if (topXYPairs[i].second >= gridStartY && find(
      xySeen.begin(),
      xySeen.end(),
      i) == xySeen.end()) pairIndices.push_back(i);
  }
  if (pairIndices.empty()) {
    xySeen.clear();
    for (int i = 0; i < topXYPairs.size(); ++i) {
      //cout << "i: " << i << endl;
      if (topXYPairs[i].second >= gridStartY) pairIndices.push_back(i);
    }
  }
  vector<RectPtr> boundRects;
  random_shuffle(pairIndices.begin(), pairIndices.end());
  scanRandom(boundRects, pairIndices, 5);
  filterRegions(boundRects, 50);
  classifyRegions(boundRects);
}

void
BallExtraction::findBallUpperCam(const Rect& roi)
{
  vector<RectPtr> boundRects;
  scanRandom(boundRects, roi);
  filterRegions(boundRects, 50);
  for (int j = 0; j < boundRects.size(); ++j) {
	  if (!boundRects[j])
		continue;
      //rectangle(bgrMat[currentImage], *boundRects[j], Scalar(255,0,0), 1);
	  float factorX = roi.width / boundRects[j]->width;
	  float factorY = roi.height / boundRects[j]->height;
		*boundRects[j] = *boundRects[j] - Point((boundRects[j]->width * factorX) / 2, (boundRects[j]->height * factorY) / 2);
		*boundRects[j] += Size(boundRects[j]->width * factorX, boundRects[j]->height * factorY);
		*boundRects[j] = *boundRects[j] & Rect(0, 0, getImageWidth(), getImageHeight());		
      //rectangle(bgrMat[currentImage], *boundRects[j], Scalar(255,0,0), 2);
	}
  classifyRegions(boundRects);
  //rectangle(bgrMat[currentImage], roi, Scalar(255,0,255), 1);
  //VisionUtils::displayImage(bgrMat[currentImage], "after prediction");
  //waitKey(0);
}

void
BallExtraction::findBallLowerCam()
{
  srand(visionModule->getModuleTime() * 100);
  vector<int> pairIndices;
  for (int i = 0; i < bottomXYPairs.size(); ++i) {
    //cout << "i: " << i << endl;
    if (find(xySeen.begin(), xySeen.end(), i) == xySeen.end()) pairIndices.push_back(
      i);
  }
  if (pairIndices.empty()) {
    xySeen.clear();
    for (int i = 0; i < bottomXYPairs.size(); ++i) {
      pairIndices.push_back(i);
    }
  }
  vector<RectPtr> boundRects;
  random_shuffle(pairIndices.begin(), pairIndices.end());
  scanRandom(boundRects, pairIndices, 2);
  filterRegions(boundRects, 25);
  classifyRegions(boundRects);
}

void
BallExtraction::findBallLowerCam(const Rect& roi)
{
  vector<RectPtr> boundRects;
  scanRandom(boundRects, roi);
  filterRegions(boundRects, 25);
  for (int j = 0; j < boundRects.size(); ++j) {
	  if (!boundRects[j])
		continue;
      //rectangle(bgrMat[currentImage], *boundRects[j], Scalar(255,0,0), 1);
	  float factorX = roi.width / boundRects[j]->width;
	  float factorY = roi.height / boundRects[j]->height;
		*boundRects[j] = *boundRects[j] - Point((boundRects[j]->width * factorX) / 2, (boundRects[j]->height * factorY) / 2);
		*boundRects[j] += Size(boundRects[j]->width * factorX, boundRects[j]->height * factorY);
		*boundRects[j] = *boundRects[j] & Rect(0, 0, getImageWidth(), getImageHeight());		
      //rectangle(bgrMat[currentImage], *boundRects[j], Scalar(255,0,0), 2);
	}
  classifyRegions(boundRects);
}

void
BallExtraction::getPredRoi(Rect& predRoi, const Mat& predState)
{
  predRoi.width = predState.at<float>(10);
  predRoi.height = predState.at<float>(11);
  predRoi.x = predState.at<float>(6) - predRoi.width / 2;
  predRoi.y = predState.at<float>(7) - predRoi.height / 2;
  predRoi = predRoi & Rect(0, 0, getImageWidth(), getImageHeight());
}

void
BallExtraction::processImage()
{
#ifdef DEBUG_BUILD
  auto tStart = high_resolution_clock::now();
#endif
  //! Whether the ball extraction class is working on the lower cam
  setImage();
  foundBall.clear();
  // Tracker prediction step.
  Mat predState = ballTracker->predict();
  vector<Mat> prev;
  ballTracker->getBallPreview(prev, 20, 0.05);
  
  if (ballTracker->getBallFound()) {
    //cout << "in prediction " << endl;
    //cout << predState << endl;
    Rect predRoi;
    getPredRoi(predRoi, predState);
    //VisionUtils::displayImage(bgrMat[currentImage](predRoi), "bgrMat[currentImage](predRoi)");
#ifdef MODULE_IS_REMOTE
    if (ballType == 0) {
      simBallDetector(predRoi);
    } else if (ballType == 1) {
      if (currentImage == TOP_CAM) {
        findBallUpperCam(predRoi);
      } else {
        findBallLowerCam(predRoi);
      }
    }
#else
    if (currentImage == TOP_CAM) {
      findBallUpperCam(predRoi);
    } else {
      findBallLowerCam(predRoi);
    }
#endif
  } else {
#ifdef MODULE_IS_REMOTE
    if (ballType == 0) {
      Rect rect = Rect(0, 0, getImageWidth(), getImageHeight());
      simBallDetector(rect);
    } else if (ballType == 1) {
      if (currentImage == TOP_CAM) {
        findBallUpperCam();
      } else {
        findBallLowerCam();
      }
    }
#else
    if (currentImage == TOP_CAM) findBallUpperCam();
    else findBallLowerCam();
#endif
  }
  if (!foundBall.empty()) xySeen.clear();
  // Tracker correction step
  ballTracker->updateFilter(foundBall);
  updateBallInfo();
  // Draw the predicted state.
  drawState(predState, Scalar(0, 255, 0));
  //VisionUtils::displayImage(bgrMat[currentImage], "ball", 1.0);
  //waitKey(0);
#ifdef DEBUG_BUILD
  //auto tEnd = high_resolution_clock::now();
  //duration<double> timeSpan = tEnd - tStart; 
  //PRINT("BallExtraction.Update.Time: " << timeSpan.count() << " seconds.")
#endif
  /*#ifdef DEBUG_BUILD
   if (GET_DVAR(int, drawBallContour)) {
   for (int i = 0; i < foundBall.size(); ++i) {
   rectangle(bgrMat[currentImage], foundBall[i], Scalar(0,0,255), 3);
   }
   }
   if (GET_DVAR(int, sendTime)) {
   high_resolution_clock::time_point tEnd = 
   high_resolution_clock::now();
   duration<double> time_span = tEnd - tStart;
   CommModule::addToLogMsgQueue("BallExtraction time: " +
   DataUtils::varToString(time_span.count()) + " seconds.");
   }
   #endif*/
}
