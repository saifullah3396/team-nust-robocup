/**
 * @file FeatureExtraction/GoalExtraction.cpp
 *
 * This file implements the class for goal extraction from 
 * the image. 
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 22 Aug 2017  
 */

#include "VisionModule/include/FeatureExtraction/GoalExtraction.h"

void
GoalExtraction::addGoalObstacle(const GoalPostPtr& goalPost)
{
  Obstacle obstacle(ObstacleType::GOALPOST);
  obstacle.center = goalPost->world;
  obstacle.leftBound = Point2f(goalPost->world.x, goalPost->world.y - 0.05);
  obstacle.rightBound = Point2f(goalPost->world.x, goalPost->world.y + 0.05);
  OVAR(ObsObstacles, VisionModule::obstaclesObs).data.push_back(obstacle);
}

void
GoalExtraction::addGoalLandmark(const GoalPostPtr& goalPost)
{
  auto p = goalPost->world;
  auto borderLines = fieldExt->getBorderLinesWorld();
  float minDist = 1000;
  int index = 0;
  // Find which line intersects with the current goal post
  for (int i = 0; i < borderLines.size(); ++i) {
    auto bl = borderLines[i];
    float dist = fabsf(bl->perp.x * p.x + bl->perp.y * p.y - bl->perpDist);
    if (dist < minDist) {
      minDist = dist;
      index = i;
    }
  }
  auto line = borderLines[index];
  float len = norm(p);
  Point2f robotToGoalU = Point2f(p.x / len, p.y / len);
  float cross = robotToGoalU.x * line->unit.y - robotToGoalU.y * line->unit.x;
  float dot = robotToGoalU.dot(line->unit);
  float perpAngle = atan2(cross, dot) + atan2(p.y, p.x);
  float cosa = cos(perpAngle);
  float sina = sin(perpAngle);
  KnownLandmarkPtr l = boost::make_shared<KnownLandmark>();
  l->type = FL_TYPE_GOAL_POST;
  l->pos = goalPost->world;
  l->poseFromLandmark.x = -cosa * p.x - sina * p.y; // From -R^t * t inverse rotation
  l->poseFromLandmark.y = +sina * p.x - cosa * p.y;
  l->poseFromLandmark.theta = -perpAngle;
  //cout << "l.pos: " << l.pos << endl;
  //cout << "l.poseFromLandmark.x: " << l.poseFromLandmark.x << endl;
  //cout << "l.poseFromLandmark.y: " << l.poseFromLandmark.y << endl;
  //cout << "l.poseFromLandmark.t: " << l.poseFromLandmark.theta * 180 / M_PI << endl;
  knownLandmarks.push_back(l);
}

void
GoalExtraction::addGoalPost(const GoalPostPtr& goalPost)
{
  addGoalObstacle(goalPost);
  addGoalLandmark(goalPost);
}

void
GoalExtraction::refreshGoalPosts()
{
  float currentTime = visionModule->getModuleTime();
  GPIter iter = goalPosts.begin();
  while (iter != goalPosts.end()) {
    if (currentTime - (*iter)->timeDetected < refreshTime) {
      ++iter;
    } else {
      iter = goalPosts.erase(iter);
    }
  }
}

bool
GoalExtraction::scanForPosts(vector<ScannedLinePtr>& verGoalLines)
{
  auto border = fieldExt->getBorder();
  auto robotRegions = robotExt->getRobotRegions();
  if (border.empty()) return false;
  srand(time(0));
  int scanStepHigh = 4, scanStepLow = 8;
  int yWhite = 150;
  int scanStart = rand() % scanStepHigh;
  for (int x = scanStart; x < getImageWidth(); x = x + scanStepHigh) {
    bool scan = true;
    for (int j = 0; j < robotRegions.size(); ++j) {
      if (!robotRegions[j]) continue;
      int min = robotRegions[j]->sr->leftBase.x;
      int max = robotRegions[j]->sr->rightBase.x;
      if (x > min && x < max) {
        scan = false;
        break;
      }
    }
    if (!scan) continue;
    int borderY = border[x].y - 10;
    int colorY = getY(x, borderY);
    if (colorY < yWhite) continue;
    int end = -1;
    int down = 0;
    for (int j = 1; j <= 10; ++j) {
      int yDown = borderY + j;
      colorY = getY(x, yDown);
      if (yDown > getImageHeight()) {
        end = getImageHeight();
        if (down > 5) {
          auto sl =
            boost::make_shared < ScannedLine > (Point(x, borderY), Point(x, end));
          verGoalLines.push_back(sl);
        }
        break;
      }

      if (colorY > yWhite && j != 10) {
        down++;
      } else {
        end = yDown;
      }
    }
    if (down > 5) {
      while (true) {
        int newEnd = end + scanStepLow;
        if (getY(x, newEnd) < yWhite || newEnd > getImageHeight()) {
          newEnd -= scanStepLow;
          break;
        }
        end = newEnd;
      }
      auto sl = boost::make_shared < ScannedLine > (Point(x, borderY), Point(
        x,
        end));
      verGoalLines.push_back(sl);
      line(bgrMat[currentImage], sl->p1, sl->p2, Scalar(255, 0, 255), 1);
    }
  }
  return true;
}

void
GoalExtraction::classifyPosts(vector<ScannedRegionPtr>& verGoalRegions)
{
  vector<GoalPostPtr> newPosts;
  for (int vgr = 0; vgr < verGoalRegions.size(); ++vgr) {
    auto gr = verGoalRegions[vgr];
    Rect r = gr->rect;
    if (r.width < 5 || r.area() < 50) continue;
    r = r - Point(r.width * 2.5 / 2, 0);
    r += Size(r.width * 2.5, 10);
    r = r & Rect(0, 0, getImageWidth(), getImageHeight());
    //rectangle(bgrMat[currentImage], r, Scalar(0,255,0), 2);
    //VisionUtils::displayImage(bgrMat[currentImage], "BGR");
    //waitKey(0);
    Mat cropped = getGrayImage()(r);
    float resizeRatio = 1.f;
    if (cropped.cols > 40) {
      resizeRatio = 40.f / (float) cropped.cols;
      resize(cropped, cropped, cv::Size(), resizeRatio, resizeRatio);
    }
    Mat white;
    threshold(cropped, white, 150, 255, 0);
    //auto tStart = high_resolution_clock::now();
    Mat gradX;
    Mat absGradX;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;
    Sobel(white, gradX, ddepth, 2, 0, 3, scale, delta, BORDER_DEFAULT);
    convertScaleAbs(gradX, absGradX);
    dilate(absGradX, absGradX, Mat(), Point(-1, -1), 2, 1, 1);

    vector<int> gradHist;
    gradHist.resize(absGradX.cols);
    int gradRowCheck = 0;
    int lastRowCheck = 0;
    int maxCheck = 0.1 * absGradX.rows;
    int height = -1;
    uchar* p = absGradX.data;
    for (unsigned i = 0; i < absGradX.cols * absGradX.rows; ++i) {
      if (*p++ > 100) {
        gradHist[i % absGradX.cols]++;
        if (height == -1 && gradRowCheck < maxCheck) gradRowCheck++;
      }
      if (height == -1 && i % absGradX.cols == 0) {
        if (gradRowCheck < maxCheck) lastRowCheck++;
        if (lastRowCheck >= 5) {
          height = i / absGradX.cols - lastRowCheck;
        }
        gradRowCheck = 0;
      }
    }

    if (height == -1) {
      height = absGradX.rows;
    }
    height /= resizeRatio;
    height += r.y;

    vector<int> filt(gradHist.size());
    for (int i = 1; i < gradHist.size() - 1; ++i) {
      filt[i - 1] = (gradHist[i - 1] + gradHist[i] + gradHist[i + 1]) / 3;
    }

    gradHist = filt;
    int maxima = 0;
    for (unsigned i = 0; i < gradHist.size(); ++i) {
      maxima = gradHist[i] > maxima ? gradHist[i] : maxima;
    }

    const int threshold = maxima * 0.75;
    const int minThreshold = maxima * 0.25;
    int grad = -1;
    vector < pair<int, int> > peaks;
    for (int i = 0; i < gradHist.size() - 1; i++) {
      if (gradHist[i] - gradHist[i + 1] > 0) {
        if (grad == 1 && gradHist[i] > threshold) {
          peaks.push_back(make_pair(i, 1));
        }
        grad = -1;
      } else if (gradHist[i + 1] - gradHist[i] > 0) {
        if (grad == -1 && gradHist[i] < minThreshold) {
          peaks.push_back(make_pair(i, 0));
        }
        grad = 1;
      }
    }

    //for (int i = 0; i < peaks.size(); ++i)
    //  cout << "peaks[" << peaks[i].first << "]:" << peaks[i].second << endl;

    if (peaks.empty()) continue;
    int unchangedStart = peaks[0].first;
    int last = -1;
    vector < pair<int, int> > peaksFilt;
    for (int i = 0; i < peaks.size(); i++) {
      if (peaks[i].second != last) {
        if (last != -1) {
          float peak = (peaks[i - 1].first + peaks[unchangedStart].first) / 2;
          peaksFilt.push_back(make_pair(peak, last));
        }
        unchangedStart = i;
      }
      last = peaks[i].second;
      if (i == peaks.size() - 1) {
        float peak = (peaks[i].first + peaks[unchangedStart].first) / 2;
        peaksFilt.push_back(make_pair(peak, last));
      }
    }

    //for (int i = 0; i < peaksFilt.size(); ++i)
    //  cout << "peaksFilt[" << peaksFilt[i].first << "]:" << peaksFilt[i].second << endl;

    if (peaksFilt.empty()) continue;
    //VisionUtils::displayImage(absGradX, "gradX", 1);
    //waitKey(0);
    int imageCenter = absGradX.cols / 2;
    last = peaksFilt[0].second;
    int peak1 = -1;
    int peak2 = -1;
    for (int i = 1; i < peaksFilt.size(); i++) {
      if (peaksFilt[i].second == 0) {
        if (last == 1) {
          if (peaksFilt[i - 1].first < imageCenter) {
            peak1 = peaksFilt[i - 1].first;
          }
        }
      } else {
        if (last == 0) {
          if (peak1 != -1) {
            if (peak2 == -1 && peaksFilt[i].first > imageCenter) peak2 =
              peaksFilt[i].first;
          } else {
            peak1 = peaksFilt[i].first;
          }
        }
      }
      last = peaksFilt[i].second;
    }

    if (peak1 == -1 || peak2 == -1) continue;

    peak1 /= resizeRatio;
    peak2 /= resizeRatio;

    int diff = peak2 - peak1;
    if (diff < 5) continue;
    r.x = r.x + peak1;
    r.y = gr->rect.y;
    r.width = diff;
    r.height = gr->rect.height;
    int winX = r.x;
    int winSizeX = r.width;
    int winSizeY = 10;
    int winY = r.y + r.height;
    Rect window = Rect(winX, winY, winSizeX, winSizeY);
    window = window & Rect(0, 0, getImageWidth(), getImageHeight());
    rectangle(bgrMat[currentImage], window, Scalar(255, 255, 255), 2);
    int whiteCnt = 0;
    int greenCnt = 0;
    int totalCnt = winSizeX * winSizeY / 4;
    for (int j = winY; j < winSizeY + winY; j = j + 4) {
      for (int k = winX; k < winSizeX + winX; k = k + 4) {
        auto p = getYUV(k, j);
        if (colorHandler->isColor(p, Colors::WHITE)) {
          whiteCnt++;
        } else if (colorHandler->isColor(p, Colors::GREEN)) {
          greenCnt++;
        }
      }
    }
    //cout << "i: " << i << endl;    
    //cout << "totalCnt: " << (float) totalCnt << endl;
    //cout << "white: " << (float) whiteCnt << endl;
    //cout << "otherCnt/totalCnt: " << (float) greenCnt / (float) totalCnt << endl;
    if ((float) greenCnt / (float) totalCnt > 0.1) {
      vector<Point2f> goalBases, goalBasesWorld;
      goalBases.push_back(Point2f(r.x, winY));
      goalBases.push_back(Point2f(r.x + r.width, winY));
      //VisionUtils::drawPoint(Point(r.x, winY), bgrMat[currentImage], Scalar(255,255,0));
      //VisionUtils::drawPoint(Point(r.x + r.width, winY), bgrMat[currentImage], Scalar(255,255,0));
      cameraTransform->imageToWorld(
        currentImage,
        goalBasesWorld,
        goalBases,
        0.0);
      float width = norm(goalBasesWorld[0] - goalBasesWorld[1]);
      //cout << "width: " << width << endl;
      if (width > 0.075 && width < 0.20) {
        auto cW = Point2f((goalBasesWorld[0].x + goalBasesWorld[1].x) / 2, // center world
        (goalBasesWorld[0].y + goalBasesWorld[1].y) / 2);
        auto cI = Point2f(r.x + r.width / 2, winY);
        auto gp = boost::make_shared < GoalPost > (cW, cI);
        bool exists = false;
        for (int i = 0; i < goalPosts.size(); ++i) {
          if (goalPosts[i]->checkDuplicate(gp)) {
            exists = true;
            break;
          }
        }
        if (!exists) newPosts.push_back(gp);
        rectangle(bgrMat[currentImage], r, Scalar(0, 0, 255), 2);
      }
    }
    if (vgr >= 2) break;
  }
  goalPosts.insert(goalPosts.end(), newPosts.begin(), newPosts.end());
}

void
GoalExtraction::findBestPosts()
{
  if (!goalPosts.empty()) {
    if (goalPosts.size() > 1) {
      for (int i = 0; i < goalPosts.size(); ++i) {
        if (!goalPosts[i]) continue;
        auto gp1 = goalPosts[i]->image;
        for (int j = 0; j < goalPosts.size(); ++j) {
          if (!goalPosts[j]) continue;
          if (i != j) {
            auto gp2 = goalPosts[j]->image;
            if (norm(gp1 - gp2) < 75) {
              if (gp1.y > gp2.y) {
                goalPosts[j].reset();
              }
            }
          }
        }
      }
      GPIter iter = goalPosts.begin();
      while (iter != goalPosts.end()) {
        if (*iter) {
          ++iter;
        } else {
          iter = goalPosts.erase(iter);
        }
      }
      if (goalPosts.empty()) return;

      if (goalPosts.size() > 1) { // If greater than 1, find the right pair of posts.
        vector<int> bestIds(2, -1);
        float bestDist = 0;
        for (int i = 0; i < goalPosts.size(); ++i) {
          if (!goalPosts[i]) continue;
          for (int j = 0; j < goalPosts.size(); ++j) {
            if (!goalPosts[j]) continue;
            if (i != j) {
              float dist = norm(goalPosts[i]->world - goalPosts[j]->world);
              if (dist > 1.4 && dist < 1.8) {
                float d1 = abs(dist - 1.5);
                float d2 = abs(bestDist - 1.5);
                if (d1 < d2) {
                  bestDist = dist;
                  bestIds[0] = i;
                  bestIds[1] = j;
                }
              }
            }
          }
        }

        if (bestIds[0] != -1) {
          vector<GoalPostPtr> bestPosts;
          for (int i = 0; i < bestIds.size(); ++i) {
            bestPosts.push_back(goalPosts[bestIds[i]]);
          }
          goalPosts = bestPosts;
        }
      } else {
        addGoalPost(goalPosts[0]);
      }
    } else {
      addGoalPost(goalPosts[0]);
    }
  }
}

void
GoalExtraction::updateGoalInfo()
{
  GoalInfo goalInfo;
  if (IVAR(bool, VisionModule::robotOnSideLine)) {
    if (!goalPosts.empty()) {
      if (goalPosts.size() == 2) {
        Point2f goalMid(0.f, 0.f);
        goalMid += goalPosts[0]->world;
        goalMid += goalPosts[1]->world;
        goalMid = Point2f(goalMid.x / 2, goalMid.y / 2);
        // Max norm for 3x4.5 distance from sidelines
        goalInfo.ours = norm(goalMid) < 5.4083 ? 1 : 0;
      } else if (goalPosts.size() == 1) {
        // Max norm for 3.8x4.5 distance from sidelines
        goalInfo.ours = norm(goalPosts[0]->world) < 5.889821729 ? 1 : 0;
      }
      if (goalInfo.ours == 1) {
        for (int i = 0; i < goalPosts.size(); ++i) {
          if (goalPosts[i]->world.y < 0) {
            if (goalPosts[i]->world.x > 3.5 && goalPosts[i]->world.x < 4.1) // Goal on Right - Left Post
            {
              goalInfo.leftPost = goalPosts[i]->world;
            } else if (goalPosts[i]->world.x > 1.9 && goalPosts[i]->world.x < 2.5) // Goal on Right - Right Post
            {
              goalInfo.rightPost = goalPosts[i]->world;
            }
          } else {
            if (goalPosts[i]->world.x > 3.5 && goalPosts[i]->world.x < 4.1) // Goal on Left - Right Post
            {
              goalInfo.rightPost = goalPosts[i]->world;
            } else if (goalPosts[i]->world.x > 1.9 && goalPosts[i]->world.x < 2.5) // Goal on Left - Left Post
            {
              goalInfo.leftPost = goalPosts[i]->world;
            }
          }
          //LOG_INFO("Goal post: " << goalPosts[i]->world << endl)
        }
      } else if (goalInfo.ours == 0) {
        for (int i = 0; i < goalPosts.size(); ++i) {
          if (goalPosts[i]->world.y > 0) {
            if (goalPosts[i]->world.x > 3.5 && goalPosts[i]->world.x < 4.1) // Goal on Left - Right Post
            {
              goalInfo.rightPost = goalPosts[i]->world;
            } else if (goalPosts[i]->world.x > 1.9 && goalPosts[i]->world.x < 2.5) // Goal on Left - Left Post
            {
              goalInfo.leftPost = goalPosts[i]->world;
            }
          } else {
            if (goalPosts[i]->world.x > 3.5 && goalPosts[i]->world.x < 4.1) // Goal on Left - Left Post
            {
              goalInfo.leftPost = goalPosts[i]->world;
            } else if (goalPosts[i]->world.x > 1.9 && goalPosts[i]->world.x < 2.5) // Goal on Left - Right Post
            {
              goalInfo.rightPost = goalPosts[i]->world;
            }
          }
        }
      }
      goalInfo.found = true;
    } else {
      goalInfo.found = false;
    }
  } else {
    if (goalPosts.size() == 2) {
      if (goalPosts[0]->image.x < goalPosts[1]->image.x) { // 0 is to the left in image
        goalInfo.leftPost = goalPosts[0]->world;
        goalInfo.rightPost = goalPosts[1]->world;
      } else {
        goalInfo.leftPost = goalPosts[1]->world;
        goalInfo.rightPost = goalPosts[0]->world;
      }
      findGoalSide(goalInfo);
      goalInfo.found = true;
      for (int i = 0; i < goalPosts.size(); ++i) {
        Obstacle obstacle(ObstacleType::GOALPOST);
        obstacle.center = goalPosts[i]->world;
        obstacle.leftBound = Point2f(
          goalPosts[i]->world.x,
          goalPosts[i]->world.y - 0.05);
        obstacle.rightBound = Point2f(
          goalPosts[i]->world.x,
          goalPosts[i]->world.y + 0.05);
        OVAR(ObsObstacles, VisionModule::obstaclesObs).data.push_back(obstacle);
      }
    } else {
      goalInfo.found = false;
    }
  }
  float currentTime = visionModule->getModuleTime();
  for (int i = 0; i < goalPosts.size(); ++i)
    if (goalPosts[i]->refresh) goalPosts[i]->timeDetected = currentTime;
  goalInfo.assignId();
  OVAR(GoalInfo, VisionModule::goalInfo) = goalInfo;
}

void
GoalExtraction::findGoalSide(GoalInfo& goalInfo)
{
  goalInfo.ours = -1;
  Point2f goalMid(0.f, 0.f);
  goalMid += goalInfo.leftPost;
  goalMid += goalInfo.rightPost;
  goalMid = Point2f(goalMid.x / 2, goalMid.y / 2);
  goalInfo.mid = goalMid;
  Point2f goalUnit(0.f, 0.f);
  goalUnit = goalInfo.rightPost - goalInfo.leftPost;
  float goalDiffNorm = norm(goalUnit);
  goalUnit.x = goalUnit.x / goalDiffNorm;
  goalUnit.y = goalUnit.y / goalDiffNorm;

  float goalMidNorm = norm(goalMid);
  Point2f robotToGoalU = Point2f(
    goalMid.x / goalMidNorm,
    goalMid.y / goalMidNorm);
  float cross = robotToGoalU.x * goalUnit.y - robotToGoalU.y * goalUnit.x;
  float dot = robotToGoalU.dot(goalUnit);
  float perpAngle = atan2(cross, dot) + atan2(goalMid.y, goalMid.x);
  float cosa = cos(perpAngle);
  float sina = sin(perpAngle);

  goalInfo.poseFromGoal.x = -cosa * goalMid.x - sina * goalMid.y; // From -R^t * t inverse rotation
  goalInfo.poseFromGoal.y = +sina * goalMid.x - cosa * goalMid.y;
  goalInfo.poseFromGoal.theta = -perpAngle;

  //cout << "goalInfo.poseFromLandmark.x: " << goalInfo.poseFromGoal.x << endl;
  //cout << "goalInfo.poseFromLandmark.y: " << goalInfo.poseFromGoal.y << endl;
  //cout << "goalInfo.poseFromLandmark.t: " << goalInfo.poseFromGoal.theta * 180 / M_PI << endl;

  // Assumes that goalInfo has both left post and right post determined
  auto robotRegions = robotExt->getRobotRegions();
  float closest = 1000;
  float distFromCenter = -1;
  int minIndex = -1;
  for (int i = 0; i < robotRegions.size(); ++i) {
    if (!robotRegions[i]) continue;
    Point2f goalToOtherRobot = goalMid - robotRegions[i]->world;
    float goalToPerp = abs(
      goalToOtherRobot.x * goalUnit.x + goalToOtherRobot.y * goalUnit.y);
    float gR = norm(goalToOtherRobot);
    float dist = sqrt(gR * gR - goalToPerp * goalToPerp);
    // The perpendicular distance from robot to goal line 
    //cout << "dist[" << i << "]: " << dist << endl;
    if (dist < closest) {
      closest = dist;
      distFromCenter = goalToPerp;
      minIndex = i;
    }
  }
  if (minIndex != -1) {
    //cout << "Robot found between goal posts" << endl;
    //cout << "distFromCenter: " << distFromCenter << endl;
    //cout << "dist:" << closest << endl;
    // If robot is within 80cm of the goal mid along the goal posts 
    // in either direction and 65cm in front of it then mark it as goal keeper
    if (closest < 0.65 && distFromCenter < 0.8) {
      if (robotRegions[minIndex]->ourTeam) {
        goalInfo.ours = 1;
      } else {
        goalInfo.ours = 0;
      }
    }
  }
}

void
GoalExtraction::processImage()
{
  if (!fieldExt->isFound()) return;
#ifdef DEBUG_BUILD
  auto tStart = high_resolution_clock::now();
#endif
  refreshGoalPosts();
  vector<ScannedLinePtr> verGoalLines;
  //LOG_INFO("Scanning goal posts...")
  if (!scanForPosts(verGoalLines)) return;
#ifdef DEBUG_BUILD
  auto tEnd = high_resolution_clock::now();
  duration<double> timeSpan = tEnd - tStart;
  //LOG_INFO("GoalExtraction.ScanForPosts.Time: " << timeSpan.count() << " seconds.")
  tStart = high_resolution_clock::now();
#endif
  vector<ScannedRegionPtr> verGoalRegions;
  ////LOG_INFO("Finding goal regions...")
  findRegions(verGoalRegions, verGoalLines, 8, 8, false);
#ifdef DEBUG_BUILD
  tEnd = high_resolution_clock::now();
  timeSpan = tEnd - tStart;
  //LOG_INFO("GoalExtraction.FindRegions.Time: " << timeSpan.count() << " seconds.")
  tStart = high_resolution_clock::now();
#endif
  ////LOG_INFO("Classifying goal regions...")
  classifyPosts(verGoalRegions);
#ifdef DEBUG_BUILD
  tEnd = high_resolution_clock::now();
  timeSpan = tEnd - tStart;
  //LOG_INFO("GoalExtraction.ClassifyPosts.Time: " << timeSpan.count() << " seconds.")
  tStart = high_resolution_clock::now();
#endif
  ////LOG_INFO("Finding best posts...")
  findBestPosts();
#ifdef DEBUG_BUILD
  tEnd = high_resolution_clock::now();
  timeSpan = tEnd - tStart;
  //LOG_INFO("GoalExtraction.FindBestPosts.Time: " << timeSpan.count() << " seconds.")
  tStart = high_resolution_clock::now();
#endif
  updateGoalInfo();
#ifdef DEBUG_BUILD
  tEnd = high_resolution_clock::now();
  timeSpan = tEnd - tStart;
  //LOG_INFO("GoalExtraction.UpdateGoalInfo.Time: " << timeSpan.count() << " seconds.")
  //tStart = high_resolution_clock::now();
#endif
  //cout << "goalPosts.size(): " << goalPosts.size() << endl;
  //auto tEnd = high_resolution_clock::now();
  //duration<double> timeSpan = tEnd - tStart; 
  //LOG_INFO("GoalExtraction.Update.Time: " << timeSpan.count() << " seconds.")
  /*#ifdef DEBUG_BUILD  
   if (GET_DVAR(int, drawGoalPostBase)) {
   for (int i = 0; i < bestGoalPosts.size(); ++i)
   VisionUtils::drawPoint(bestGoalPosts[i][0], bgrMat[currentImage], Scalar(0,0,0));
   }
   if (GET_DVAR(int, sendTime)) {
   auto tEnd =	high_resolution_clock::now();
   duration<double> timeSpan = tEnd - tStart;
   CommModule::addToLogMsgQueue("GoalExtraction time: " +
   DataUtils::varToString(timeSpan.count()) + " seconds.");
   }
   #endif*/
  //cout << "verGoalRegions.size(): " << verGoalRegions.size() << endl;
  //drawRegions(bgrMat[currentImage], verGoalRegions, Scalar(0,255,255));
  //cout << "Regions: " << verGoalRegions.size() << endl;
  //VisionUtils::displayImage(bgrMat[currentImage], "BGR", 1.0);
}
//---------------------------------______GOAL EXTRACTION_____---------------------------//
