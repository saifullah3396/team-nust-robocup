/**
 * @file FeatureExtraction/FieldExtraction.cpp
 *
 * This file implements the class for field extraction from the image.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author AbdulRehman
 * @date 22 Aug 2017
 */

#include "VisionModule/include/FeatureExtraction/FieldExtraction.h"

void
FieldExtraction::processImage()
{
#ifdef DEBUG_BUILD
  auto tStart = high_resolution_clock::now();
#endif

  auto borderPoints = regionSeg->getBorderPoints();
  auto avgHeight = regionSeg->getFieldAvgHeight();
  auto minBestHeight = regionSeg->getFieldMinBestHeight();
  int cutoffHeight = 0;
  sort(
    borderPoints.begin(),
    borderPoints.end(),
    [](const Point& p1, const Point& p2) {
      return (p1.x < p2.x) ||
      ((p1.x == p2.x) && (p1.y < p2.y));
    });
  vector < Point > borderPointsFilterBelow;
  if (!borderPoints.empty()) {
    borderPointsFilterBelow.push_back(borderPoints[0]);
    for (int i = 1; i < borderPoints.size(); ++i) {
      //VisionUtils::drawPoint(borderPoints[i], bgrMat[currentImage], Scalar(255,255,255)); 
      if (borderPointsFilterBelow.back().x == borderPoints[i].x) {
        continue;
      } else {
        borderPointsFilterBelow.push_back(borderPoints[i]);
      }
    }
  }

  vector < Point > borderPointsFilterAbove;
  //cout << "c1: " << minBestHeight << endl;
  if (abs(minBestHeight - avgHeight) < 150) {
    //cout << "HERE" << endl;	
    cutoffHeight = minBestHeight - 25;
  } else {
    cutoffHeight = avgHeight - 100;
  }
  //cout << "c2: " << cutoffHeight << endl;
  for (int i = 0; i < borderPointsFilterBelow.size(); ++i) {
    //VisionUtils::drawPoint(borderPointsFilterBelow[i], bgrMat[currentImage], Scalar(0,255,255));
    if (borderPointsFilterBelow[i].y > cutoffHeight) {
      borderPointsFilterAbove.push_back(borderPointsFilterBelow[i]);
      //VisionUtils::drawPoint(borderPointsFilterBelow[i], bgrMat[currentImage], Scalar(255,0,255));
    }
  }

#ifdef DEBUG_BUILD
  auto tEnd = high_resolution_clock::now();
  duration<double> timeSpan = tEnd - tStart;
  //LOG_INFO("FieldExtraction.FilterPoints.Time: " << timeSpan.count() << " seconds.");
  tStart = high_resolution_clock::now();
#endif

  fieldHeight = getImageHeight();
  vector < Point > maskPoints;
  border.clear();
  borderLines.clear();
  borderLinesWorld.clear();
  fieldFound = false;
  mt19937 rng;
  uniform_real_distribution<> dist
    { 0, 1 };
  if (borderPointsFilterAbove.size() >= 6) {
    float bestNx = 0;
    float bestNy = 0;
    float bestD = 0;
    int maxSum = 0;
    int pointsCnt = 0;
    for (int i = 0; i < 200; i++) {
      Point2d p1 =
        borderPointsFilterAbove[(int) (dist(rng) * borderPointsFilterAbove.size())];
      Point2d p2 =
        borderPointsFilterAbove[(int) (dist(rng) * borderPointsFilterAbove.size())];
      if (p1.x != p2.x || p1.y != p2.y) {
        if (p1.x > p2.x) {
          Point2d tmp = p1;
          p1 = p2;
          p2 = tmp;
        }
        int dx = p1.x - p2.x;
        int dy = p1.y - p2.y;
        float len = sqrt(dx * dx + dy * dy);
        if (len < 16) continue;
        float nx = -dy / len;
        float ny = dx / len;
        float d = nx * p1.x + ny * p1.y;
        int sum = 0;
        int cnt = 0;
        for (int j = 0; j < borderPointsFilterAbove.size(); ++j) {
          Point2d p = borderPointsFilterAbove[j];
          float dist = fabsf(nx * p.x + ny * p.y - d);
          if (dist < 4.f) {
            cnt++;
            sum += 1;
          } else if (dist > 4.f && dist < 32) {
            sum -= 1;
          }
        }
        if (sum > maxSum) {
          maxSum = sum;
          pointsCnt = cnt;
          bestNx = nx;
          bestNy = ny;
          bestD = d;
        }
      }
    }

    // if enough points left, search for the best second line matching for the
    // rest of the border points with RANSAC algorithm
    if (pointsCnt >= 5) {
      vector < Point2d > pointsLeft;
      vector < Point2d > pointsDone;
      for (int j = 0; j < borderPointsFilterAbove.size(); ++j) {
        Point2d p = borderPointsFilterAbove[j];
        float dist = fabsf(bestNx * p.x + bestNy * p.y - bestD);
        if (dist >= 4.f) {
          pointsLeft.push_back(p);
        } else {
          pointsDone.push_back(p);
        }
      }
      float bestNx2 = 0;
      float bestNy2 = 0;
      float bestD2 = 0;
      int pointsCnt2 = 0;
      if (pointsLeft.size() >= 4) {
        int maxSum2 = 0;
        for (int i = 0; i < 200; i++) {
          Point2d p1 = pointsLeft[(int) (dist(rng) * pointsLeft.size())];
          Point2d p2 = pointsLeft[(int) (dist(rng) * pointsLeft.size())];
          if (p1.x != p2.x || p1.y != p2.y) {
            if (p1.x > p2.x) {
              Point2d tmp = p1;
              p1 = p2;
              p2 = tmp;
            }
            int dx = p1.x - p2.x;
            int dy = p1.y - p2.y;
            float len = sqrtf(dx * dx + dy * dy);
            if (len < 16) continue;
            float nx = -dy / len;
            float ny = dx / len;
            float d = nx * p1.x + ny * p1.y;
            int sum = 0;
            int cnt = 0;
            for (int j = 0; j < pointsLeft.size(); ++j) {
              Point2d p = pointsLeft[j];
              float dist = nx * p.x + ny * p.y - d;
              if (fabsf(dist) < 4) {
                sum += 1;
                cnt++;
              } else if (dist > 4 && dist < 32) {
                sum -= 1;
              }
            }
            for (int j = 0; j < pointsDone.size(); ++j) {
              Point2d p = pointsDone[j];
              float dist = nx * p.x + ny * p.y - d;
              if (dist > 4) {
                sum -= 2;
              }
            }
            if (sum > maxSum2) {
              maxSum2 = sum;
              pointsCnt2 = cnt;
              bestNx2 = nx;
              bestNy2 = ny;
              bestD2 = d;
            }
          }
        }
      }
      Vec4f best = Vec4f(
        0,
        bestD / bestNy,
        getImageWidth() - 1,
        (bestD - bestNx * (getImageWidth() - 1)) / bestNy);
      if (bestNy2 == 0) {
        borderLines.push_back(best);
      } else {
        Vec4f best2 = Vec4f(
          0,
          bestD2 / bestNy2,
          getImageWidth() - 1,
          (bestD2 - bestNx2 * (getImageWidth() - 1)) / bestNy2);
        Point2d intersection = findIntersection(best, best2);
        if (intersection.x > 0 && intersection.x < getImageWidth() && intersection.y > 0 && intersection.y < getImageHeight()) {
          VisionUtils::drawPoint(intersection, bgrMat[currentImage]);
          if (best[1] > best2[1]) {
            borderLines.push_back(
              Vec4f(best[0], best[1], intersection.x, intersection.y));
            borderLines.push_back(
              Vec4f(intersection.x, intersection.y, best2[2], best2[3]));
          } else {
            borderLines.push_back(
              Vec4f(best2[0], best2[1], intersection.x, intersection.y));
            borderLines.push_back(
              Vec4f(intersection.x, intersection.y, best[2], best[3]));
          }
        } else {
          borderLines.push_back(best);
        }
      }
      //interpolate the field-border from one or two lines
      for (int x = 0; x < getImageWidth(); x++) {
        if (bestNy == 0) continue;
        int y1 = (int) ((bestD - bestNx * x) / bestNy);
        if (pointsCnt2 >= 4 && bestNy2 != 0) {
          int y2 = (int) ((bestD2 - bestNx2 * x) / bestNy2);
          if (y2 > y1) y1 = y2;
        }
        if (y1 < 0) y1 = 0;
        if (y1 >= getImageHeight() - 2) y1 = getImageHeight() - 2;
        border.push_back(Point(x, y1));
        maskPoints.push_back(Point(x, y1 - 10));
        fieldHeight = y1 < fieldHeight ? y1 : fieldHeight;
      }
    } /*else {
     
     // interpolate the field-border with default values (if no field border is
     // visible)
     for (int x = 0; x < getImageWidth(); x++) {
     border.push_back(Point(x, 0));  // 0 means a field border on the top of the
     // image, so that all pixels below are valid
     // field pixels
     maskPoints.push_back(Point(x, 0));
     }
     }*/
  }
#ifdef DEBUG_BUILD
  tEnd = high_resolution_clock::now();
  timeSpan = tEnd - tStart;
  //LOG_INFO("FieldExtraction.RANSAC.Time: " << timeSpan.count() << " seconds.");
  tStart = high_resolution_clock::now();
#endif
  maskPoints.insert(maskPoints.begin(), Point(0, getImageHeight()));
  maskPoints.push_back(Point(getImageWidth(), getImageHeight()));
  fillConvexPoly(mask, &maskPoints[0], (int) maskPoints.size(), 255, 8, 0);
  if (fieldHeight >= getImageHeight()) fieldHeight = 0;

  if (fieldHeight != 0) {
    vector < Point2f > imgPoints;
    vector < Point2f > worldPoints;
    imgPoints.push_back(Point(0, fieldHeight));
    imgPoints.push_back(Point(getImageWidth(), fieldHeight));
    imgPoints.push_back(Point(getImageWidth(), getImageHeight()));
    imgPoints.push_back(Point(0, getImageHeight()));
    //cout << "fieldext" << endl;
    //cout << "img: " << endl << imgPoints << endl;
    cameraTransform->imageToWorld(currentImage, worldPoints, imgPoints, 0);
    fieldRect.x = 0;
    fieldRect.y = fieldHeight;
    fieldRect.width = getImageWidth();
    fieldRect.height = getImageHeight() - fieldHeight;
    fieldFound = true;
  } else {
    return;
  }
  //tEnd = high_resolution_clock::now();
  //timeSpan = tEnd - tStart; 
  //LOG_INFO("FieldExtraction.Transform.Time: " << timeSpan.count() << " seconds.");
  //tStart = high_resolution_clock::now();
  for (size_t i = 0; i < borderLines.size(); ++i) {
    auto l = borderLines[i];
    //line(bgrMat[currentImage], Point(l[0],l[1]), Point(l[2],l[3]), Scalar(255,255,0), 2);
    if (l[0] < 10 && l[2] < 10) continue;
    if (l[0] > getImageWidth() - 10 && l[2] > getImageWidth() - 10) continue;
    Point2f wp1, wp2;
    cameraTransform->imageToWorld(currentImage, wp1, Point2f(l[0], l[1]), 0.f);
    cameraTransform->imageToWorld(currentImage, wp2, Point2f(l[2], l[3]), 0.f);
    auto diff = wp2 - wp1;
    float d = norm(diff);
    auto unit = Point2f(diff.x / d, diff.y / d);
    FittedLinePtr fl = boost::make_shared < FittedLine > (wp1, wp2);
    fl->unit = unit;
    fl->d = d;
    fl->diff = diff;
    fl->perp.x = -fl->diff.y / fl->d;
    fl->perp.y = fl->diff.x / fl->d;
    fl->perpDist = fl->perp.x * fl->p1.x + fl->perp.y * fl->p1.y;
    borderLinesWorld.push_back(fl);
  }
#ifdef DEBUG_BUILD
  //tEnd = high_resolution_clock::now();
  //timeSpan = tEnd - tStart; 
  //LOG_INFO("FieldExtraction.borderLines.Time: " << timeSpan.count() << " seconds.");
#endif
  //cout << "world: " << endl << worldPoints << endl;
  //VisionUtils::drawPoints(imgPoints, bgrMat[currentImage]);
  //cameraTransform->imageToWorld(currentImage, worldPoints, imgPoints, 0);
  if (!border.empty()) polylines(
    bgrMat[currentImage],
    border,
    false,
    Scalar(255, 0, 0, 8),
    2);
  //if(!border.empty())
  //polylines(bgrMat[currentImage], border, false, Scalar(255, 0, 0, 8), 2);
  //for (int i = 0; i < borderLines.size(); ++i)
  //{
  //  auto l = borderLines[i];
  //  line(bgrMat[currentImage], Point(l[0],l[1]), Point(l[2],l[3]), Scalar(0,0,255), 2); 
  //}
  //VisionUtils::displayImage(bgrMat[currentImage], "field");
  // waitKey(0);
  /*#ifdef DEBUG_BUILD
   if (GET_DVAR(int, drawBorder)) {
   if(!border.empty())
   polylines(bgrMat[currentImage], border, false, Scalar(255, 0, 0, 8), 2);
   }
   if (GET_DVAR(int, drawBorderLines)) {
   for (int i = 0; i < borderLines.size(); ++i) {
   Vec4i l = borderLines[i];
   line(
   bgrMat[currentImage],
   Point(l[0], l[1]),
   Point(l[2], l[3]),
   Scalar(255,0,0),
   2
   );
   }
   }
   if (GET_DVAR(int, sendTime)) {
   high_resolution_clock::time_point tEnd =
   high_resolution_clock::now();
   duration<double> timeSpan = tEnd - tStart;
   CommModule::addToLogMsgQueue("FieldExtraction time: " +
   DataUtils::varToString(timeSpan.count()) + " seconds.");
   }
   #endif*/
}
