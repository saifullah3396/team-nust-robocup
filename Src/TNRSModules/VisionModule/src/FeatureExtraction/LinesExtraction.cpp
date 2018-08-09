/**
 * @file FeatureExtraction/LinesExtraction.h
 *
 * This file declares the class for field corners extraction from
 * the image.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 22 Aug 2017 
 */

#include <chrono>
#include "VisionModule/include/FeatureExtraction/LinesExtraction.h"

Point2f
LinesExtraction::findIntersection(const FittedLinePtr& l1,
  const FittedLinePtr& l2)
{
  auto p1 = l1->p1;
  auto p2 = l1->p2;
  auto p3 = l2->p1;
  auto p4 = l2->p2;
  auto p1p2 = p1 - p2;
  auto p3p4 = p3 - p4;
  auto det = p1p2.x * p3p4.y - p1p2.y * p3p4.x;
  auto c1 = p1.x * p2.y - p1.y * p2.x;
  auto c2 = p3.x * p4.y - p3.y * p4.x;
  if (det != 0.0) {
    //! det == 0 -> Lines are parallel
    auto pX = (c1 * p3p4.x - p1p2.x * c2) / det;
    auto pY = (c1 * p3p4.y - p1p2.y * c2) / det;
    return Point2f(pX, pY);
  } else {
    return Point2f(-100, -100);
  }
}

bool
LinesExtraction::checkEllipse(Ellipse& e)
{
  float ratio = e.rs / e.rl;
  if (ratio < 0.8 && ratio > 1.2) return false;
  if (e.rl < 0.45 || e.rs < 0.45 || e.rl > 0.9 || e.rs > 0.9) return false;
  return true;
}

bool
LinesExtraction::scanForEdges(vector<vector<ScannedEdgePtr> >& connectedEdges)
{
  auto border = fieldExt->getBorder();
  auto fHeight = fieldExt->getFieldHeight();
  auto robotRegions = robotExt->getRobotRegions();
  if (border.empty()) {
    return false;
  }
  vector < Point2f > verImagePoints;
  vector < Point2f > horImagePoints;
  vector < Point2f > verWorldPoints;
  vector < Point2f > horWorldPoints;
  //lineEdges.clear();// = regionSeg->getLineEdges();  

  srand(time(0));
  int scanStepHigh = 24, scanStepLow = 2;
  int scanStart = rand() % scanStepHigh + fHeight;
  for (int y = scanStart; y < getImageHeight(); y = y + scanStepHigh) {
    //uchar* p = whiteEdges.ptr<uchar>(y);
    int horStartLine = -1;
    int horStartLow = rand() % scanStepLow;
    for (int x = horStartLow; x < getImageWidth(); x = x + scanStepLow) {
      bool scan = true;
      for (int j = 0; j < robotRegions.size(); ++j) {
        if (!robotRegions[j]) continue;
        if (robotRegions[j]->sr->rect.contains(Point(x, y))) {
          scan = false;
          break;
        }
      }
      if (!scan) continue;
      auto color = getYUV(x, y);
      //! Horizontal Scanning
      if (colorHandler->isColor(color, Colors::WHITE)) {
        if (horStartLine == -1) horStartLine = x;
        if (horStartLine != -1) {
          int diff = abs(x - horStartLine);
          if (diff >= 24) {
            horStartLine = -1;
          }/*
           int index = (horStartLine + x - scanStepLow) / 2;
           p[index] = 255;
           auto se = 
           boost::make_shared<ScannedEdge>(Point(index, y));
           lineEdges.push_back(se);
           horStartLine = -1;*/
        }
      } else {
        if (horStartLine != -1) {
          int diff = abs(x - horStartLine);
          if (diff >= 0) {
            int index;
            if (diff == 0) {
              index = horStartLine;
            } else {
              index = (horStartLine + x - scanStepLow) / 2;
            }
            //p[index] = 255;
            horImagePoints.push_back(Point2f(index, y));
            //auto se = 
            //  boost::make_shared<ScannedEdge>(Point(index, y));
            //horLineEdges.push_back(se);
          }
        }
        horStartLine = -1;
      }
    }
  }

  scanStart = rand() % scanStepHigh;
  for (int x = scanStart; x < getImageWidth(); x = x + scanStepHigh) {
    int verStartLine = -1;
    int verStartLow = rand() % scanStepLow + border[x].y;
    for (int y = verStartLow; y < getImageHeight(); y = y + scanStepLow) {
      bool scan = true;
      for (int j = 0; j < robotRegions.size(); ++j) {
        if (!robotRegions[j]) continue;
        if (robotRegions[j]->sr->rect.contains(Point(x, y))) {
          scan = false;
          break;
        }
      }
      if (!scan) continue;
      auto color = getYUV(x, y);
      if (colorHandler->isColor(color, Colors::WHITE)) {
        if (verStartLine == -1) {
          verStartLine = y;
        }
        if (verStartLine != -1) {
          float diff = y - verStartLine;
          if (diff >= 24) {
            //int index = (verStartLine + y - scanStepLow) / 2;
            //uchar* p = whiteEdges.ptr<uchar>(index);
            //p[x] = 255;
            //auto se = 
            // boost::make_shared<ScannedEdge>(Point2f(x, index));
            //lineEdges.push_back(se);
            verStartLine = -1;
          }
        }
      } else {
        if (verStartLine != -1) {
          float diff = y - verStartLine;
          if (diff >= 1) {
            int index;
            if (diff == 0) {
              index = verStartLine;
            } else {
              index = (verStartLine + y - scanStepLow) / 2;
            }
            //uchar* p = whiteEdges.ptr<uchar>(index);
            //p[x] = 255;
            verImagePoints.push_back(Point2f(x, index));
            //auto se = 
            // boost::make_shared<ScannedEdge>(Point2f(x, index));
            //verLineEdges.push_back(se);
          }
        }
        verStartLine = -1;
      }
    }
  }

  vector<ScannedEdgePtr> verLineEdges;
  vector<ScannedEdgePtr> horLineEdges;
  if (!verImagePoints.empty()) {
    cameraTransform->imageToWorld(
      currentImage,
      verWorldPoints,
      verImagePoints,
      0.0);
    for (int i = 0; i < verWorldPoints.size(); ++i) {
      auto se =
        boost::make_shared < ScannedEdge > (verImagePoints[i], verWorldPoints[i]);
      verLineEdges.push_back(se);
    }
    findConnectedEdges(connectedEdges, verLineEdges, 30, 30, true);
  }
  if (!horImagePoints.empty()) {
    cameraTransform->imageToWorld(
      currentImage,
      horWorldPoints,
      horImagePoints,
      0.0);
    for (int i = 0; i < horWorldPoints.size(); ++i) {
      auto se =
        boost::make_shared < ScannedEdge > (horImagePoints[i], horWorldPoints[i]);
      horLineEdges.push_back(se);
    }
    findConnectedEdges(connectedEdges, horLineEdges, 30, 30, false);
  }

  //VisionUtils::drawPoints(horImagePoints, bgrMat[currentImage]);
  //imshow("horizontal", bgrMat[currentImage]);
  //waitKey(0);
  //VisionUtils::drawPoints(verImagePoints, bgrMat[currentImage]);
  //imshow("vertical", bgrMat[currentImage]);
  //waitKey(0);

  //cout << "connectedEdges.size():"  << connectedEdges.size() << endl;
  //for (int i = 0; i < connectedEdges.size(); ++i) {
  //for (int j = 0; j < connectedEdges[i].size(); ++j) {
  //  VisionUtils::drawPoint(connectedEdges[i][j]->pI, bgrMat[currentImage], Scalar(0,255,0));
  //}
  //imshow("bgr", bgrMat[currentImage]);
  //waitKey(0);
  //}
  if (connectedEdges.empty()) return false;
  return true;
}

void
LinesExtraction::findConnectedEdges(
  vector<vector<ScannedEdgePtr> >& connectedEdges,
  vector<ScannedEdgePtr>& scannedEdges, const unsigned& xTol,
  const unsigned& yTol, const bool& verticalScan)
{
  typedef vector<ScannedEdgePtr>::iterator sEIter;
  sEIter iter = scannedEdges.begin();
  while (iter != scannedEdges.end()) {
    if (*iter) ++iter;
    else iter = scannedEdges.erase(iter);
  }

  sort(scannedEdges.begin(), scannedEdges.end(), [](
    const ScannedEdgePtr& se1,
    const ScannedEdgePtr& se2)
  { return se1->pI.x < se2->pI.x;});

  ScannedEdgePtr pred;
  for (int i = 0; i < scannedEdges.size(); ++i) {
    ScannedEdgePtr se = scannedEdges[i];
    se->closestDist = 1000;
    se->angleW = 1000;
    se->bestNeighbor.reset();
    se->bestTo.reset();
    if (pred) se->pred = pred;
    pred = se;
  }
  int minDist = 12;
  int maxDist = sqrt(yTol * yTol + xTol * xTol);
  for (int i = 0; i < scannedEdges.size(); ++i) {
    ScannedEdgePtr se = scannedEdges[i];
    ScannedEdgePtr neighbor = se->pred;
    while (neighbor) {
      int diffY = abs(neighbor->pI.y - se->pI.y);
      if (diffY < yTol) {
        int diffX = abs(neighbor->pI.x - se->pI.x);
        if (verticalScan) {
          if (diffX < minDist) {
            neighbor = neighbor->pred;
            continue;
          }
        } else {
          if (diffY < minDist) {
            neighbor = neighbor->pred;
            continue;
          }
        }

        int dist = sqrt(diffX * diffX + diffY * diffY);
        if (dist < maxDist) {
          if (dist < se->closestDist) {
            se->closestDist = dist;
            se->bestNeighbor = neighbor;
            se->bestNeighbor->bestTo = se;
          } else {
            if (se->bestNeighbor) {
              //cout << "se->closestDist: " << se->closestDist << endl;
              //cout << "se->bestNeighbor: " << se->bestNeighbor << endl;
              //cout << "se->angleW: " << se->angleW * 180 /3.14 << endl;
              break;
            }
          }
        } else {
          if (se->bestNeighbor) {
            //cout << "se->closestDist: " << se->closestDist << endl;
            //cout << "se->bestNeighbor: " << se->bestNeighbor << endl;
            //cout << "se->angleW: " << se->angleW * 180 /3.14 << endl;
            break;
          }
        }
      }
      neighbor = neighbor->pred;
    }
    if (se->bestNeighbor) {
      //cout << "i: " << i << endl;
      se->angleW = atan2(
        se->bestNeighbor->pW.y - se->pW.y,
        se->bestNeighbor->pW.x - se->pW.x);
      //VisionUtils::drawPoint(se->pI, bgrMat[currentImage], Scalar(0,0,0));
      //VisionUtils::drawPoint(se->bestNeighbor->pI, bgrMat[currentImage], Scalar(0,255,0));
      //cout << "se->angleW: " << se->angleW * 180 /3.14 << endl;
      //imshow("bgr", bgrMat[currentImage]);
      //waitKey(0);
    }
  }
  for (int i = 0; i < scannedEdges.size(); ++i) {
    if (scannedEdges[i]->angleW == 1000) {
      if (scannedEdges[i]->bestTo) scannedEdges[i]->angleW =
        scannedEdges[i]->bestTo->angleW;
      else scannedEdges[i].reset();
    }
    // VisionUtils::drawPoint(scannedEdges[i]->pI, bgrMat[currentImage], Scalar(0,0,0));
    //cout << "angle:" << scannedEdges[i]->angleW * 180 / M_PI << endl;
    // imshow("bgr", bgrMat[currentImage]);
    // waitKey(0);
  }
  reverse(scannedEdges.begin(), scannedEdges.end());
  //vector<ScannedEdgePtr> chainParents;
  int minChainLength = 2;
  for (int i = 0; i < scannedEdges.size(); ++i) {
    if (!scannedEdges[i]) continue;
    vector<ScannedEdgePtr> groupedEdges;
    ScannedEdgePtr se = scannedEdges[i];
    if (se->searched) continue;
    groupedEdges.push_back(se);
    //chainParents.push_back(se);
    ScannedEdgePtr neighbor = se->bestNeighbor;
    //VisionUtils::drawPoint(se->pI, bgrMat[currentImage], Scalar(0,0,0));
    while (neighbor) {
      if (fabsf(neighbor->angleW - se->angleW) > 30 * M_PI / 180) {
        //cout << "break" << endl;
        //imshow("bgr", bgrMat[currentImage]);
        //waitKey(0);
        break;
      }
      if (neighbor->searched) {
        //cout << "break" << endl;
        //imshow("bgr", bgrMat[currentImage]);
        //waitKey(0);
        break;
      }
      //VisionUtils::drawPoint(neighbor->pI, bgrMat[currentImage], Scalar(0,255,0));
      //cout << "se->angleW: " << se->angleW * 180 / M_PI << endl;
      // cout << "neighbor->angleW: " << neighbor->angleW * 180 / M_PI << endl;
      // cout << "diff:" << fabsf(neighbor->angleW - se->angleW) * 180 / M_PI  << endl;
      neighbor->searched = true;
      groupedEdges.push_back(neighbor);
      neighbor = neighbor->bestNeighbor;
    }
    se->searched = true;
    if (groupedEdges.size() > minChainLength) {
      connectedEdges.push_back(groupedEdges);
    }
  }
}

void
LinesExtraction::findLinesFromEdges(
  vector<vector<ScannedEdgePtr> >& connectedEdges,
  vector<FittedLinePtr>& worldLines)
{
  mt19937 rng;
  uniform_real_distribution<> dist
    { 0, 1 };
  const int maxIters = 5;
  for (size_t i = 0; i < connectedEdges.size(); ++i) {
    float bestNx = 0;
    float bestNy = 0;
    float bestD = 0;
    int maxSum = 0;
    int pointsCnt = 0;
    for (int n = 0; n < maxIters; ++n) {
      auto p1 =
        connectedEdges[i][(int) (dist(rng) * connectedEdges[i].size())]->pW;
      auto p2 =
        connectedEdges[i][(int) (dist(rng) * connectedEdges[i].size())]->pW;
      if (p1.x != p2.x || p1.y != p2.y) {
        if (p1.x > p2.x) {
          Point2d tmp = p1;
          p1 = p2;
          p2 = tmp;
        }
        auto diff = p1 - p2;
        float len = norm(diff);
        if (len > 0.20) { // Length greater than 0.20 meters
          float nx = -diff.y / len;
          float ny = diff.x / len;
          float perpDist = nx * p1.x + ny * p1.y;
          int sum = 0;
          int cnt = 0;
          for (int j = 0; j < connectedEdges[i].size(); ++j) {
            auto p = connectedEdges[i][j]->pW;
            float dist = fabsf(nx * p.x + ny * p.y - perpDist);
            if (dist < 0.1f) {
              cnt++;
              sum += 1;
            } else if (dist > 0.1f && dist < 0.25f) {
              sum -= 1;
            }
          }
          if (sum > maxSum) {
            maxSum = sum;
            pointsCnt = cnt;
            bestNx = nx;
            bestNy = ny;
            bestD = perpDist;
          }
          if (pointsCnt >= 0.8 * connectedEdges[i].size()) {
            break;
          }
        }
      }
    }
    if (pointsCnt < 0.5 * connectedEdges[i].size()) {
      auto fl = boost::make_shared<FittedLine>();
      fl->circleLine = true;
      vector < Point2f > points;
      for (int j = 0; j < connectedEdges[i].size(); ++j) {
        points.push_back(connectedEdges[i][j]->pW);
      }
      fl->points = boost::make_shared < vector<Point2f> > (points);
    } else {
      if (pointsCnt >= 3) {
        vector < Point2f > collinearPoints;
        for (int j = 0; j < connectedEdges[i].size(); ++j) {
          auto p = connectedEdges[i][j]->pW;
          float dist = fabsf(bestNx * p.x + bestNy * p.y - bestD);
          if (dist <= 0.1f) collinearPoints.push_back(p);
        }
        sort(
          collinearPoints.begin(),
          collinearPoints.end(),
          [](const Point2f& p1, const Point2f& p2) {
            return (p1.x < p2.x) ||
            ((p1.x == p2.x) && (p1.y < p2.y));
          });
        Point2f minP = collinearPoints[0];
        Point2f maxP = collinearPoints.back();
        auto fl = boost::make_shared < FittedLine > (minP, maxP);
        fl->diff = maxP - minP;
        fl->d = norm(fl->diff);
        fl->perp = Point2f(bestNx, bestNy);
        fl->perpDist = bestD;
        fl->unit = Point2f(fl->diff.x / fl->d, fl->diff.y / fl->d);
        fl->points = boost::make_shared < vector<Point2f> > (collinearPoints);
        worldLines.push_back(fl);
        /*Point2f p11 = Point(fl->p1.x * 100 + 500, 350 - fl->p1.y * 100);
         Point2f p12 = Point(fl->p2.x * 100 + 500, 350 - fl->p2.y * 100);
         line(olImage, p11, p12, Scalar(0,0,255), 2);
         cout << "fl->d: " << fl->d << endl;
         imshow("olImage", olImage);
         waitKey(0);*/
      }
    }
  }
}

void
LinesExtraction::filterLines(vector<FittedLinePtr>& worldLines,
  vector<Point2f>& circlePoints)
{
  auto borderLines = fieldExt->getBorderLinesWorld();
  FittedLinePtr borderLine;
  if (borderLines.size() > 1) {
    borderLine =
      borderLines[0]->d > borderLines[1]->d ? borderLines[0] : borderLines[1];
  } else {
    borderLine = borderLines[0];
  }
  //Point2f p11 = Point(borderLine->p1.x * 100 + 500, 350 - borderLine->p1.y * 100);
  //Point2f p12 = Point(borderLine->p2.x * 100 + 500, 350 - borderLine->p2.y * 100);
  //line(olImage, p11, p12, Scalar(0,255,255), 2);
  //imshow("olImage", olImage);
  //waitKey(0);
  auto angleL = atan2(borderLine->unit.y, borderLine->unit.x);
  //cout << "angleL1: " << angleL * 180 / 3.14<< endl;
  angleL = angleL > M_PI_2 ? M_PI - angleL : angleL;
  angleL = angleL < 0 ? M_PI + angleL : angleL;
  //cout << "angleL2: " << angleL * 180 / 3.14<< endl;
  for (size_t i = 0; i < worldLines.size(); ++i) {
    if (!worldLines[i]) continue;
    auto wl = worldLines[i];
    //Point2f p11 = Point(wl->p1.x * 100 + 500, 350 - wl->p1.y * 100);
    //Point2f p12 = Point(wl->p2.x * 100 + 500, 350 - wl->p2.y * 100);
    //line(olImage, p11, p12, Scalar(255,255,255), 3);
    float dist =
      borderLine->perp.x * wl->p1.x + borderLine->perp.y * wl->p1.y - borderLine->perpDist;
    if (fabsf(dist) < 0.35) { // Perpendicular distance from border line
      worldLines[i].reset();
      continue;
    }
    Point2f inter = findIntersection(wl, borderLine);
    if (inter.x != -100) {
      float diff1 = norm(inter - wl->p1);
      float diff2 = norm(inter - wl->p2);
      float ratio = diff1 / wl->d + diff2 / wl->d;
      if (diff1 < 0.35 || diff2 < 0.35 || (ratio > 0.9 && ratio < 1.1)) {
        worldLines[i].reset();
        continue;
      }
    }
    auto angleW = atan2(wl->unit.y, wl->unit.x);
    angleW = angleW > M_PI_2 ? M_PI - angleW : angleW;
    angleW = angleW < 0 ? M_PI + angleW : angleW;
    //cout << "angleW: " << angleW * 180 / 3.14<< endl;
    auto angleD = abs(angleW - angleL);
    //cout << "angleDiff: " << angleD  * 180 / 3.14<< endl;
    // 15-75 degrees the difference between border line angle and the estimated world line angle
    // Might or might not have to increase the tolerance in real world 
    worldLines[i]->angle = angleW;
    if (angleD >= 0.261666667 && angleD <= 1.308333333) {
      if (fabsf(dist) > 2.5f) {
        wl->circleLine = true;
        continue;
      }
    }
    //line(olImage, p11, p12, Scalar(255,0,0), 1);
  }

  //imshow("LINES1", olImage); 
  //waitKey(0);
  for (size_t i = 0; i < worldLines.size(); ++i) {
    //olImage= Scalar(0);
    if (!worldLines[i]) continue;
    if (worldLines[i]->circleLine) continue;
    auto wl1 = worldLines[i];
    vector < Point2f > collinearPoints;
    vector < Point2f > points2;
    //Point2f p11 = Point(wl1->p1.x * 100 + 500, 350 - wl1->p1.y * 100);
    //Point2f p12 = Point(wl1->p2.x * 100 + 500, 350 - wl1->p2.y * 100);
    //line(olImage, p11, p12, Scalar(255,0,0), 2);
    collinearPoints.push_back(wl1->p1);
    collinearPoints.push_back(wl1->p2);
    for (size_t j = i; j < worldLines.size(); ++j) {
      if (!worldLines[j]) continue;
      if (worldLines[j]->circleLine) continue;
      if (i != j) {
        auto wl2 = worldLines[j];
        //Point2f p21 = Point(wl2->p1.x * 100 + 500, 350 - wl2->p1.y * 100);
        //Point2f p22 = Point(wl2->p2.x * 100 + 500, 350 - wl2->p2.y * 100);
        //line(olImage, p21, p22, Scalar(0,255,0), 2);
        //cout << "angle1: " << wl1->angle * 180/ 3.14 << endl;
        //cout << "angle2: " << wl2->angle * 180/ 3.14 << endl;

        if (abs(wl2->angle - wl1->angle) < 0.261666667) {
          float dist =
            wl1->perp.x * wl2->p1.x + wl1->perp.y * wl2->p1.y - wl1->perpDist;
          if (fabsf(dist) < 0.1) {
            collinearPoints.push_back(wl2->p1);
            collinearPoints.push_back(wl2->p2);
            points2 = *(wl2->points);
            worldLines[j].reset();
          }
        }
      }
    }
    if (!collinearPoints.empty()) {
      Point2f p1, p2;
      float maxDist = 0;
      for (int j = 0; j < collinearPoints.size(); ++j) {
        for (int k = j; k < collinearPoints.size(); ++k) {
          float dist = norm(collinearPoints[j] - collinearPoints[k]);
          if (dist > maxDist) {
            maxDist = dist;
            p1 = collinearPoints[j];
            p2 = collinearPoints[k];
          }
        }
      }
      auto wl = boost::make_shared < FittedLine > (p1, p2);
      wl->diff = p2 - p1;
      wl->d = norm(wl->diff);
      wl->unit = Point2f(wl->diff.x / wl->d, wl->diff.y / wl->d);
      wl->perp = Point2f(-wl->diff.y / wl->d, wl->diff.x / wl->d);
      wl->perpDist = wl->perp.x * wl->p1.x + wl->perp.y * wl->p1.y;
      vector < Point2f > points1 = *(worldLines[i]->points);
      points1.insert(points1.begin(), points2.begin(), points2.end());
      wl->points = boost::make_shared < vector<Point2f> > (points1);
      worldLines[i] = wl;
      //cout << "result" << endl;
      Point2f p11 = Point(p1.x * 100 + 500, 350 - p1.y * 100);
      Point2f p12 = Point(p2.x * 100 + 500, 350 - p2.y * 100);
      //VisionUtils::drawPoint(p11, olImage, Scalar(255,0,0));
      //VisionUtils::drawPoint(p12, olImage, Scalar(255,0,0));
      //line(olImage, p11, p12, Scalar(0,255,0), 1);
    }
  }
  //imshow("olImage", olImage); 
  //waitKey(0);

  for (size_t i = 0; i < worldLines.size(); ++i) {
    if (!worldLines[i]) continue;
    //cout << "i; " << i << endl;
    auto wl = worldLines[i];
    float dist =
      borderLine->perp.x * wl->p1.x + borderLine->perp.y * wl->p1.y - borderLine->perpDist;
    // cout << "wl->d " << wl->d << endl;
    //cout << "fabsf(dist)" <<fabsf(dist) << endl;
    // cout << "circleLine1: " << wl->circleLine << endl;
    if (wl->d < 0.65f && fabsf(dist) > 2.5f) { // Perpendicular distance from border line
      wl->circleLine = true;
    }
    if (wl->circleLine) {
      auto points = *(wl->points);
      circlePoints.insert(circlePoints.begin(), points.begin(), points.end());
    }
    //} else {
    // cout << "circleLine2: " << wl->circleLine << endl;
    //   Point2f p11 = Point(wl->p1.x * 100 + 500, 350 - wl->p1.y * 100);
    //   Point2f p12 = Point(wl->p2.x * 100 + 500, 350 - wl->p2.y * 100);
    //line(olImage, p11, p12, Scalar(0,0,255), 2);
    // }
    //imshow("olImage", olImage);
    // waitKey(0);
  }

  FlIter iter = worldLines.begin();
  while (iter != worldLines.end()) {
    if (*iter) ++iter;
    else iter = worldLines.erase(iter);
  }
  //cout << "Filtered worldLines.size() : " << worldLines.size() << endl;
}

void
LinesExtraction::findFeatures(vector<FittedLinePtr>& worldLines)
{
  for (size_t i = 0; i < worldLines.size(); ++i) {
    if (!worldLines[i]) continue;
    if (worldLines[i]->circleLine) continue;
    //olImage = Scalar(0);
    auto wl1 = worldLines[i];
    //Point2f p11 = Point(wl1->p1.x * 100 + 500, 350 - wl1->p1.y * 100);
    //Point2f p12 = Point(wl1->p2.x * 100 + 500, 350 - wl1->p2.y * 100);
    //line(olImage, p11, p12, Scalar(0,0,255), 2);
    //VisionUtils::drawPoint(p11, olImage, Scalar(0,0,255));
    //VisionUtils::drawPoint(p12, olImage, Scalar(0,0,255));
    for (size_t j = i; j < worldLines.size(); ++j) {
      if (!worldLines[j]) continue;
      if (worldLines[j]->circleLine) continue;
      if (j != i) {
        auto wl2 = worldLines[j];
        Point2f inter = findIntersection(wl1, wl2);
        Point2f imageP;
        cameraTransform->worldToImage(
          currentImage,
          Point3f(inter.x, inter.y, 0.f),
          imageP);
        if (imageP.x < 100 || imageP.x > getImageWidth() - 100 || imageP.y < 100 || imageP.y > getImageHeight() - 100) {
          continue;
        }
        if (inter.x == -100) continue;
        //Point2f p21 = Point(wl2->p1.x * 100 + 500, 350 - wl2->p1.y * 100);
        //Point2f p22 = Point(wl2->p2.x * 100 + 500, 350 - wl2->p2.y * 100);
        //line(olImage, p21, p22, Scalar(0,255,0), 2);
        VisionUtils::drawPoint(
          Point(inter.x * 100 + 500, 350 - 100 * inter.y),
          olImage,
          Scalar(0, 0, 255));
        //cout << "got inter" << endl;
        bool firstPass = false;
        bool secondPass = false;
        bool center1 = false;
        bool center2 = false;
        Point2f diff1 = inter - wl1->p1;
        float d1 = norm(diff1);
        //cout << "d1: " << d1 << endl;
        //cout << "d1 - wl1->d: " << d1 - wl1->d<< endl;
        Point2f unit1 = Point2f(diff1.x / d1, diff1.y / d1);
        if (unit1.x / (float) wl1->unit.x < 0) {
          if (d1 < 0.2) {
            firstPass = true;
          }
        } else {
          if (d1 - wl1->d < 0.3) { // meaning within 30 centimeters ahead of p2
            firstPass = true;
            float r = d1 / wl1->d;
            if (r > 0.05 && r < 0.95) {
              center1 = true;
            }
          }
        }

        //cout << "center1: " << center1 << endl;
        if (firstPass) {
          //cout << "In first pass" << endl;
          Point2f diff2 = inter - wl2->p1;
          float d2 = norm(diff2);
          //cout << "d2: " << d2 << endl;
          //cout << "wl2->d: " << wl2->d << endl;
          //cout << "d2 - wl2->d: " << d2 - wl2->d<< endl;
          Point2f unit2 = Point2f(diff2.x / d2, diff2.y / d2);
          if (unit2.x / (float) wl2->unit.x < 0) {
            if (d2 < 0.2) {
              //cout << " 1"  << endl;
              secondPass = true;
            }
          } else {
            if (d2 - wl2->d < 0.3) { // meaning within 30 centimeters ahead of p2
              //cout << "2"  << endl;
              float r = d2 / wl2->d;
              if (r > 0.05 && r < 0.95) {
                center2 = true;
              }
              secondPass = true;
            }
          }
          //cout << "center2: " << center2 << endl;
          // if (d1 / d2 > 3.f || d1 > d2 < 1/3.f)
          //  continue;

          if (secondPass) {
            //cout << "In second pass" << endl;
            // Finding angle
            float angle = acos(wl1->unit.dot(wl2->unit));
            //cout << "angle1:" << angle * 180/ 3.14 << endl;
            angle = angle > M_PI_2 ? M_PI - angle : angle;
            //cout << "angle:" << angle  * 180/ 3.14 << endl;
            //float diff45 = abs(angle - M_PI_2 / 2);
            //cout << "diff45:" << diff45 * 180/ 3.14<< endl;
            //Point2f p11 = Point(wl1->p1.x * 100 + 500, 350 - wl1->p1.y * 100);
            //Point2f p12 = Point(wl1->p2.x * 100 + 500, 350 - wl1->p2.y * 100);
            //Point2f p21 = Point(wl2->p1.x * 100 + 500, 350 - wl2->p1.y * 100);
            //Point2f p22 = Point(wl2->p2.x * 100 + 500, 350 - wl2->p2.y * 100);
            //line(olImage, p11, p12, Scalar(255,255,255), 2);
            //line(olImage, p21, p22, Scalar(255,255,255), 2);
            //cout << "center1: " << center1 << endl;
            //cout << "center2: " << center2 << endl;
            if (angle > 1.308333333) // 75 degrees
            {
              //cout << "center1: "<< center1 << endl;
              //cout << "center2: "<< center2 << endl;
              if (center1) {
                if (center2) {
                  //VisionUtils::drawPoint(Point(inter.x * 100 + 500, 350 - 100 * inter.y), olImage, Scalar(0,0,255));
                } else {
                  // Line 2 is perpendicular
                  // T joint
                  float norm1, norm2;
                  Point2f dist1, dist2;
                  dist1 = wl2->p1 - inter;
                  dist2 = wl2->p2 - inter;
                  norm1 = norm(dist1);
                  norm2 = norm(dist2);
                  Point2f unitT =
                    norm1 > norm2 ?
                      Point2f(dist1.x / norm1, dist1.y / norm1) :
                      Point2f(dist2.x / norm2, dist2.y / norm2);
                  computeLandmark(inter, unitT, FL_TYPE_T_CORNER);
                  //cout << "T joint " << endl;
                  VisionUtils::drawPoint(
                    Point(inter.x * 100 + 500, 350 - 100 * inter.y),
                    olImage,
                    Scalar(255, 0, 255));
                }
              } else if (center2) {
                // Line 1 is perpendicular
                // T joint
                float norm1, norm2;
                Point2f dist1, dist2;
                dist1 = wl1->p1 - inter;
                dist2 = wl1->p2 - inter;
                norm1 = norm(dist1);
                norm2 = norm(dist2);
                Point2f unitT =
                  norm1 > norm2 ?
                    Point2f(dist1.x / norm1, dist1.y / norm1) :
                    Point2f(dist2.x / norm2, dist2.y / norm2);
                computeLandmark(inter, unitT, FL_TYPE_T_CORNER);
                //cout << "T joint " << endl;
                VisionUtils::drawPoint(
                  Point(inter.x * 100 + 500, 350 - 100 * inter.y),
                  olImage,
                  Scalar(255, 0, 255));
              } else {
                // L joint
                float norm11, norm12;
                Point2f dist11, dist12;
                dist11 = wl1->p1 - inter;
                dist12 = wl1->p2 - inter;
                norm11 = norm(dist11);
                norm12 = norm(dist12);
                Point2f unitL1 =
                  norm11 > norm12 ?
                    Point2f(dist11.x / norm11, dist11.y / norm11) :
                    Point2f(dist12.x / norm12, dist12.y / norm12);
                float norm21, norm22;
                Point2f dist21, dist22;
                dist21 = wl2->p1 - inter;
                dist22 = wl2->p2 - inter;
                norm21 = norm(dist21);
                norm22 = norm(dist22);
                Point2f unitL2 =
                  norm21 > norm22 ?
                    Point2f(dist21.x / norm21, dist21.y / norm21) :
                    Point2f(dist22.x / norm22, dist22.y / norm22);
                Point2f unitL = unitL1 + unitL2;
                float normL = norm(unitL);
                unitL.x = unitL.x / normL;
                unitL.y = unitL.y / normL;
                computeLandmark(inter, unitL, FL_TYPE_L_CORNER);
                //cout << "L joint " << endl;
                VisionUtils::drawPoint(
                  Point(inter.x * 100 + 500, 350 - 100 * inter.y),
                  olImage,
                  Scalar(255, 255, 0));
              }
            }/* else if (angle > 0.087222222 * 2 && angle < 1.046666667) {
             /*if (!center1) {
             if (!center2) {
             if (wl1->d < 0.5) {
             //line(olImage, p11, p12, Scalar(0,255,255), 2);
             circlePoints.push_back(wl1->p1);
             circlePoints.push_back(wl1->p1);
             }
             if (wl2->d < 0.5) {
             //line(olImage, p21, p22, Scalar(0,255,255), 2);
             circlePoints.push_back(wl2->p1);
             circlePoints.push_back(wl2->p1);
             }
             //both 1 and 2
             } else {
             if (wl1->d < 0.5) {
             //line(olImage, p11, p12, Scalar(0,255,255), 2);
             circlePoints.push_back(wl1->p1);
             circlePoints.push_back(wl1->p1);
             }
             //add 1
             }
             } else if (!center2) {
             //add 2
             if (wl2->d < 0.5) {
             //line(olImage, p21, p22, Scalar(0,255,255), 2);
             circlePoints.push_back(wl2->p1);
             circlePoints.push_back(wl2->p1);
             }
             }
             circlePoints.push_back(inter);
             }*/
          }
        }
        //imshow("olImage", olImage);
        //waitKey(0);
      }
    }
  }
}

void
LinesExtraction::computeLandmark(const Point2f& inter,
  const Point2f& unitToBaseLine, const unsigned& type)
{
  float len = norm(inter);
  Point2f interUnit = Point2f(inter.x / len, inter.y / len);
  float cross = interUnit.x * unitToBaseLine.y - interUnit.y * unitToBaseLine.x;
  float dot = interUnit.dot(unitToBaseLine);
  float perpAngle = atan2(cross, dot) + atan2(inter.y, inter.x);
  float cosa = cos(perpAngle);
  float sina = sin(perpAngle);
  KnownLandmarkPtr l = boost::make_shared<KnownLandmark>();
  l->type = type;
  l->pos = inter;
  l->poseFromLandmark.x = -cosa * inter.x - sina * inter.y; // From -R^t * t inverse rotation
  l->poseFromLandmark.y = +sina * inter.x - cosa * inter.y;
  l->poseFromLandmark.theta = -perpAngle;
  knownLandmarks.push_back(l);
  //cout << "inter: " << inter << endl;               
  //cout << "robotX: "<< l.poseFromLandmark.x << endl;
  //cout << "robotY: "<< l.poseFromLandmark.y << endl;
  //cout << "robotOrientation: "<< l.poseFromLandmark.theta * 180 / 3.14 << endl;
  //VisionUtils::drawPoint(Point(inter.x * 100 + 500, 350 - 100 * inter.y), olImage, Scalar(0,255,0));
  //imshow("olImage", olImage);
  //waitKey(0);
}

void
LinesExtraction::computeCircleLandmark(const Circle& c,
  const vector<FittedLinePtr>& worldLines)
{
  for (size_t i = 0; i < worldLines.size(); ++i) {
    if (!worldLines[i]) continue;
    vector < Point2f > inters;
    auto wl = worldLines[i];
    if (wl->d > 0.6) {
      if (findCircleLineIntersection(c, wl, inters)) {
        if (!inters.empty()) {
          for (int i = 0; i < inters.size(); ++i) {
            Point2f diff;
            diff = inters[i] - c.center;
            float dist = norm(diff);
            Point2f unit = Point2f(diff.x / dist, diff.y / dist);
            computeLandmark(c.center, unit, FL_TYPE_CIRCLE);
          }
          break;
        }
        //VisionUtils::drawPoint(Point(inters[0].x * 100 + 500, 350 - 100 * inters[0].y), olImage, Scalar(0,255,0));
        //VisionUtils::drawPoint(Point(inters[1].x * 100 + 500, 350 - 100 * inters[1].y), olImage, Scalar(0,255,0));
        //imshow("olImage", olImage);
        //waitKey(0);
      }
    }
  }
}

bool
LinesExtraction::findCircleLineIntersection(const Circle& c,
  const FittedLinePtr& wl, vector<Point2f>& intersections)
{
  float t =
    wl->unit.x * (c.center.x - wl->p1.x) + wl->unit.y * (c.center.y - wl->p1.y);
  Point2f bisectorP;
  bisectorP.x = t * wl->unit.x + wl->p1.x;
  bisectorP.y = t * wl->unit.y + wl->p1.y;
  float perpDist = norm(bisectorP - c.center);
  if (perpDist < 0.2) {
    float dt = sqrt(c.radius * c.radius - perpDist * perpDist);
    Point2f inter1, inter2;
    inter1.x = bisectorP.x + dt * wl->unit.x;
    inter1.y = bisectorP.y + dt * wl->unit.y;

    inter2.x = bisectorP.x - dt * wl->unit.x;
    inter2.y = bisectorP.y - dt * wl->unit.y;
    intersections.push_back(inter1);
    intersections.push_back(inter2);
    return true;
  } else {
    return false;
  }
}

bool
LinesExtraction::findCircle(Circle& circleOutput, vector<Point2f>& circlePoints)
{
  //auto tStart = high_resolution_clock::now();
  if (circlePoints.empty()) return false;
  //vector<Point2f> circlePoints;
  //if (circlePoints.size() < 10) {
  //vector<Point2f> remains;
  //vector<Point> locations;
  //findNonZero(whiteEdges, locations);
  //Mat(locations).copyTo(remains);
  //vector<Point2f> worldRemains;
  //if (!remains.empty())
  //  cameraTransform->imageToWorld(currentImage, worldRemains, remains, 0.0);
  //Mat meanMat;
  //reduce(circlePoints, meanMat, 01, CV_REDUCE_AVG);
  //Point2f meanP = Point2f(meanMat.at<float>(0,0), meanMat.at<float>(0,1)); 
  //float maxRadius = 2.0;
  //for (int i = 0; i < circlePoints.size(); ++i) {
  //  if (norm(meanP - circlePoints[i]) < maxRadius)
  //  {
  //    circlePoints.push_back(circlePoints[i]);
  //  }
  //}
  //for (int i = 0; i < worldRemains.size(); ++i) {
  //  if (norm(meanP - worldRemains[i]) < maxRadius)
  //  {
  //    circlePoints.push_back(worldRemains[i]);
  //  }
  //}
  //} else {
  //circlePoints = circlePoints;
  //}
  //cout << "circlePoints: " << circlePoints.size() << endl;
  //! Draw remaining world points
  //int clusterCount = 2;
  /*if (worldRemains.size() > 10) {
   Mat centers, labels;
   Mat points = Mat(worldRemains).reshape(1);
   kmeans(
   points, clusterCount, labels,
   TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 10, 1.0),
   3, KMEANS_RANDOM_CENTERS, centers);
   vector<vector<Point2f> > clusters(clusterCount);
   for(int i = 0; i < worldRemains.size(); ++i) {
   int clusterIdx = labels.at<int>(i);
   Point2f p = worldRemains[i];
   clusters[clusterIdx].push_back(p);
   }
   if (clusters[0].size() > clusters[1].size()) {
   circlePoints = clusters[0];
   } else {
   circlePoints = clusters[1];
   }
   }*/
  mt19937 rng;
  uniform_real_distribution<> dist
    { 0, 1 };
  Mat meanMat;
  reduce(circlePoints, meanMat, 01, CV_REDUCE_AVG);
  Point2f meanP = Point2f(meanMat.at<float>(0, 0), meanMat.at<float>(0, 1));
  int maxCount = circlePoints.size() > 50 ? 50 : circlePoints.size();
  // VisionUtils::drawPoint(Point(meanP.x * 100 + 500, 350 - meanP.y * 100), olImage, Scalar(0,255,0));
  //for (size_t i = 0; i < circlePoints.size(); ++i) {
  //   Point2f wp = circlePoints[i];
  //  Point p = Point(wp.x * 100 + 500, 350 - wp.y * 100);
  //   VisionUtils::drawPoint(p, olImage, Scalar(255,255,255));
  // }
  int bestPoints = 0;
  Circle bestCircle;
  if (circlePoints.size() >= 10) {
    float radius = 0.75;
    for (int n = 0; n < 20; ++n) {
      int numPoints = 0;
      Point2f p1 = circlePoints[(int) (dist(rng) * circlePoints.size())];
      Point2f p2 = circlePoints[(int) (dist(rng) * circlePoints.size())];
      auto c = Circle(p1, p2, radius, meanP);
      auto center = c.center;
      for (int i = 0; i < maxCount; ++i) {
        float dist = norm(center - circlePoints[i]) - radius;
        if (fabsf(dist) < 0.2) {
          ++numPoints;
        } else {
          --numPoints;
        }
      }
      if (numPoints > bestPoints) {
        bestPoints = numPoints;
        bestCircle = c;
      }
      circle(
        olImage,
        Point(c.center.x * 100 + 500, 350 - c.center.y * 100),
        c.radius * 100,
        Scalar(0, 255, 255),
        1);
      if (bestPoints >= maxCount * 0.5) break;
    }
  }
  //auto tEnd = high_resolution_clock::now();
  //duration<double> timeSpan = tEnd - tStart; 
  //PRINT("LinesExtraction.CircleTime3.Update.Time: " << timeSpan.count() << " seconds.")
  if (bestPoints >= maxCount * 0.5) {
    circleOutput = bestCircle;
    return true;
  } else {
    return false;
  }
}

void
LinesExtraction::addLineLandmarks(vector<FittedLinePtr>& worldLines)
{
  //cout << "Number of landmarks observerd1: " << OVAR(ObsLandmarks, VisionModule::unknownLandmarksObs).data.size() << endl;
  int count = 0;
  for (size_t i = 0; i < worldLines.size(); ++i) {
    if (!worldLines[i]) continue;
    auto wl = worldLines[i];
    int numPoints = ceil(wl->d / 0.2);
    for (int i = 0; i < numPoints; ++i) {
      UnknownLandmarkPtr l = boost::make_shared<UnknownLandmark>();
      l->type = FL_TYPE_LINES;
      float r = i / (float) numPoints;
      l->pos.x = wl->p1.x + wl->unit.x * wl->d * r;
      l->pos.y = wl->p1.y + wl->unit.y * wl->d * r;
      //VisionUtils::drawPoint(Point(l->pos.x * 100 + 500, 350 - 100 * l->pos.y), olImage);
      unknownLandmarks.push_back(l);
      count++;
    }
    if (count >= 20) break;
  }
  //cout << "Number of landmarks observerd2: " << OVAR(ObsLandmarks, VisionModule::unknownLandmarksObs).data.size() << endl;
}

void
LinesExtraction::processImage()
{
  if (!fieldExt->isFound()) return;
  olImage = Scalar(0);
#ifdef DEBUG_BUILD
  auto tStart = high_resolution_clock::now();
#endif
  vector < vector<ScannedEdgePtr> > connectedEdges;
  if (!scanForEdges(connectedEdges)) return;

#ifdef DEBUG_BUILD      
  auto tEnd = high_resolution_clock::now();
  duration<double> timeSpan = tEnd - tStart;
  //PRINT("LinesExtraction.ScanForEdges.Time: " << timeSpan.count() << " seconds.")
  tStart = high_resolution_clock::now();
#endif

  vector<FittedLinePtr> worldLines;
  findLinesFromEdges(connectedEdges, worldLines);

#ifdef DEBUG_BUILD
  tEnd = high_resolution_clock::now();
  timeSpan = tEnd - tStart;
  //PRINT("LinesExtraction.findLinesFromEdges.Time: " << timeSpan.count() << " seconds.")
  tStart = high_resolution_clock::now();
#endif

  vector < Point2f > circlePoints;
  filterLines(worldLines, circlePoints);

#ifdef DEBUG_BUILD
  tEnd = high_resolution_clock::now();
  timeSpan = tEnd - tStart;
  //PRINT("LinesExtraction.filterLines.Time: " << timeSpan.count() << " seconds.")
  //! Draw world lines
  /*for (size_t i = 0; i < worldLines.size(); ++i) {
   if (!worldLines[i])
   continue;
   auto wl1 = worldLines[i];
   Point2f p11 = Point(wl1->p1.x * 100 + 500, 350 - wl1->p1.y * 100);
   Point2f p12 = Point(wl1->p2.x * 100 + 500, 350 - wl1->p2.y * 100);
   line(olImage, p11, p12, Scalar(0,0,255), 2);
   }*/
  tStart = high_resolution_clock::now();
#endif

  findFeatures(worldLines);

#ifdef DEBUG_BUILD
  tEnd = high_resolution_clock::now();
  timeSpan = tEnd - tStart;
  //PRINT("LinesExtraction.findFeatures.Time: " << timeSpan.count() << " seconds.")
  tStart = high_resolution_clock::now();
#endif

  Circle circleOutput;
  if (findCircle(circleOutput, circlePoints)) {
    computeCircleLandmark(circleOutput, worldLines);
#ifdef DEBUG_BUILD
    tEnd = high_resolution_clock::now();
    timeSpan = tEnd - tStart;
    //PRINT("LinesExtraction.findCircle.Update.Time: " << timeSpan.count() << " seconds.")
    circle(olImage, Point(circleOutput.center.x * 100 + 500, 350 - circleOutput.center.y * 100), circleOutput.radius * 100, Scalar(0,0,255), 1);
#endif
  }
  //cout << "circle" << endl;
#ifdef DEBUG_BUILD
  tStart = high_resolution_clock::now();
#endif
  addLineLandmarks(worldLines);
  //cout << "addLineLandamrks" << endl;
#ifdef DEBUG_BUILD
  tEnd = high_resolution_clock::now();
  timeSpan = tEnd - tStart;
#endif  
  //PRINT("LinesExtraction.addLineLandmarks.Time: " << timeSpan.count() << " seconds.")
  //imshow("ol", olImage);
  //waitKey(0);
}
