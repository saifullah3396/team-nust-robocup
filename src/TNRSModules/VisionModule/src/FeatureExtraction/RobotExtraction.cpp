/**
 * @file FeatureExtraction/RobotExtraction.h
 *
 * This file declares the class for robots extraction from 
 * the image. 
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author AbdulRehman
 * @date 22 Aug 2017  
 */

#include "VisionModule/include/FeatureExtraction/RobotExtraction.h"

template<typename T>
  double
  colorDistance(const T& color, const T& groupColor)
  {
    double h = (color[0] - groupColor[0]) / 180.0;
    double s = (color[1] - groupColor[1]) / 255.0;
    double v = (color[2] - groupColor[2]) / 255.0;
    return sqrt(h * h + s * s + v * v);
  }

void
RobotExtraction::linkRegions(vector<ScannedRegionPtr>& resultRegions,
  vector<ScannedRegionPtr>& regions, const unsigned& xTol, const unsigned& yTol)
{
  typedef vector<ScannedRegionPtr>::iterator sEIter;
  sEIter iter = regions.begin();
  while (iter != regions.end()) {
    if ((*iter)->rect.width > 5 && (*iter)->rect.height > 5) ++iter;
    else iter = regions.erase(iter);
  }

  sort(regions.begin(), regions.end(), [](
    const ScannedRegionPtr& sr1,
    const ScannedRegionPtr& sr2)
  { return sr1->center.x < sr2->center.x;});

  ScannedRegionPtr rPred;
  for (int i = 0; i < regions.size(); ++i) {
    regions[i]->closestDist = 1000;
    regions[i]->bestNeighbor.reset();
    if (rPred) {
      regions[i]->pred = rPred;
    }
    rPred = regions[i];
  }

  int maxDist = sqrt(yTol * yTol + xTol * xTol);
  for (int i = 0; i < regions.size(); ++i) {
    ScannedRegionPtr sr = regions[i];
    ScannedRegionPtr neighbor = sr->pred;
    while (neighbor != 0) {
      int diffY = abs(neighbor->center.y - sr->center.y);
      diffY -= sr->rect.height / 2 + neighbor->rect.height / 2;
      if (diffY > 0) {
        if (diffY < xTol) {
          int diffX = abs(neighbor->center.x - sr->center.x);
          diffX -= sr->rect.width / 2 + neighbor->rect.width / 2;
          if (diffX > 0) {
            int dist = sqrt(diffX * diffX + diffY * diffY);
            if (dist < maxDist) {
              if (dist < sr->closestDist) {
                sr->closestDist = dist;
                sr->bestNeighbor = neighbor;
              }
            }
          } else {
            if (diffY < sr->closestDist) {
              sr->closestDist = diffY;
              sr->bestNeighbor = neighbor;
            }
          }
        }
      } else {
        int diffX = abs(neighbor->center.x - sr->center.x);
        diffX -= sr->rect.width / 2 + neighbor->rect.width / 2;
        if (diffX > 0) {
          if (diffX < xTol) {
            if (diffX < sr->closestDist) {
              sr->closestDist = diffX;
              sr->bestNeighbor = neighbor;
            }
          }
        } else {
          sr->closestDist = 0;
          sr->bestNeighbor = neighbor;
        }
      }
      //rectangle(in, sr->rect, Scalar(0,0,0), 1);
      neighbor = neighbor->pred;
    }
  }

  reverse(regions.begin(), regions.end());
  vector<ScannedRegionPtr> rChainParents;
  for (int i = 0; i < regions.size(); ++i) {
    ScannedRegionPtr sr = regions[i];
    if (sr->searched) continue;
    rChainParents.push_back(sr);
    ScannedRegionPtr neighbor = sr->bestNeighbor;
    while (neighbor != 0) {
      if (neighbor->searched) break;
      neighbor->searched = true;
      neighbor = neighbor->bestNeighbor;
    }
  }

  for (int i = 0; i < rChainParents.size(); ++i) {
    ScannedRegionPtr sr = rChainParents[i];
    ScannedRegionPtr neighbor = sr->bestNeighbor;
    while (neighbor != 0) {
      sr->rect = sr->rect | neighbor->rect;
      neighbor = neighbor->bestNeighbor;
    }
    resultRegions.push_back(sr);
  }
}

void
RobotExtraction::filterJerseys(const vector<Point>& border,
  vector<ScannedLinePtr>& jerseyLines)
{
  for (int i = 0; i < jerseyLines.size(); ++i) {
    auto jl = jerseyLines[i];
    if (jl->p1.y < border[jl->p1.x].y) {
      if (border[jl->p1.x].y - jl->p1.y > 200) {
        jerseyLines[i].reset();
      }
    }
  }
}

void
RobotExtraction::filterRegions(vector<ScannedRegionPtr>& regions,
  const unsigned& threshold)
{
  for (size_t i = 0; i < regions.size(); ++i) {
    if (!regions[i]) continue;
    for (size_t j = 0; j < regions.size(); ++j) {
      if (!regions[j]) continue;
      if (i != j) {
        auto r1 = regions[i]->rect;
        auto r2 = regions[j]->rect;
        Rect overlap = r1 & r2;
        if (overlap.area() > 0) {
          regions[i] = boost::make_shared < ScannedRegion > (r1 | r2);
          regions[j].reset();
          continue;
        }
        int x11 = r1.x;
        int x12 = r1.x + r1.width;
        int x21 = r2.x;
        int x22 = r2.x + r2.width;
        int d1 = abs(x11 - x21);
        int d2 = abs(x11 - x22);
        int d3 = abs(x12 - x21);
        int d4 = abs(x12 - x22);
        int distX = d1 < d2 ? d1 : d2;
        distX = distX < d3 ? distX : d3;
        distX = distX < d4 ? distX : d4;
        if (distX < threshold) {
          int y11 = r1.y;
          int y12 = r1.y + r1.height;
          int y21 = r2.y;
          int y22 = r2.y + r2.height;
          int d1 = abs(y11 - y21);
          int d2 = abs(y11 - y22);
          int d3 = abs(y12 - y21);
          int d4 = abs(y12 - y22);
          int distY = d1 < d2 ? d1 : d2;
          distY = distY < d3 ? distY : d3;
          distY = distY < d4 ? distY : d4;
          if (distY < threshold) {
            regions[i] = boost::make_shared < ScannedRegion > (r1 | r2);
            regions[j].reset();
          }
        }
      }
    }
  }
}

void
RobotExtraction::findRobotRegions()
{
  auto verJerseyLinesOurs = regionSeg->getVerJerseyLinesOurs();
  auto horJerseyLinesOurs = regionSeg->getHorJerseyLinesOurs();
  auto verJerseyLinesOpps = regionSeg->getVerJerseyLinesOpps();
  auto horJerseyLinesOpps = regionSeg->getHorJerseyLinesOpps();

  filterJerseys(border, verJerseyLinesOurs);
  filterJerseys(border, horJerseyLinesOurs);
  filterJerseys(border, verJerseyLinesOpps);
  filterJerseys(border, horJerseyLinesOpps);

  vector<ScannedRegionPtr> verJerseyRegionsOurs;
  vector<ScannedRegionPtr> horJerseyRegionsOurs;
  vector<ScannedRegionPtr> verJerseyRegionsOpps;
  vector<ScannedRegionPtr> horJerseyRegionsOpps;
  //cout << "Finding Jersey Regions" << endl;
  findRegions(verJerseyRegionsOurs, verJerseyLinesOurs, 36, 36, false);
  findRegions(horJerseyRegionsOurs, horJerseyLinesOurs, 36, 36, true);
  findRegions(verJerseyRegionsOpps, verJerseyLinesOpps, 36, 36, false);
  findRegions(horJerseyRegionsOpps, horJerseyLinesOpps, 36, 36, true);
  //cout << "Found Jersey Regions" << endl;
  //cout << "Linking" << endl;	
  vector<ScannedRegionPtr> verJOursFiltered;
  vector<ScannedRegionPtr> horJOursFiltered;
  vector<ScannedRegionPtr> verJOppsFiltered;
  vector<ScannedRegionPtr> horJOppsFiltered;
  linkRegions(verJOursFiltered, verJerseyRegionsOurs, 36, 36);
  linkRegions(horJOursFiltered, horJerseyRegionsOurs, 36, 36);
  linkRegions(verJOppsFiltered, verJerseyRegionsOpps, 36, 36);
  linkRegions(horJOppsFiltered, horJerseyRegionsOpps, 36, 36);
  //cout << "Linking" << endl;	
  horJOursFiltered.insert(
    horJOursFiltered.end(),
    verJOursFiltered.begin(),
    verJOursFiltered.end());

  horJOppsFiltered.insert(
    horJOppsFiltered.end(),
    verJOppsFiltered.begin(),
    verJOppsFiltered.end());
  //cout << "Linking" << endl;	
  filterRegions(horJOursFiltered, 36);
  filterRegions(horJOppsFiltered, 36);

  for (int i = 0; i < horJOursFiltered.size(); ++i) {
    if (horJOursFiltered[i]) {
      auto jr = horJOursFiltered[i];
      int w = jr->rect.width;
      int h = jr->rect.height;
      if (w > 5 * h || h > 5 * w) continue;
      robotRegions.push_back(boost::make_shared < RobotRegion > (jr, true));
    }
  }
  for (int i = 0; i < horJOppsFiltered.size(); ++i) {
    if (horJOppsFiltered[i]) {
      auto jr = horJOppsFiltered[i];
      int w = jr->rect.width;
      int h = jr->rect.height;
      if (w > 5 * h || h > 5 * w) continue;
      robotRegions.push_back(boost::make_shared < RobotRegion > (jr, false));
    }
  }

  sort(robotRegions.begin(), robotRegions.end(), [](
    const RobotRegionPtr& rr1,
    const RobotRegionPtr& rr2)
  { return rr1->sr->center.x < rr2->sr->center.x;});
}

void
RobotExtraction::classifyRobots()
{
  for (int i = 0; i < outputRegions.size(); ++i) {
    if (outputRegions[i]) outputRegions[i]->refresh = false;
  }

  vector<RobotRegionPtr> newRobotRegions;
  for (int i = 0; i < robotRegions.size(); ++i) {
    auto rr = robotRegions[i];
    ScannedRegionPtr robotFound;
    drawRegion(bgrMat[currentImage], rr->sr, Scalar(255, 255, 255));
    //cout << "Area: " << rr->sr->rect.area() << endl;
    if (rr->sr->rect.area() > 10000) {
      robotFound = rr->sr;
      rr->posFromJersey = true;
    } else if (rr->sr->rect.area() > 1000) {
      // Large area
      if (rr->sr->rect.height >= rr->sr->rect.width) {
        int halfHeight = rr->sr->rect.height / 2;
        int minY = rr->sr->rect.y - halfHeight;
        int maxY = rr->sr->leftBase.y + halfHeight;
        int minX = rr->sr->leftBase.x - rr->sr->rect.width;
        int maxX = rr->sr->rightBase.x + rr->sr->rect.width;
        vector<ScannedLinePtr> bodyLines;
        for (int j = 0; j < verRobotLines.size(); ++j) {
          if (!verRobotLines[j]) continue;
          auto vl = verRobotLines[j];
          if (vl->p1.x > minX && vl->p1.x < maxX) {
            if (vl->p1.y > minY && vl->p1.y < maxY) bodyLines.push_back(vl);
          }
        }
        vector < Point > verPts;
        if (bodyLines.empty()) continue;
        for (int j = 0; j < bodyLines.size(); ++j) {
          verPts.push_back(bodyLines[j]->p1);
          verPts.push_back(bodyLines[j]->p2);
        }
        Rect bodyRect = boundingRect(verPts) | rr->sr->rect;
        float hRatio = bodyRect.height / ((float) rr->sr->rect.height);
        if (hRatio > 1.5) {
          robotFound = boost::make_shared < ScannedRegion > (bodyRect);
          //drawRegion(bgrMat[currentImage], robotFound, Scalar(255,0,0));
          //region found
        }
      } else {
        int minY = rr->sr->rect.y - rr->sr->rect.height;
        int maxY = rr->sr->leftBase.y + rr->sr->rect.height;
        int minX = rr->sr->leftBase.x - rr->sr->rect.width;
        int maxX = rr->sr->rightBase.x + rr->sr->rect.width;
        vector<ScannedLinePtr> bodyLines;
        for (int j = 0; j < horRobotLines.size(); ++j) {
          if (!horRobotLines[j]) continue;
          auto hl = horRobotLines[j];
          if (hl->p1.x > minX && hl->p1.x < maxX) {
            if (hl->p1.y > minY && hl->p1.y < maxY) bodyLines.push_back(hl);
          } else if (hl->p2.x > minX && hl->p2.x < maxX) {
            if (hl->p2.y > minY && hl->p2.y < maxY) bodyLines.push_back(hl);
          }
        }
        vector < Point > horPts;
        if (bodyLines.empty()) continue;
        for (int j = 0; j < bodyLines.size(); ++j) {
          horPts.push_back(bodyLines[j]->p1);
          horPts.push_back(bodyLines[j]->p2);
        }
        Rect bodyRect = boundingRect(horPts) | rr->sr->rect;
        float wRatio = bodyRect.width / ((float) rr->sr->rect.width);
        if (wRatio > 1.5) {
          rr->fallen = true;
          robotFound = boost::make_shared < ScannedRegion > (bodyRect);
          //drawRegion(bgrMat[currentImage], robotFound, Scalar(0,255,0));
          //region found
        }
      }
    } else {
      // Small area
      int doubleWidth = rr->sr->rect.width * 2;
      int minY = rr->sr->rect.y - rr->sr->rect.height;
      int maxY = rr->sr->leftBase.y + rr->sr->rect.height;
      int minX = rr->sr->leftBase.x - doubleWidth;
      int maxX = rr->sr->rightBase.x + doubleWidth;
      vector<ScannedLinePtr> bodyLines;
      for (int j = 0; j < verRobotLines.size(); ++j) {
        if (!verRobotLines[j]) continue;
        auto vl = verRobotLines[j];
        if (vl->p1.x > minX && vl->p1.x < maxX) {
          if (vl->p1.y > minY && vl->p1.y < maxY) bodyLines.push_back(vl);
        }
      }
      vector < Point > verPts;
      if (bodyLines.empty()) continue;
      for (int j = 0; j < bodyLines.size(); ++j) {
        verPts.push_back(bodyLines[j]->p1);
        verPts.push_back(bodyLines[j]->p2);
      }
      Rect bodyRect = boundingRect(verPts) | rr->sr->rect;
      float hRatio = bodyRect.height / ((float) rr->sr->rect.height);
      if (hRatio > 1.5) {
        robotFound = boost::make_shared < ScannedRegion > (bodyRect);
        //drawRegion(bgrMat[currentImage], robotFound, Scalar(0,0,255));
        // region found
      } else {
        minX = rr->sr->leftBase.x - doubleWidth;
        maxX = rr->sr->rightBase.x + doubleWidth;
        vector < ScannedLinePtr > bodyLines;
        for (int j = 0; j < horRobotLines.size(); ++j) {
          if (!horRobotLines[j]) continue;
          auto hl = horRobotLines[j];
          if (hl->p1.x > minX && hl->p1.x < maxX) {
            if (hl->p1.y > minY && hl->p1.y < maxY) bodyLines.push_back(hl);
          } else if (hl->p2.x > minX && hl->p2.x < maxX) {
            if (hl->p2.y > minY && hl->p2.y < maxY) bodyLines.push_back(hl);
          }
        }
        vector < Point > horPts;
        if (bodyLines.empty()) continue;
        for (int j = 0; j < bodyLines.size(); ++j) {
          horPts.push_back(bodyLines[j]->p1);
          horPts.push_back(bodyLines[j]->p2);
        }
        Rect bodyRect = boundingRect(horPts) | rr->sr->rect;
        float wRatio = bodyRect.width / ((float) rr->sr->rect.width);
        if (wRatio > 1.5f) {
          rr->fallen = true;
          robotFound = boost::make_shared < ScannedRegion > (bodyRect);
          //drawRegion(bgrMat[currentImage], robotFound, Scalar(255,255,0));
          // region found
        }
      }
    }
    if (robotFound) {
      // If robot is too close and its legs are not showing
      int robotMaxY = robotFound->rect.y + robotFound->rect.height;
      if (getImageHeight() - robotMaxY < 10 && !rr->fallen) {
        Rect updatedJersey;
        updatedJersey.x = robotFound->leftBase.x;
        updatedJersey.width = robotFound->rightBase.x - updatedJersey.x;
        updatedJersey.y = rr->sr->rect.y;
        updatedJersey.height = rr->sr->rect.height;
        rr->sr = boost::make_shared < ScannedRegion > (updatedJersey);
        rr->posFromJersey = true;
      } else {
        rr->sr = robotFound;
      }

      if (rr->posFromJersey) {
        // Jersey center with approximate height of 40 cms
        cameraTransform->imageToWorld(
          currentImage,
          rr->world,
          rr->sr->center,
          0.35);
      } else {
        cameraTransform->imageToWorld(
          currentImage,
          rr->world,
          robotFound->centerBase,
          0.0);
      }
      // Update new regions and check if a robot already exists in the region
      float closest = 1000;
      int minIndex = -1;
      for (int i = 0; i < outputRegions.size(); ++i) {
        if (!outputRegions[i]) continue;
        float dist = norm(rr->world - outputRegions[i]->world);
        if (dist < closest) {
          closest = dist;
          minIndex = i;
        }
      }
      if (minIndex != -1 && closest < outputRegions[minIndex]->minDistValidation) {
        // Update previous detected region
        outputRegions[minIndex] = rr;
        outputRegions[minIndex]->refresh = true;
      } else {
        // Add newly detected region
        newRobotRegions.push_back(rr);
      }
    }
  }
  outputRegions.insert(
    outputRegions.end(),
    newRobotRegions.begin(),
    newRobotRegions.end());
}

void
RobotExtraction::filterLinesBelowField()
{
  border = fieldExt->getBorder();
  verRobotLines = regionSeg->getVerRobotLines();
  horRobotLines = regionSeg->getHorRobotLines();

  for (int i = 0; i < verRobotLines.size(); ++i) {
    auto rl = verRobotLines[i];
    if (rl->p1.y < border[rl->p1.x].y) {
      if (rl->p2.y - border[rl->p2.x].y > 5) {
        rl->p1.y = border[rl->p2.x].y - 50;
      } else {
        verRobotLines[i].reset();
      }
    }
  }

  /*for (int i = 0; i < horRobotLines.size(); ++i) {
   auto rl = horRobotLines[i];
   if(rl->p1.y < border[rl->p1.x].y) {
   if (rl->p2.y - border[rl->p2.x].y < 10) {
   horRobotLines[i].reset();
   } else {
   for (int x = 0; x < border.size(); ++x) {
   if (border[x].y == rl->p1.y)
   rl->p1 = border[x];
   }
   }
   } else if (rl->p2.y < border[rl->p2.x].y) {
   if (rl->p1.y - border[rl->p1.x].y < 10) {
   horRobotLines[i].reset();
   } else {
   for (int x = 0; x < border.size(); ++x) {
   if (border[x].y == rl->p2.y)
   rl->p2 = border[x];
   }
   }
   }
   }*/

  /*for (int i = 0; i < horRobotLines.size(); ++i) {
   if (horRobotLines[i])
   line(bgrMat[currentImage], horRobotLines[i]->p1, horRobotLines[i]->p2, Scalar(255,0,0), 2);
   }
   
   for (int i = 0; i < verRobotLines.size(); ++i) {
   if (verRobotLines[i])
   line(bgrMat[currentImage], verRobotLines[i]->p1, verRobotLines[i]->p2, Scalar(255,0,0), 2);
   }*/
}

void
RobotExtraction::findStrayRegions()
{
  for (int i = 0; i < verRobotLines.size(); ++i) {
    if (!verRobotLines[i]) continue;
    auto rl = verRobotLines[i];
    if (rl->p1.y < border[rl->p1.x].y) {
      if (rl->p2.y - border[rl->p2.x].y > 5) {
        rl->p1.y = border[rl->p2.x].y;
      } else {
        verRobotLines[i].reset();
      }
    }
  }
  vector<ScannedRegionPtr> verStrayRegions;
  vector<ScannedRegionPtr> verStrayRegionsF;
  //for (int i = 0; i < horRobotLines.size(); ++i) {
  //  if (horRobotLines[i])
  //    line(bgrMat[currentImage], horRobotLines[i]->p1, horRobotLines[i]->p2, Scalar(255,0,0), 2);
  //}
  findRegions(verStrayRegions, verRobotLines, 18, 18, false);
  //drawRegions(bgrMat[currentImage], verStrayRegions, Scalar(0,0,0));
  linkRegions(verStrayRegionsF, verStrayRegions, 36, 36);
  for (int i = 0; i < verStrayRegionsF.size(); ++i) {
    Rect r = verStrayRegionsF[i]->rect;
    r = r - Point(r.width / 2, 0);
    r += Size(r.width, 0);
    r = r & Rect(0, 0, getImageWidth(), getImageHeight());
    verStrayRegionsF[i]->rect = r;
    strayRegions.push_back(verStrayRegionsF[i]->rect);
  }
  drawRegions(bgrMat[currentImage], verStrayRegionsF, Scalar(255, 0, 0));
}

void
RobotExtraction::refreshRobotRegions()
{
  float currentTime = visionModule->getModuleTime();
  RRIter iter = outputRegions.begin();
  while (iter != outputRegions.end()) {
    if (*iter) {
      if (currentTime - (*iter)->timeDetected < refreshTime) {
        ++iter;
      } else {
        iter = outputRegions.erase(iter);
      }
    } else {
      iter = outputRegions.erase(iter);
    }
  }
  robotRegions.clear();
}

void
RobotExtraction::updateRobotsInfo()
{
  for (int i = 0; i < outputRegions.size(); ++i) {
    if (!outputRegions[i]) continue;
    for (int j = 0; j < outputRegions.size(); ++j) {
      if (!outputRegions[j]) continue;
      if (i != j) {
        float dist = abs(
          outputRegions[i]->sr->center.x - outputRegions[j]->sr->center.x);
        if (dist < 15 && outputRegions[i]->ourTeam == outputRegions[j]->ourTeam) { // 15 pixels difference of centers
          outputRegions[j].reset();
        }
      }
    }
  }
  float currentTime = visionModule->getModuleTime();
  for (int i = 0; i < outputRegions.size(); ++i) {
    if (!outputRegions[i]) continue;
    if (outputRegions[i]->refresh) outputRegions[i]->timeDetected = currentTime;
    ObstacleType type = ObstacleType::UNKNOWN;
    if (outputRegions[i]->ourTeam) {
      if (!outputRegions[i]->fallen) type = ObstacleType::TEAMMATE;
      else type = ObstacleType::TEAMMATE_FALLEN;
    } else {
      if (!outputRegions[i]->fallen) type = ObstacleType::OPPONENT;
      else type = ObstacleType::OPPONENT_FALLEN;
    }
    Obstacle obstacle(type);
    obstacle.center = outputRegions[i]->world;
    if (outputRegions[i]->posFromJersey) {
      auto left = Point(
        outputRegions[i]->sr->leftBase.x,
        outputRegions[i]->sr->center.y);
      auto right = Point(
        outputRegions[i]->sr->rightBase.x,
        outputRegions[i]->sr->center.y);
      cameraTransform->imageToWorld(
        currentImage,
        obstacle.leftBound,
        left,
        0.35);
      cameraTransform->imageToWorld(
        currentImage,
        obstacle.rightBound,
        right,
        0.35);
    } else {
      cameraTransform->imageToWorld(
        currentImage,
        obstacle.leftBound,
        outputRegions[i]->sr->leftBase,
        0.0);
      cameraTransform->imageToWorld(
        currentImage,
        obstacle.rightBound,
        outputRegions[i]->sr->rightBase,
        0.0);
    }
    OVAR(ObsObstacles, VisionModule::obstaclesObs).data.push_back(obstacle);
  }
}

void
RobotExtraction::processImage()
{
  if (!fieldExt->isFound()) return;
#ifdef DEBUG_BUILD
  auto tStart = high_resolution_clock::now();
#endif
  strayRegions.clear();
  //LOG_INFO("Refreshing robot regions...")
  refreshRobotRegions();
  //LOG_INFO("Filtering robot region lines...")
  filterLinesBelowField();
  //LOG_INFO("Finding robot regions...")
  findRobotRegions();
  //LOG_INFO("Classifying robots...")
  classifyRobots();
  //LOG_INFO("Robot regions size: " << outputRegions.size())
  //if (!outputRegions.empty()) {
  //  for (int i = 0; i < outputRegions.size(); ++i) {
  //    cout << "Region[" << i << "]: " << outputRegions[i]->world << endl;
  //    cout << "outputRegions[i]->posFromJersey: "<< outputRegions[i]->posFromJersey << endl;
  //  }
  //}
  findStrayRegions();
  updateRobotsInfo();
  for (int i = 0; i < outputRegions.size(); ++i) {
    if (!outputRegions[i]) continue;
    auto rr = outputRegions[i];
    if (rr->ourTeam) rectangle(
      bgrMat[currentImage],
      rr->sr->rect,
      Scalar(255, 255, 255),
      2);
    else rectangle(bgrMat[currentImage], rr->sr->rect, Scalar(0, 0, 0), 2);
  }
#ifdef DEBUG_BUILD
  auto tEnd = high_resolution_clock::now();
  duration<double> timeSpan = tEnd - tStart;
  //LOG_INFO("RobotExtraction.Update.Time: " << timeSpan.count() << " seconds.")
#endif
  //VisionUtils::displayImage(bgrMat[currentImage], "robots");
  /*#ifdef DEBUG_BUILD
   if (GET_DVAR(int, sendTime)) {
   high_resolution_clock::time_point tEnd = 
   high_resolution_clock::now();
   duration<double> timeSpan = tEnd - tStart;
   CommModule::addToLogMsgQueue("RobotExtraction time: " +
   DataUtils::varToString(timeSpan.count()) + " seconds.");
   }
   #endif*/
}
