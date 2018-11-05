/**
 * @file LocalizationModule/src/ParticleFilter.cpp
 *
 * The class for implementing particle filter for robot localization.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author TeamNust 2015
 * @date 17 Sep 2017
 */

#include "LocalizationModule/include/ParticleFilter.h"
#include "Utils/include/TeamPositions.h"

#define ALPHA_SLOW  .01 //0.05
#define ALPHA_FAST .1

ParticleFilter::ParticleFilter(LocalizationModule* lModule) :
  MemoryBase(lModule), lModule(lModule),
    cycleTime(lModule->getPeriodMinMS() / (1000.f)), nParticles(20), genStd(3),
    lostCount(0),
    //recheckCount(0),
    //inRecheck(false),
    predictionStd(3), measurementStd(3), initiated(false), localized(false),
    lastKnownHalf(0), unitVecY(NUM_CAMS), prevGoalInfoId(0), 
    avgFilteredState(RobotPose2D<float>(0.f, 0.f, 0.f)),
    landmarkTypeCount(NUM_LANDMARK_TYPES),
    landmarkTypeStarts(NUM_LANDMARK_TYPES)
{
  GET_CONFIG(
    "ParticleFilter",
    (double, GenerationStd.x, genStd[0]), 
    (double, GenerationStd.y, genStd[1]), 
    (double, GenerationStd.theta, genStd[2]),
    (double, MotionStd.x, motionStd[0]), 
    (double, MotionStd.y, motionStd[1]), 
    (double, MotionStd.theta, motionStd[2]), 
    (double, PredictionStd.x, predictionStd[0]), 
    (double, PredictionStd.y, predictionStd[1]), 
    (double, PredictionStd.theta, predictionStd[2]), 
    (double, MeasurementStd.x, measurementStd[0]), 
    (double, MeasurementStd.y, measurementStd[1]), 
    (double, MeasurementStd.theta, measurementStd[2]), 
  );

  gaussianConst = 0.5 / (M_PI * measurementStd[0] * measurementStd[1]);
  expConstX = -0.5 / (measurementStd[0] * measurementStd[0]);
  expConstY = -0.5 / (measurementStd[1] * measurementStd[1]);
  estimatedStates.set_capacity(30);
  LOG_INFO("Setting up landmarks.")
  setupLandmarks();
  LOG_INFO("Setting up Voronoi map.")
  setupVoronoiMap();
  LOG_INFO("Setting up view vectors.")
  setupViewVectors();
  OVAR(int, LocalizationModule::sideConfidence) = 0;
  reset();
}

void
ParticleFilter::reset()
{
  particles.clear();
  estimatedStates.clear();
  initiated = false;
  localized = false;
  lastKnownHalf = 0;
  OVAR(int, LocalizationModule::positionConfidence) = 0;
}

void
ParticleFilter::setupLandmarks()
{
  landmarkTypeCount[FL_TYPE_GOAL_POST] = FL_GOAL_POSTS;
  landmarkTypeCount[FL_TYPE_T_CORNER] = FL_T_CORNERS;
  landmarkTypeCount[FL_TYPE_LINES] = 0;
  landmarkTypeCount[FL_TYPE_L_CORNER] = FL_L_CORNERS;
  landmarkTypeCount[FL_TYPE_CIRCLE] = FL_CIRCLES;
  landmarkTypeCount[FL_TYPE_PENALTY_MARK] = FL_PENALTY_MARKS;
  landmarkTypeStarts[FL_TYPE_GOAL_POST] = FL_LT_GOALPOST;
  landmarkTypeStarts[FL_TYPE_T_CORNER] = FL_GOAL_POSTS;
  landmarkTypeStarts[FL_TYPE_LINES] = 0;
  landmarkTypeStarts[FL_TYPE_L_CORNER] = FL_GOAL_POSTS + FL_T_CORNERS;
  landmarkTypeStarts[FL_TYPE_CIRCLE] =
    FL_GOAL_POSTS + FL_T_CORNERS + FL_L_CORNERS;
  landmarkTypeStarts[FL_TYPE_PENALTY_MARK] =
    FL_GOAL_POSTS + FL_T_CORNERS + FL_L_CORNERS + FL_CIRCLES;

  //for (int i = 0; i < 21; ++i) {
  // cout << "landmarksOnField[" << i << "]: " << landmarksOnField[i].x << endl;
  //  cout << "landmarksOnField[" << i << "]: " << landmarksOnField[i].y << endl;
  //  cout << "landmarksOnField[" << i << "]: " << landmarksOnField[i].theta * 180 / M_PI<< endl;
  //}

  for (int i = 0; i < FL_GOAL_POSTS; ++i) {
    fieldLandmarks.push_back(
      boost::make_shared < Landmark > (FL_TYPE_GOAL_POST, Point2f(
        landmarksOnField[i].x,
        landmarksOnField[i].y)));
    fieldLandmarks.back()->id = fieldLandmarks.size() - 1;
  }
  vector < Point2f > fieldCornerPs;
  fieldCornerPs.push_back(Point2f(4.5, 3.0));
  fieldCornerPs.push_back(Point2f(4.5, -3.0));
  fieldCornerPs.push_back(Point2f(-4.5, -3.0));
  fieldCornerPs.push_back(Point2f(-4.5, 3.0));
  vector < Point2f > midLinePs;
  midLinePs.push_back(Point2f(0.0, 3.0));
  midLinePs.push_back(Point2f(0.0, -3.0));
  vector < Point2f > goalBox1Ps;
  goalBox1Ps.push_back(Point2f(4.5, 1.1));
  goalBox1Ps.push_back(Point2f(3.9, 1.1));
  goalBox1Ps.push_back(Point2f(3.9, -1.1));
  goalBox1Ps.push_back(Point2f(4.5, -1.1));
  vector < Point2f > goalBox2Ps;
  goalBox2Ps.push_back(Point2f(-4.5, 1.1));
  goalBox2Ps.push_back(Point2f(-3.9, 1.1));
  goalBox2Ps.push_back(Point2f(-3.9, -1.1));
  goalBox2Ps.push_back(Point2f(-4.5, -1.1));

  float gridResolution = 0.20;
  for (int i = 0; i < fieldCornerPs.size(); ++i) {
    int k = i + 1;
    k = k == fieldCornerPs.size() ? 0 : k;
    float ratio = ceil(
      norm(fieldCornerPs[k] - fieldCornerPs[i]) / gridResolution);
    for (int j = 0; j < ratio; ++j) {
      fieldLandmarks.push_back(
        boost::make_shared < Landmark > (FL_TYPE_LINES, Point2f(
          fieldCornerPs[i].x + (fieldCornerPs[k].x - fieldCornerPs[i].x) * j / ratio,
          fieldCornerPs[i].y + (fieldCornerPs[k].y - fieldCornerPs[i].y) * j / ratio)));
      fieldLandmarks.back()->id = fieldLandmarks.size() - 1;
    }
  }
  for (int i = 0; i < goalBox1Ps.size(); ++i) {
    int k = i + 1;
    if (k == goalBox1Ps.size()) {
      break;
    }
    float ratio = ceil(norm(goalBox1Ps[k] - goalBox1Ps[i]) / gridResolution);
    for (int j = 0; j < ratio; ++j) {
      fieldLandmarks.push_back(
        boost::make_shared < Landmark > (FL_TYPE_LINES, Point2f(
          goalBox1Ps[i].x + (goalBox1Ps[k].x - goalBox1Ps[i].x) * j / ratio,
          goalBox1Ps[i].y + (goalBox1Ps[k].y - goalBox1Ps[i].y) * j / ratio)));
      fieldLandmarks.back()->id = fieldLandmarks.size() - 1;
    }
  }
  for (int i = 0; i < goalBox2Ps.size(); ++i) {
    int k = i + 1;
    if (k == goalBox2Ps.size()) {
      break;
    }
    float ratio = ceil(norm(goalBox2Ps[k] - goalBox2Ps[i]) / gridResolution);
    for (int j = 0; j < ratio; ++j) {
      fieldLandmarks.push_back(
        boost::make_shared < Landmark > (FL_TYPE_LINES, Point2f(
          goalBox2Ps[i].x + (goalBox2Ps[k].x - goalBox2Ps[i].x) * j / ratio,
          goalBox2Ps[i].y + (goalBox2Ps[k].y - goalBox2Ps[i].y) * j / ratio)));
      fieldLandmarks.back()->id = fieldLandmarks.size() - 1;
    }
  }
  float ratio = ceil(norm(midLinePs[1] - midLinePs[0]) / gridResolution);
  for (int j = 1; j < ratio; ++j) {
    fieldLandmarks.push_back(
      boost::make_shared < Landmark > (FL_TYPE_LINES, Point2f(
        midLinePs[0].x + (midLinePs[1].x - midLinePs[0].x) * j / ratio,
        midLinePs[0].y + (midLinePs[1].y - midLinePs[0].y) * j / ratio)));
    fieldLandmarks.back()->id = fieldLandmarks.size() - 1;
  }

  float theta = 0;
  float eRadius = 0.75;
  ratio = 12;
  for (int i = 0; i <= ratio; ++i) {
    float eX = eRadius * cos(M_PI * i / ratio);
    float eY = eRadius * sin(M_PI * i / ratio);
    fieldLandmarks.push_back(
      boost::make_shared < Landmark > (FL_TYPE_LINES, Point2f(eX, eY)));
    fieldLandmarks.back()->id = fieldLandmarks.size() - 1;
    if (i != 0 && i != ratio) {
      fieldLandmarks.push_back(
        boost::make_shared < Landmark > (FL_TYPE_LINES, Point2f(eX, -eY)));
      fieldLandmarks.back()->id = fieldLandmarks.size() - 1;
    }
  }
}

void
ParticleFilter::setupVoronoiMap()
{
  vector < Point2f > voronoiMapPs;
  voronoiMapPs.push_back(Point2f(4.5, 0.0));
  voronoiMapPs.push_back(Point2f(3.9, 0.0));
  voronoiMapPs.push_back(Point2f(4.5, 2.05));
  voronoiMapPs.push_back(Point2f(4.5, -2.05));
  voronoiMapPs.push_back(Point2f(4.5 / 2, 0.0));
  voronoiMapPs.push_back(Point2f(4.5 / 2, 3.0));
  voronoiMapPs.push_back(Point2f(4.5 / 2, -3.0));
  voronoiMapPs.push_back(Point2f(-4.5, 0.0));
  voronoiMapPs.push_back(Point2f(-3.9, 0.0));
  voronoiMapPs.push_back(Point2f(-4.5, 2.05));
  voronoiMapPs.push_back(Point2f(-4.5, -2.05));
  voronoiMapPs.push_back(Point2f(-4.5 / 2, 0.0));
  voronoiMapPs.push_back(Point2f(-4.5 / 2, 3.0));
  voronoiMapPs.push_back(Point2f(-4.5 / 2, -3.0));
  voronoiMapPs.push_back(Point2f(0.0, 0.0));
  voronoiMapPs.push_back(Point2f(0.0, 1.875));
  voronoiMapPs.push_back(Point2f(0.0, -1.875));
  //voronoiMapPs.push_back(Point2f(0.75, 0.0)); 
  //voronoiMapPs.push_back(Point2f(-0.75, 0.0)); 
  float fieldWidth, fieldHeight, mapResolution;
  GET_CONFIG(
    "PathPlanner",
    (float, Map.cellSize, mapResolution), (float, Map.fieldWidth, fieldWidth), (float, Map.fieldHeight, fieldHeight), )
  Size mapSize = Size(fieldWidth / mapResolution, fieldHeight / mapResolution);

  Mat distMap, voronoiColored;
  Mat voronoiPMap = Mat(mapSize, CV_8UC1, Scalar(255));
  Mat flImage = Mat(mapSize, CV_8UC3);
  for (int j = 0; j < voronoiMapPs.size(); ++j) {
    auto p = Point(
      voronoiMapPs[j].x / mapResolution + mapSize.width / 2,
      mapSize.height / 2 - voronoiMapPs[j].y / mapResolution);
    voronoiPMap.at < uchar > (p.y, p.x) = 0;
  }
  distanceTransform(voronoiPMap, distMap, voronoiLabels, CV_DIST_L1, 5);
  voronoiColored.create(voronoiLabels.size(), CV_8UC3);
  Scalar colors[] =
    { Scalar(0, 0, 0), Scalar(255, 0, 0), Scalar(255, 128, 0), Scalar(
      255,
      255,
      0), Scalar(0, 255, 0), Scalar(0, 128, 255), Scalar(0, 255, 255), Scalar(
      0,
      0,
      255), Scalar(255, 0, 255), Scalar(255, 255, 255), Scalar(255, 255, 128) };
  numLabels = 0;
  for (int i = 0; i < voronoiLabels.rows; i++) {
    const int* ll = (const int*) voronoiLabels.ptr(i);
    //const float* dd = (const float*)distMap.ptr(i);
    uchar* d = (uchar*) voronoiColored.ptr(i);
    for (int j = 0; j < voronoiLabels.cols; j++) {
      int idx = ll[j] == 0 ? 0 : (ll[j] - 1) % 10 + 1;
      //float scale = 1.f/(1 + dd[j]*dd[j]*0.0004f);
      int b = cvRound(colors[idx][0]);
      int g = cvRound(colors[idx][1]);
      int r = cvRound(colors[idx][2]);
      d[j * 3] = (uchar) b;
      d[j * 3 + 1] = (uchar) g;
      d[j * 3 + 2] = (uchar) r;
      numLabels = ll[j] > numLabels ? ll[j] : numLabels;
    }
  }
  labelledLandmarks.resize(numLabels);
  for (int i = 0; i < fieldLandmarks.size(); ++i) {
    auto fl = fieldLandmarks[i];
    Point pos = Point(fl->pos.x * 100 + 500, 350 - 100 * fl->pos.y);
    int landmarkLabel = voronoiLabels.at < uint > (pos.y, pos.x);
    labelledLandmarks[landmarkLabel - 1].push_back(fl);
    //VisionUtils::drawPoint(pos, flImage, Scalar(255,255,255));
    //VisionUtils::drawPoint(pos, voronoiColored, Scalar(255,255,255));
  }

  // For debugging
  /*for (int i = 0; i < labelledLandmarks.size(); ++i) {
   for (int j = 0; j < labelledLandmarks[i].size(); ++j) {
   Landmark ll = labelledLandmarks[i][j];
   }
   }*/

  // For debugging
  /*for (int j = 0; j < fieldLandmarks.size(); ++j) {
   //cout << "landmark[" << j << "]: " << fieldLandmarks[j].pos << endl;
   Point pos = Point(fieldLandmarks[j]->pos.x * 100 + 500, 350 - 100 * fieldLandmarks[j]->pos.y);
   Vec3b color = voronoiColored.at<Vec3b>(pos.y, pos.x);
   VisionUtils::drawPoint(pos, flImage, Scalar(color[0],color[1],color[2]));
   }*/

  // For debugging
  //VisionUtils::displayImage(voronoiColored, "voronoiMap");
  //VisionUtils::displayImage(flImage, "flImage");
  //waitKey(0);
}

void
ParticleFilter::setupViewVectors()
{
  vector<float> fovX(NUM_CAMS);
  vector<float> fovY(NUM_CAMS);
  GET_CONFIG(
    "UpperCamera",
    (float, Intrinsic.fovX, fovX[TOP_CAM]), (float, Intrinsic.fovY, fovY[TOP_CAM]), )
  GET_CONFIG(
    "LowerCamera",
    (float, Intrinsic.fovX, fovX[BOTTOM_CAM]), (float, Intrinsic.fovY, fovY[BOTTOM_CAM]), )
  fovX[0] *= M_PI / 180;
  fovY[0] *= M_PI / 180;
  fovX[1] *= M_PI / 180;
  fovY[1] *= M_PI / 180;
  for (unsigned i = 0; i < NUM_CAMS; ++i) {
    unitVecY[i][0] = 0;
    if (i == TOP_CAM) unitVecY[i][1] = -sin(fovY[i] / 2);
    else unitVecY[i][1] = sin(fovY[i] / 2);
    unitVecY[i][2] = cos(fovY[i] / 2);
  }
  maxDistGround = 10.0;
}

void
ParticleFilter::init(const RobotPose2D<float>& state)
{
  NormalDistribution<double> distX;
  NormalDistribution<double> distY;
  NormalDistribution<double> distTheta;
  wSlow = 0.0, wFast = 0.0;
  nParticles = 20;
  particles.clear();
  float w = 1.0 / nParticles;
  for (unsigned i = 0; i < nParticles; ++i) {
    Particle p;
    p.state.x = distX(random, state.x, genStd[0]);
    p.state.y = distY(random, state.y, genStd[1]);
    p.state.theta = distTheta(random, state.theta, genStd[2]);
    p.weight = w;
    particles.push_back(p);
  }
  avgFilteredState = state;
  initiated = true;
  OVAR(int, LocalizationModule::sideConfidence) = 100;
}

void
ParticleFilter::init(const vector<RobotPose2D<float> >& states)
{
  NormalDistribution<double> distX;
  NormalDistribution<double> distY;
  NormalDistribution<double> distTheta;
  wSlow = 0.0, wFast = 0.0;
  nParticles = 40;
  float w = 1.0 / nParticles;
  avgFilteredState = RobotPose2D<float>(0.f, 0.f, 0.f);
  particles.clear();
  int divider = nParticles / states.size();
  cout << "divider:" << divider << endl;
  cout << "states.size(): " << states.size() << endl;
  for (unsigned i = 0; i < nParticles; ++i) {
    cout << "i / divider: " << i / divider << endl;
    Particle p;
    p.state.x = distX(random, states[i / divider].x, genStd[0]);
    p.state.y = distY(random, states[i / divider].y, genStd[1]);
    p.state.theta = distTheta(random, states[i / divider].theta, genStd[2]);
    p.weight = w;
    avgFilteredState += p.state;
    particles.push_back(p);
  }
  avgFilteredState.x = avgFilteredState.x / nParticles;
  avgFilteredState.y = avgFilteredState.y / nParticles;
  avgFilteredState.theta = avgFilteredState.theta / nParticles;
  initiated = true;
}

void
ParticleFilter::init()
{
  /*Mat olImage = Mat(
   Size(
   1000,
   700
   ),
   CV_8UC3,
   Scalar(0,0,0)
   );*/
  NormalDistribution<double> distX;
  NormalDistribution<double> distY;
  NormalDistribution<double> distTheta;
  wSlow = 0.0, wFast = 0.0;
  nParticles = 20;
  float w = 1.0 / nParticles;
  avgFilteredState = RobotPose2D<float>(0.f, 0.f, 0.f);
  particles.clear();
  int divider = nParticles / estimatedStates.size();
  for (unsigned i = 0; i < nParticles; ++i) {
    Particle p;
    p.state.x = distX(random, estimatedStates[i / divider].x, genStd[0]);
    p.state.y = distY(random, estimatedStates[i / divider].y, genStd[1]);
    p.state.theta = distTheta(
      random,
      estimatedStates[i / divider].theta,
      genStd[2]);
    p.weight = w;
    //cout << "p.state.x: " << p.state.x << endl;
    //cout << "p.state.y: " << p.state.y << endl;
    //cout << "p.state.t: " << p.state.theta << endl;
    /*circle(
     olImage,
     Point(p.state.x * 100 + 500, 350 - p.state.y * 100),
     10,
     Scalar(255, 0, 0)
     );
     line(
     olImage,
     Point(p.state.x * 100 + 500, 350 - p.state.y * 100),
     Point(p.state.x * 100 + 500 + 15 * cos(p.state.theta), 350 - p.state.y * 100 - 15 * sin(p.state.theta)),
     Scalar(255, 0, 0)
     );*/
    avgFilteredState += p.state;
    particles.push_back(p);
  }
  //imshow("olImage", olImage);
  //waitKey(0);
  avgFilteredState.x = avgFilteredState.x / nParticles;
  avgFilteredState.y = avgFilteredState.y / nParticles;
  avgFilteredState.theta = avgFilteredState.theta / nParticles;
  initiated = true;
}

void
ParticleFilter::estimateForSideLines()
{
  // Determine the state of the robot and initiate the particle filter
  GoalInfo goalInfo = IVAR(GoalInfo, LocalizationModule::goalInfo);
  if (prevGoalInfoId != 0 && goalInfo.id != prevGoalInfoId) {
    // cout << "Finding a state estimate..." << endl;
    // cout << "Estimated state..." << endl;
    // cout << "goal found: " << goalInfo.found << endl;
    // cout << "goal ours: " << goalInfo.ours << endl;
    // cout << "goal left: " << goalInfo.leftPost << endl;
    // cout << "goal right: " << goalInfo.rightPost << endl;
    if (goalInfo.found) {
      if (goalInfo.ours == 1) {
        if (goalInfo.leftPost.y != -100) {
          if (goalInfo.leftPost.y < 0) {
            auto state = RobotPose2D<float>(
              -4.5 - goalInfo.leftPost.y,
              3.0,
              -M_PI_2);
            //cout << "State.x: " << state.x << endl;
            //cout << "State.y: " << state.y << endl;
            //cout << "State.theta: " << state.theta << endl;
            estimatedStates.push_back(state);
          } else {
            auto state = RobotPose2D<float>(
              -4.5 + goalInfo.leftPost.y,
              -3.0,
              M_PI_2);
            //cout << "State.x: " << state.x << endl;
            //cout << "State.y: " << state.y << endl;
            //cout << "State.theta: " << state.theta << endl;
            estimatedStates.push_back(state);
          }
        }

        if (goalInfo.rightPost.y != -100) {
          if (goalInfo.rightPost.y < 0) {
            auto state = RobotPose2D<float>(
              -4.5 - goalInfo.rightPost.y,
              3.0,
              -M_PI_2);
            //cout << "State.x: " << state.x << endl;
            //cout << "State.y: " << state.y << endl;
            //cout << "State.theta: " << state.theta << endl;
            estimatedStates.push_back(state);
          } else {
            auto state = RobotPose2D<float>(
              -4.5 + goalInfo.rightPost.y,
              -3.0,
              M_PI_2);
            //cout << "State.x: " << state.x << endl;
            //cout << "State.y: " << state.y << endl;
            //cout << "State.theta: " << state.theta << endl;
            estimatedStates.push_back(state);
          }
        }
      } else if (goalInfo.ours == 0) {
        if (goalInfo.leftPost.y != -100) {
          if (goalInfo.leftPost.y < 0) {
            auto state = RobotPose2D<float>(
              goalInfo.leftPost.y + 4.5,
              -3.0,
              M_PI_2);
            estimatedStates.push_back(state);
          } else {
            auto state = RobotPose2D<float>(
              goalInfo.leftPost.y - 4.5,
              3.0,
              -M_PI_2);
            estimatedStates.push_back(state);
          }
        }

        if (goalInfo.rightPost.y != -100) {
          if (goalInfo.rightPost.y < 0) {
            auto state = RobotPose2D<float>(
              goalInfo.leftPost.y + 4.5,
              -3.0,
              M_PI_2);
            estimatedStates.push_back(state);
          } else {
            auto state = RobotPose2D<float>(
              goalInfo.rightPost.y - 4.5,
              3.0,
              -M_PI_2);
            estimatedStates.push_back(state);
          }
        }
      }
    }
  }
  prevGoalInfoId = goalInfo.id;
  if (!knownLandmarks.empty()) {
    /*Mat olImage = Mat(
     Size(
     1000,
     700
     ),
     CV_8UC3,
     Scalar(0,0,0)
     );*/
    for (int i = 0; i < knownLandmarks.size(); ++i) {
      KnownLandmarkPtr l = knownLandmarks[i];
      auto rPose = l->poseFromLandmark;
      unsigned lCount = landmarkTypeCount[l->type];
      unsigned lStart = landmarkTypeStarts[l->type];
      //cout << "Type: " << l->type << endl;
      float minDist = 1000;
      RobotPose2D<float> estimate;
      //cout << "poseX: " << rPose.x << endl;
      //cout << "poseY: " << rPose.y << endl;
      //cout << "poseTheta: " << rPose.theta * 180 / M_PI << endl;
      for (int j = 0; j < lCount; ++j) {
        auto lPose = landmarksOnField[j + lStart];
        //cout << "lX: " << lPose.x << endl;
        //cout << "lY: " << lPose.y << endl;
        //cout << "lTheta: " << lPose.theta * 180 / M_PI << endl;
        RobotPose2D<float> wPose;
        wPose.x = lPose.x + rPose.x * cos(lPose.theta) - rPose.y * sin(
          lPose.theta);
        wPose.y = lPose.y + rPose.x * sin(lPose.theta) + rPose.y * cos(
          lPose.theta);
        wPose.theta = MathsUtils::rangeToPi(rPose.theta + lPose.theta);
        // Should be in our own half which is in -ve x and not too close to the goal line.
        // Should be either closer to upper line or lower line which is 3 
        // meters from center
        //float rangeT = MathsUtils::rangeToPi(wPose.theta);
        bool positive = false;
        if (wPose.x > -4.25 && wPose.x < 0.f) {
          if (wPose.y > 2.5f) {
            if (wPose.theta > -1.7444 && wPose.theta < -1.39556) {
              positive = true;
            }
          } else if (wPose.y < -2.5f) {
            if (wPose.theta > 1.39556 && wPose.theta < 1.7444) {
              positive = true;
            }
          }
        }
        //cout << "j; " << j << endl;
        //cout << "wX: " << wPose.x << endl;
        //cout << "wY: " << wPose.y << endl;
        //cout << "wTheta: " << wPose.theta * 180 / M_PI << endl;
        if (positive) {
          estimatedStates.push_back(wPose);
        } else {
          continue;
        }
        /*circle(
         olImage,
         Point(wPose.x * 100 + 500, 350 - wPose.y * 100),
         10,
         Scalar(255, 0, 0)
         );
         line(
         olImage,
         Point(wPose.x * 100 + 500, 350 - wPose.y * 100),
         Point(wPose.x * 100 + 500 + 15 * cos(wPose.theta), 350 - wPose.y * 100 - 15 * sin(wPose.theta)),
         Scalar(255, 0, 0)
         );*/
        //VisionUtils::displayImage(olImage,"olImage");
        //waitKey(0);
      }
    }
    knownLandmarks.clear();
  }
}

void
ParticleFilter::estimateFromLandmarks()
{
  Mat olImage = Mat(Size(1000, 700), CV_8UC3, Scalar(0, 0, 0));
  auto& lastPose = OVAR(RobotPose2D<float>, LocalizationModule::lastKnownPose2D);
  auto& useLastEstimate = IVAR(bool, LocalizationModule::localizeWithLastKnown);

  if (knownLandmarks.empty()) {
    //cout << "knownLandmarks: " << landmarksObs.data.size() << endl;
    for (int i = 0; i < knownLandmarks.size(); ++i) {
      KnownLandmarkPtr l = knownLandmarks[i];
      auto rPose = l->poseFromLandmark;
      unsigned lCount = landmarkTypeCount[l->type];
      unsigned lStart = landmarkTypeStarts[l->type];
      //cout << "Type: " << l->type << endl;
      float minDist = 1000;
      RobotPose2D<float> estimate;
      //cout << "poseX: " << rPose.x << endl;
      //cout << "poseY: " << rPose.y << endl;
      //cout << "poseTheta: " << rPose.theta * 180 / M_PI << endl;
      for (int j = 0; j < lCount; ++j) {
        auto lPose = landmarksOnField[j + lStart];
        //cout << "lX: " << lPose.x << endl;
        //cout << "lY: " << lPose.y << endl;
        //cout << "lTheta: " << lPose.theta * 180 / M_PI << endl;
        RobotPose2D<float> wPose;
        wPose.x = lPose.x + rPose.x * cos(lPose.theta) - rPose.y * sin(
          lPose.theta);
        wPose.y = lPose.y + rPose.x * sin(lPose.theta) + rPose.y * cos(
          lPose.theta);
        wPose.theta = MathsUtils::rangeToPi(rPose.theta + lPose.theta);
        if (useLastEstimate && lastPose.x != -1e3) {
          if (wPose.x > -4.75f && wPose.x < 4.75f && wPose.y > -3.25 && wPose.y < 3.25) {
            float d = MathsUtils::dist(
              wPose.x,
              wPose.y,
              lastPose.x,
              lastPose.y);
            if (d < 1.f) { // Within 25 cm radius
              estimatedStates.push_back(wPose);
              circle(
                olImage,
                Point(wPose.x * 100 + 500, 350 - wPose.y * 100),
                10,
                Scalar(255, 0, 0));
              line(
                olImage,
                Point(wPose.x * 100 + 500, 350 - wPose.y * 100),
                Point(
                  wPose.x * 100 + 500 + 15 * cos(wPose.theta),
                  350 - wPose.y * 100 - 15 * sin(wPose.theta)),
                Scalar(255, 0, 0));
            }
          }
        } else {
          if (lastKnownHalf == 0) {
            if (wPose.x > -4.75f && wPose.x < 0.f && wPose.y > -3.25 && wPose.y < 3.25) {
              estimatedStates.push_back(wPose);
              /*circle(
               olImage,
               Point(wPose.x * 100 + 500, 350 - wPose.y * 100),
               10,
               Scalar(255, 0, 0)
               );
               line(
               olImage,
               Point(wPose.x * 100 + 500, 350 - wPose.y * 100),
               Point(wPose.x * 100 + 500 + 15 * cos(wPose.theta), 350 - wPose.y * 100 - 15 * sin(wPose.theta)),
               Scalar(255, 0, 0)
               );*/
              //cout << "wX: " << wPose.x << endl;
              //cout << "wY: " << wPose.y << endl;
              //cout << "wTheta: " << wPose.theta * 180 / M_PI << endl;
            }
          } else {
            if (wPose.x > 0.f && wPose.x < 4.75f && wPose.y > -3.25f && wPose.y < 3.25f) {
              estimatedStates.push_back(wPose);
              //cout << "wX: " << wPose.x << endl;
              //cout << "wY: " << wPose.y << endl;
              //cout << "wTheta: " << wPose.theta * 180 / M_PI << endl;
            }
          }
        }
      }
    }
    knownLandmarks.clear();
  }
  //cout << "lastKnownHalf: " << lastKnownHalf << endl;
  GoalInfo goalInfo = IVAR(GoalInfo, LocalizationModule::goalInfo);
  if (prevGoalInfoId != 0 && goalInfo.id != prevGoalInfoId) {
    if (goalInfo.found) {
      /*Mat olImage = Mat(
       Size(
       1000,
       700
       ),
       CV_8UC3,
       Scalar(0,0,0)
       );*/
      auto rPose = goalInfo.poseFromGoal;
      RobotPose2D<float> wPose;
      if (goalInfo.ours == 1) {
        RobotPose2D<float> lPose = landmarksOnField[FL_LB_GOALPOST];
        lPose.y = 0.f;
        wPose.x = lPose.x + rPose.x * cos(lPose.theta) - rPose.y * sin(
          lPose.theta);
        wPose.y = lPose.y + rPose.x * sin(lPose.theta) + rPose.y * cos(
          lPose.theta);
        wPose.theta = MathsUtils::rangeToPi(rPose.theta + lPose.theta);
      } else if (goalInfo.ours == 0) {
        RobotPose2D<float> lPose = landmarksOnField[FL_LT_GOALPOST];
        lPose.y = 0.f;
        wPose.x = lPose.x + rPose.x * cos(lPose.theta) - rPose.y * sin(
          lPose.theta);
        wPose.y = lPose.y + rPose.x * sin(lPose.theta) + rPose.y * cos(
          lPose.theta);
        wPose.theta = MathsUtils::rangeToPi(rPose.theta + lPose.theta);
      } else {
        RobotPose2D<float> lPose1 = landmarksOnField[FL_LB_GOALPOST];
        lPose1.y = 0.f;
        RobotPose2D<float> lPose2 = landmarksOnField[FL_LT_GOALPOST];
        lPose2.y = 0.f;
        RobotPose2D<float> wPose1, wPose2;
        wPose1.x = lPose1.x + rPose.x * cos(lPose1.theta) - rPose.y * sin(
          lPose1.theta);
        wPose1.y = lPose1.y + rPose.x * sin(lPose1.theta) + rPose.y * cos(
          lPose1.theta);
        wPose1.theta = rPose.theta + lPose1.theta;
        wPose2.x = lPose2.x + rPose.x * cos(lPose2.theta) - rPose.y * sin(
          lPose2.theta);
        wPose2.y = lPose2.y + rPose.x * sin(lPose2.theta) + rPose.y * cos(
          lPose2.theta);
        wPose2.theta = MathsUtils::rangeToPi(rPose.theta + lPose2.theta);
        if (lastKnownHalf == 0) {
          if (wPose1.x < 0.f) wPose = wPose1;
          else if (wPose2.x < 0.f) wPose = wPose2;
        } else {
          if (wPose1.x > 0.f) wPose = wPose1;
          else if (wPose2.x > 0.f) wPose = wPose2;
        }
      }
      if (wPose.x > -4.75f && wPose.x < 4.75f && wPose.y > -3.25 && wPose.y < 3.25) {
        if (useLastEstimate && lastPose.x != -1e3) {
          float d = MathsUtils::dist(wPose.x, wPose.y, lastPose.x, lastPose.y);
          if (d < 1.f) { // Within 25 cm radius
            estimatedStates.push_back(wPose);
          }
        } else {
          estimatedStates.push_back(wPose);
        }
      }
      /*circle(
       olImage,
       Point(wPose.x * 100 + 500, 350 - wPose.y * 100),
       10,
       Scalar(255, 0, 0)
       );
       line(
       olImage,
       Point(wPose.x * 100 + 500, 350 - wPose.y * 100),
       Point(wPose.x * 100 + 500 + 15 * cos(wPose.theta), 350 - wPose.y * 100 - 15 * sin(wPose.theta)),
       Scalar(255, 0, 0)
       );*/
    }
  }
  prevGoalInfoId = goalInfo.id;
  //imshow("olImage", olImage);
  //waitKey(0);
}

void
ParticleFilter::getRobotStateEstimate()
{
  if (IVAR(bool, LocalizationModule::robotOnSideLine)) {
    if (estimatedStates.size() >= 30) {
      RobotPose2D<float> avgEstimatedState(0.f, 0.f, 0.f);
      for (int i = 0; i < estimatedStates.size(); ++i) {
        avgEstimatedState += estimatedStates[i];
      }
      avgEstimatedState.x = avgEstimatedState.x / 30;
      avgEstimatedState.y = avgEstimatedState.y / 30;
      avgEstimatedState.theta = avgEstimatedState.theta / 30;
      //cout << "avgEstimate.x" << avgEstimatedState.x << endl;
      //cout << "avgEstimate.y" << avgEstimatedState.y << endl;
      //cout << "avgEstimate.theta" << avgEstimatedState.theta * 180 / M_PI << endl;
      avgEstimatedState.y =
        abs(avgEstimatedState.y + 3.0) < abs(avgEstimatedState.y - 0.3) ?
          -3.0 : 3.0;
      avgEstimatedState.theta =
        abs(avgEstimatedState.theta + M_PI_2) < abs(
          avgEstimatedState.theta - M_PI_2) ? -M_PI_2 : M_PI_2;
      estimatedStates.clear();
      // 10 while in game - 20 while in starting pose determination
      estimatedStates.set_capacity(10);
      lastKnownHalf = 0;
      OVAR(int, LocalizationModule::sideConfidence) = 100;
      init(avgEstimatedState);
    }
    estimateForSideLines();
  } else {
    cout << "estimatedStates: " << estimatedStates.size() << endl;
    if (estimatedStates.size() >= 10) {
      //inRecheck = false;
      init();
    }
    estimateFromLandmarks();
  }
}

void
ParticleFilter::update()
{
  if (IVAR(bool, LocalizationModule::robotOnSideLine)) {
    if (estimatedStates.capacity() != 30) estimatedStates.set_capacity(30);
  } else {
    if (estimatedStates.capacity() != 10) estimatedStates.set_capacity(10);
  }
  if (!initiated) {
    getRobotStateEstimate();
  } else {
    //if (inRecheck) {
    //  getRobotStateEstimate();
    //}
    if (localized) {
      nParticles = nParticles > 20 ? 20 : nParticles;
    }
    /*if (prediction(positionInput)) {
     avgFilteredState = RobotPose2D<float>(0.f, 0.f, 0.f);
     float w = 1.0 / nParticles;
     for (unsigned i = 0; i < nParticles; ++i) {
     avgFilteredState = avgFilteredState + particles[i].state;
     particles[i].weight = w;
     }
     avgFilteredState.x = avgFilteredState.x / nParticles;
     avgFilteredState.y = avgFilteredState.y / nParticles;
     avgFilteredState.theta = avgFilteredState.theta / nParticles;
     } else {*/
    vector<LandmarkPtr> obsLandmarks;
    for (size_t i = 0; i < knownLandmarks.size(); ++i) {
      obsLandmarks.push_back(knownLandmarks[i]);
    }
    for (size_t i = 0; i < unknownLandmarks.size(); ++i) {
      obsLandmarks.push_back(unknownLandmarks[i]);
    }
    if (!obsLandmarks.empty()) {
      //minMaxGroundDist();
      prediction();
      //cout << "Updating weights" << endl;
      updateWeights(obsLandmarks);
      //cout << "Normalizing weights" << endl;
      if (normalizeWeights(false)) {
        //cout << "Resampling" << endl;
        resample();
      }
    } else {
      prediction();
      updateWeights (vector<LandmarkPtr>());
      if(normalizeWeights(true))
        resample();
    }
    checkIfLocalized();
    updateSideConfidence();
  }
  knownLandmarks.clear();
  unknownLandmarks.clear();
}

void 
ParticleFilter::prediction()
{
  if(!positionInputs.empty()) {
    auto& input = positionInputs.front();
    NormalDistribution<double> distX;
    NormalDistribution<double> distY;
    NormalDistribution<double> distTheta;
    for (unsigned i = 0; i < nParticles; ++i) {
      particles[i].state.x = 
        distX(random, particles[i].state.x + input.x, motionStd[0]);
      particles[i].state.y = 
        distY(random, particles[i].state.y + input.y, motionStd[1]);
      particles[i].state.theta = 
        distTheta(
          random, particles[i].state.theta + input.theta, motionStd[2]);
    }
    positionInputs.pop();
  } else {
    addPredictionNoise();
  }
}

void
ParticleFilter::addPredictionNoise()
{
  NormalDistribution<double> distX;
  NormalDistribution<double> distY;
  NormalDistribution<double> distTheta;
  for (unsigned i = 0; i < nParticles; ++i) {
    particles[i].state.x = distX(
      random,
      particles[i].state.x,
      predictionStd[0]);
    particles[i].state.y = distY(
      random,
      particles[i].state.y,
      predictionStd[1]);
    particles[i].state.theta = distTheta(
      random,
      particles[i].state.theta,
      predictionStd[2]);
  }
}

void
ParticleFilter::prediction(const VelocityInput<double>& vI)
{
  NormalDistribution<double> distX;
  NormalDistribution<double> distY;
  NormalDistribution<double> distTheta;
  for (unsigned i = 0; i < nParticles; ++i) {
    Particle p = particles[i];
    double thetaF = p.state.theta + cycleTime * vI.dTheta;
    double velMag = sqrt(vI.dX * vI.dX + vI.dY * vI.dY);
    double velAngle = atan2(vI.dY, vI.dX);
    double deltaX;
    double deltaY;
    if (vI.dTheta != 0.0) {
      double arcRadius = velMag / vI.dTheta;
      deltaX = arcRadius * (sin(thetaF + velAngle) - sin(
        p.state.theta + velAngle));
      deltaY = arcRadius * (-cos(thetaF + velAngle) + cos(
        p.state.theta + velAngle));
    } else {
      deltaX = cycleTime * vI.dX;
      deltaY = cycleTime * vI.dY;
    }
    double xF = p.state.x + deltaX;
    double yF = p.state.y + deltaY;
    particles[i].state.x = distX(random, xF, predictionStd[0]);
    particles[i].state.y = distY(random, yF, predictionStd[1]);
    particles[i].state.theta = distTheta(random, thetaF, predictionStd[2]);
  }
}

void
ParticleFilter::updateWeights(const vector<LandmarkPtr>& obsLandmarks)
{
  //cout << "Number of landmarks: " << obsLandmarks.size() << endl;
  avgFilteredState = RobotPose2D<float>(0.f, 0.f, 0.f);
  sumWeights = 0.0;
  maxWeight = 0.0;
  if (obsLandmarks.empty()) {
    for (unsigned i = 0; i < nParticles; ++i) {
      avgFilteredState = avgFilteredState + particles[i].state;
    }
    avgFilteredState.x = avgFilteredState.x / nParticles;
    avgFilteredState.y = avgFilteredState.y / nParticles;
    avgFilteredState.theta = avgFilteredState.theta / nParticles;
    return;
  }
  //int iterations = 0;
  for (unsigned i = 0; i < nParticles; ++i) {
    Particle p = particles[i];
    /* Mat olImage = Mat(
     Size(
     1000,
     700
     ),
     CV_8UC3,
     Scalar(0,0,0)
     );*/
    double w = particles[i].weight;
    vector<bool> lMarked(fieldLandmarks.size(), false);
    /*for (int j = 0; j < fieldLandmarks.size(); ++j) {
     auto ll = fieldLandmarks[j];
     VisionUtils::drawPoint(Point(ll->pos.x * 100 + 500, 350 - 100 * ll->pos.y), olImage, Scalar(0,0,255));
     }*/
    for (unsigned j = 0; j < obsLandmarks.size(); ++j) {
      bool badObs = false;
      auto obs = obsLandmarks[j];
      auto ct = cos(p.state.theta);
      auto st = sin(p.state.theta);
      Point2f tPos;
      tPos.x = p.state.x + obs->pos.x * ct - obs->pos.y * st;
      tPos.y = p.state.y + obs->pos.x * st + obs->pos.y * ct;
      //cout << "Obs Pos: " << obs->pos << endl;
      //cout << "T Pos: " << tPos << endl;
      if (abs(tPos.x) > 4.75 || abs(tPos.y) > 3.25) {
        badObs = true;
        //++badObs;
        //continue;
      }
      //VisionUtils::drawPoint(Point(tPos.x * 100 + 500, 350 - 100 * tPos.y), olImage, Scalar(255,0,0));
      if (!badObs) {
        //cout << "T Pos: " << Point(tPos.x * 100 + 500, 350 - 100 * tPos.y) << endl;
        //imshow("olImage", olImage);
        //waitKey(0);
        int index = -1;
        unsigned label =
          voronoiLabels.at < uint > (350 - tPos.y * 100, 100 * tPos.x + 500);
        double minDist = 1000;
        //if (!labelledLandmarks[label-1].empty()) {
        //  index = labelledLandmarks[label-1][0]->id;
        //  minDist = norm(labelledLandmarks[label-1][0]->pos - tPos);
        //}
        //cout << "label: " << label << endl;
        //cout << "labelledLandmarks[label-1].size(): " << labelledLandmarks[label-1].size() << endl;
        //cout << "fieldLandmarks.size(): " << fieldLandmarks.size() << endl;
        for (unsigned k = 0; k < labelledLandmarks[label - 1].size(); ++k) {
          //cout << "k: " << k << endl;
          auto ll = labelledLandmarks[label - 1][k];
          //cout << "1" << endl;
          //cout << "ll->id:"<< ll->id << endl;
          if (!lMarked[ll->id]) {
            //cout << "2" << endl;
            if (obs->type == ll->type) {
              //cout << "3" << endl;
              double d = norm(ll->pos - tPos);
              //cout << "4" << endl;
              if (d < minDist) {
                minDist = d;
                index = ll->id;
              }
              //cout << "5" << endl;
            }
          }
          //iterations++;
        }
        if (minDist < 0.25) {
          auto ll = fieldLandmarks[index];
          double dx = (tPos.x - ll->pos.x);
          dx *= dx;
          double dy = (tPos.y - ll->pos.y);
          dy *= dy;
          w *= gaussianConst * exp(dx * expConstX + dy * expConstY);
          //cout << "w: " << gaussianConst * exp(dx*expConstX + dy*expConstY) << endl;
          //VisionUtils::drawPoint(Point(ll->pos.x * 100 + 500, 350 - 100 * ll->pos.y), olImage, Scalar(0,255,0));
          lMarked[index] = true;
        } else {
          double dx = 0.25 / sqrt(2);
          dx *= dx;
          double dy = 0.25 / sqrt(2);
          dy *= dy;
          w *= gaussianConst * exp(dx * expConstX + dy * expConstY);
        }
      } else {
        double dx = 0.25 / sqrt(2);
        dx *= dx;
        double dy = 0.25 / sqrt(2);
        dy *= dy;
        w *= gaussianConst * exp(dx * expConstX + dy * expConstY);
      }
    }
    //imshow("olImage", olImage);
    //waitKey(0);
    particles[i].weight = w;
    sumWeights += w;
    maxWeight = w > maxWeight ? w : maxWeight;
    avgFilteredState = avgFilteredState + particles[i].state;
    //cout << "i; " << i << endl;
    //cout << "maxWeight: " << maxWeight << endl;
    //cout << "w: " << w << endl;
    //imshow("olImage", olImage);
    //waitKey(0);
    //imshow("olImage", olImage);
    //waitKey(0);
  }
  //cout << "iters:" << iterations << endl;
  avgFilteredState.x = avgFilteredState.x / nParticles;
  avgFilteredState.y = avgFilteredState.y / nParticles;
  avgFilteredState.theta = avgFilteredState.theta / nParticles;
  double avgWeights = sumWeights / nParticles;
  wSlow += ALPHA_SLOW * (avgWeights - wSlow);
  wFast += ALPHA_FAST * (avgWeights - wFast);
}

void
ParticleFilter::setupWithRandomSideLine()
{
  vector<RobotPose2D<float> > initialGuess;
  float x = 0.f;
  int numStatesOneSide = 10;
  for (int i = 0; i < numStatesOneSide; ++i) {
    initialGuess.push_back(RobotPose2D<float>(x, 3.f, -M_PI_2));
    initialGuess.push_back(RobotPose2D<float>(x, -3.f, M_PI_2));
    x -= 4.5 / numStatesOneSide;
  }
  init(initialGuess);
}

void
ParticleFilter::setupWithTeamPositions()
{
  vector<RobotPose2D<float> > initialGuess;
  unsigned kickOffTeam =
    IVAR(RoboCupGameControlData, LocalizationModule::gameData).kickOffTeam;
  unsigned ourTeamNumber =
    IVAR(RoboCupGameControlData, LocalizationModule::gameData).teams[0].teamNumber;
  if (true) { //kickOffTeam == ourTeamNumber
    for (int i = 0; i < 5; ++i) {
      initialGuess.push_back(sidePositionsAtt[i]);
    }
  } else {
    for (int i = 0; i < 5; ++i) {
      initialGuess.push_back(sidePositionsDef[i]);
    }
  }
  init(initialGuess);
}

void
ParticleFilter::minMaxGroundDist()
{
  vector<Matrix4f> T(NUM_CAMS);
  vector<Vector3f> unitVecYT(NUM_CAMS);
  vector<float> slopes(NUM_CAMS);
  T[TOP_CAM] = IVAR(Matrix4f, LocalizationModule::upperCamInFeet);
  T[BOTTOM_CAM] = IVAR(Matrix4f, LocalizationModule::lowerCamInFeet);
  for (unsigned i = 0; i < NUM_CAMS; ++i) {
    unitVecYT[i] = T[i].block(0, 0, 3, 3) * unitVecY[i];
    slopes[i] = unitVecYT[i][0] / unitVecYT[i][2];
  }
  minDistGround =
    slopes[BOTTOM_CAM] * (0.0 - T[BOTTOM_CAM](2, 3)) + T[BOTTOM_CAM](0, 3);
}

bool
ParticleFilter::normalizeWeights(const bool& noData)
{
  if (noData) {
    for (int i = 0; i < nParticles; ++i)
      particles[i].weight = 1.0 / nParticles;
    return true;
  } else {
    //cout << "maxWeight: " << maxWeight << endl;
    if (maxWeight < 1e-9) {
      lostCount++;
      OVAR(int, LocalizationModule::positionConfidence) =
        (1 - lostCount / 20) * 100;
      if (OVAR(int, LocalizationModule::positionConfidence) < 50) {
        reset();
        lostCount = 0;
        return false;
      } else {
        for (int i = 0; i < nParticles; ++i)
          particles[i].weight = 1.0 / nParticles;
        return true;
      }
    } else {
      for (int i = 0; i < nParticles; ++i)
        particles[i].weight = particles[i].weight / sumWeights;
      lostCount = 0;
      return true;
    }
  }
}

void
ParticleFilter::resample()
{
  vector<Particle> resampled;
  double rNumber = random.FloatU() * (1.0 / nParticles);
  double w = particles[0].weight;
  int j = 0;
  double vl = 0;
  double threshold = 1.0 - (wFast / wSlow);
  if (threshold > vl) vl = threshold;
  for (unsigned i = 0; i < nParticles; ++i) {
    /*if (random.FloatU() < vl) {
     NormalDistribution<double> distX;
     NormalDistribution<double> distY;
     NormalDistribution<double> distTheta;
     Particle p;
     p.state.x = distX(random, avgFilteredState.x, 0.25 / 3);
     p.state.y = distY(random, avgFilteredState.y, 0.25 / 3);
     p.state.theta = distTheta(random, avgFilteredState.theta, 10 * M_PI / 180 / 3);
     p.weight = 1.0 / nParticles;
     resampled.push_back(p);
     } else {*/
    double u = rNumber + i / (double) nParticles;
    while (u > w) {
      j++;
      w += particles[j].weight;
    }
    resampled.push_back(particles[j]);
    //}
  }
  /*RandomSelect<double> distr(weights.begin(), weights.end());
   for (unsigned i = 0; i < nParticles; ++i) {
   if(particles[i].weight > 0.3) {
   resampled[i] = particles[i];
   } else {
   int index = distr(random);
   resampled[i] = particles[index];
   }
   resampled[i].weight = 1.0 / nParticles;
   }*/
  for (unsigned i = 0; i < nParticles; ++i) {
    resampled[i].weight = 1.0 / nParticles;
  }
  particles = resampled;
}

void
ParticleFilter::checkIfLocalized()
{
  if (particles.empty()) {
    localized = false;
    OVAR(int, LocalizationModule::positionConfidence) = 0;
  } else {
    localized = true;
    for (int i = 0; i < nParticles; ++i) {
      double dist = MathsUtils::dist(
        particles[i].state.x,
        particles[i].state.y,
        avgFilteredState.x,
        avgFilteredState.y);
      if (dist > 0.5) {
        localized = false;
        OVAR(int, LocalizationModule::positionConfidence) = 0;
        return;
      }
    }
    // Localized so set lastknownposition to new position
    OVAR(RobotPose2D<float>, LocalizationModule::lastKnownPose2D) =
      avgFilteredState;
    OVAR(int, LocalizationModule::positionConfidence) = 100;
  }
}

void
ParticleFilter::updateSideConfidence()
{
  GoalInfo goalInfo = IVAR(GoalInfo, LocalizationModule::goalInfo);
  if (prevGoalInfoId != 0 && goalInfo.id != prevGoalInfoId) {
    if (goalInfo.found) {
      Point2f goalInMap;
      auto ct = cos(avgFilteredState.theta);
      auto st = sin(avgFilteredState.theta);
      goalInMap.x =
        avgFilteredState.x + goalInfo.mid.x * ct - goalInfo.mid.y * st;
      goalInMap.y =
        avgFilteredState.y + goalInfo.mid.x * st + goalInfo.mid.y * ct;
      float distOurs = norm(goalInMap - Point2f(-4.5, 0));
      float distOpp = norm(goalInMap - Point2f(4.5, 0));
      auto& sideConfidence = OVAR(int, LocalizationModule::sideConfidence);
      if (distOurs < distOpp) {
        if (goalInfo.ours == 0) {
          // If seen goal post is closer to our goal post
          // but classified as opponents goalpost
          // Decrease the side confidence
          sideConfidence *= 0.90;
        } else if (goalInfo.ours == 1) {
          sideConfidence /= 0.90;
          sideConfidence = sideConfidence >= 100 ? 100 : sideConfidence;
        }
      } else {
        if (goalInfo.ours == 1) {
          // If seen goal post is closer to our opponent's post
          // but classified as our goalpost
          // Decrease the side confidence
          sideConfidence *= 0.90;
        } else if (goalInfo.ours == 0) {
          sideConfidence /= 0.90;
          sideConfidence = sideConfidence >= 100 ? 100 : sideConfidence;
        }
      }
      OVAR(int, LocalizationModule::sideConfidence) = sideConfidence;
    }
  }
  prevGoalInfoId = goalInfo.id;

  if (OVAR(int, LocalizationModule::sideConfidence) <= 30) {
    // Mirror all particles
    for (unsigned i = 0; i < nParticles; ++i) {
      particles[i].state.x = -particles[i].state.x;
      particles[i].state.y = -particles[i].state.y;
      particles[i].state.theta = particles[i].state.theta + M_PI;
    }
    OVAR(int, LocalizationModule::sideConfidence) = 70;
  }
}
