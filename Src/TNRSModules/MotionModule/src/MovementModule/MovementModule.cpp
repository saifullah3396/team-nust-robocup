/**
 * @file MotionModule/MovementModule/MovementModule.h
 *
 * This file declares the class for the robot navigation.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 06 Oct 2017
 */

#include "MotionModule/include/MovementModule/MovementModule.h"

void
MovementModule::setBehaviorConfig(
  boost::shared_ptr<BehaviorConfig> behaviorConfig)
{
  this->behaviorConfig =
    (boost::static_pointer_cast < MBMovementConfig > (behaviorConfig));
}

boost::shared_ptr<MBMovementConfig>
MovementModule::getBehaviorCast()
{
  return boost::static_pointer_cast < MBMovementConfig > (behaviorConfig);
}

void
MovementModule::loadInitialConfig()
{
  behaviorConfig = boost::make_shared<MBMovementConfig>();
}

void
MovementModule::loadStaticConfig()
{
  int tempDebug;
  int tempSendFootsteps;
  GET_CONFIG(
    "MovementModule",
    (int, debug, tempDebug), (int, sendFootsteps, tempSendFootsteps), )
  SET_DVAR(int, debug, tempDebug);
  SET_DVAR(int, sendFootsteps, tempSendFootsteps);

  //! read parameters from config file:
  GET_CONFIG(
    "PathPlanner",
    (double, PathPlanner.accuracyX, accuracyX), (double, PathPlanner.accuracyY, accuracyY), (double, PathPlanner.accuracyTheta, accuracyTheta), (double, Map.cellSize, cellSize), (int, PathPlanner.numAngleBins, numAngleBins), (bool, PathPlanner.forwardSearch, forwardSearch), (double, PathPlanner.maxStepX, footMaxStepX), (double, PathPlanner.maxStepY, footMaxStepY), (double, PathPlanner.maxStepTheta, footMaxStepTheta), (double, PathPlanner.maxStepInvX, footMaxStepInvX), (double, PathPlanner.maxStepInvY, footMaxStepInvY), (double, PathPlanner.maxStepInvTheta, footMaxStepInvTheta), (double, Walk.footStepTime, footStepTime), );

  vector<double> stepRangeX = vector<double>(
    stepRanges[0],
    stepRanges[0] + sizeof(stepRanges[0]) / sizeof(stepRanges[0][0]));
  vector<double> stepRangeY = vector<double>(
    stepRanges[1],
    stepRanges[1] + sizeof(stepRanges[1]) / sizeof(stepRanges[1][0]));

  ASSERT(stepRangeX.size() == stepRangeY.size());
  //! create step range
  stepRange.clear();
  stepRange.reserve(stepRangeX.size());
  double x, y;
  for (int i = 0; i < stepRangeX.size(); ++i) {
    x = (double) stepRangeX[i];
    y = (double) stepRangeY[i];
    stepRange.push_back(pair<double, double>(x, y));
  }
  //! insert first point again at the end!
  stepRange.push_back(stepRange[0]);
  prevFeetMidP.setZero();
  footstepsImage = Mat(Size(1000, 700), CV_8UC3, Scalar(0, 0, 0));
}

MovementModule::MovementModule(MotionModule* motionModule) :
  MotionBehavior(motionModule), DebugBase("MovementModule", this),
    motionProxy(motionModule->getSharedMotionProxy()),
    pathPlanner(motionModule->getPathPlanner()), pathPlanned(false),
    proceedToFinish(false)
{
  initDebugBase();
  loadStaticConfig();
  loadInitialConfig();
}

void
MovementModule::initiate()
{
  PRINT("Initiating MovementModule")
  if (getBehaviorCast()->ballTrack) {
    htModule = boost::shared_ptr < HeadTracking > (new HeadTracking(
      motionModule));
    auto htConfig =
      boost::make_shared < MBHeadTrackingConfig > (MBHeadTrackingTypes::FOLLOW_BALL);
    htModule->setBehaviorConfig(htConfig);
    htModule->initiate();
  }
  postureModule = boost::shared_ptr < PostureModule > (new PostureModule(
    motionModule));
  auto pConfig = boost::make_shared<MBPostureConfig>();
  pConfig->timeToReachP = 1.f;
  pConfig->posture = PostureState::STAND_WALK;
  postureModule->setBehaviorConfig(pConfig);
  postureModule->initiate();

  goal =
    (boost::static_pointer_cast < MBMovementConfig > (behaviorConfig))->goal;
  if (setGoal(goal)) {
    PRINT("MovementModule.setGoal() successful. Continuing...")
  } else {
    if (getBehaviorCast()->reachClosest) {
      if (findRandomCloseGoal(goal)) {
        PRINT("MovementModule.setGoal() successful. Continuing...")
      } else {
        ERROR("MovementModule.setGoal() failed. Exiting...")
        inBehavior = false;
        return;
      }
    } else {
      ERROR("MovementModule.setGoal() failed. Exiting...")
    }
  }
  if (!setStart()) {
    ERROR("MovementModule.setStart() failed. Exiting...")
    inBehavior = false;
    return;
  }
  OVAR(bool, MotionModule::robotInMotion) = false;
  inBehavior = true;
  PRINT("MovementModule initiation successful...")
}

void
MovementModule::reinitiate()
{
  if (getBehaviorCast()->ballTrack) {
    if (!htModule) {
      htModule = boost::shared_ptr < HeadTracking > (new HeadTracking(
        motionModule));
      auto htConfig =
        boost::make_shared < MBHeadTrackingConfig > (MBHeadTrackingTypes::FOLLOW_BALL);
      htModule->setBehaviorConfig(htConfig);
      htModule->initiate();
    }
  }
  goal =
    (boost::static_pointer_cast < MBMovementConfig > (behaviorConfig))->goal;
  if (setGoal(goal)) {
    PRINT("MovementModule.setGoal() successful. Continuing...")
    pathPlanned = false;
    OVAR(bool, MotionModule::robotInMotion) = false;
  } else {
    if (getBehaviorCast()->reachClosest) {
      if (findRandomCloseGoal(goal)) {
        PRINT("MovementModule.setGoal() successful. Continuing...")
        pathPlanned = false;
        OVAR(bool, MotionModule::robotInMotion) = false;
      } else {
        ERROR("MovementModule.setGoal() failed. Exiting...")
        inBehavior = false;
        return;
      }
    } else {
      ERROR("MovementModule.setGoal() failed. Exiting...")
    }
  }
}

void
MovementModule::update()
{
  if (!inBehavior) return;

  if (proceedToFinish) {
    if (!postureModule->getState()) {
      postureModule->update();
    } else {
      inBehavior = false;
    }
    return;
  }

  if (behaviorConfig->killBehavior) {
    finishBehaviorSafely();
    return;
  }

  if (!postureModule->getState()) {
    postureModule->update();
    return;
  }

  if (!pathPlanned) {
    if (forwardSearch) {
      replan();
    } else {
      plan();
    }
  }

  if (pathPlanned) updateWalk();
  else finishBehaviorSafely();
}

void
MovementModule::finishBehaviorSafely()
{
  OVAR(bool, MotionModule::robotInMotion) = false;
  motionProxy->stopMove();
  postureModule->initiate();
  proceedToFinish = true;
}

bool
MovementModule::findRandomCloseGoal(RobotPose2D<float>& goal)
{
  cout << "goal.x: " << goal.x << endl;
  cout << "goal.y: " << goal.y << endl;
  cout << "goal.theta: " << goal.theta << endl;
  RobotPose2D<float> rPose = IVAR(RobotPose2D<float>, MotionModule::robotPose2D);
  Point2f unit(0.f, 0.f);
  unit.x = rPose.x - goal.x;
  unit.y = rPose.y - goal.y;
  float unitNorm = norm(unit);
  unit.x = unit.x / unitNorm;
  unit.y = unit.y / unitNorm;
  int iter = 0;
  while (true) {
    if (iter > 10) break;
    goal.x = goal.x + unit.x * 0.10;
    goal.y = goal.y + unit.y * 0.10;
    cout << "goal.x: " << goal.x << endl;
    cout << "goal.y: " << goal.y << endl;
    cout << "goal.theta: " << goal.theta << endl;
    if (setGoal(goal)) {
      return true;
    }
    iter++;
  }
  return false;
}

bool
MovementModule::setGoal(const RobotPose2D<float>& goal)
{
  return pathPlanner->setGoal(goal.x, goal.y, goal.theta);
}

void
MovementModule::checkPathValidity()
{
  if (!pathPlanner->checkGoal()) {
    PRINT("PathPlanner.checkGoal() failed. Goal not reachble anymore.")
    finishBehaviorSafely();
  }

  if (!pathPlanner->checkPathValidity()) {
    PRINT("PathPlanner.checkPathValidity() failed. Path no more valid.")
    PRINT("Replanning...")
    if ((*toPlanned).getLeg() == LEFT) {
      State left = *(pathPlanner->getPathBegin() + currentStep + 3);
      State right = *(pathPlanner->getPathBegin() + currentStep + 4);
      setStart(left, right);
    } else if ((*toPlanned).getLeg() == RIGHT) {
      State right = *(pathPlanner->getPathBegin() + currentStep + 3);
      State left = *(pathPlanner->getPathBegin() + currentStep + 4);
      setStart(left, right);
    }
    replan();
    //motionProxy->stopMove();
    //pathPlanned = false;
    OVAR(bool, MotionModule::robotInMotion) = false;
  }
}

void
MovementModule::plan()
{
  if (pathPlanner->plan()) {
    if (pathPlanner->getPathSize() <= 1) {
      pathPlanned = false;
    } else {
      toPlanned = pathPlanner->getPathBegin();
      pathPlanned = true;
      PRINT("PathPlanner.plan() successful.")
    }
  }
}

void
MovementModule::replan()
{
  bool pathExisted = pathPlanner->pathExists();

  //! calculate path by replanning (if no planning information exists
  //! this call is equal to pathPlanner->plan())
  if (pathPlanner->replan()) {
    PRINT("PathPlanner.replan() successful.")
    if (pathPlanner->getPathSize() <= 1) {
      pathPlanned = false;
      return;
    } else {
      toPlanned = pathPlanner->getPathBegin();
      pathPlanned = true;
      return;
    }
  } else if (pathExisted) {
    PRINT(
      "PathPlanner.replan() failed. Reseting previous planning information.")
    if (!setStart()) {
      pathPlanned = false;
      return;
    }
    if (pathPlanner->plan()) {
      if (pathPlanner->getPathSize() <= 1) {
        pathPlanned = false;
        return;
      } else {
        toPlanned = pathPlanner->getPathBegin();
        pathPlanned = true;
        return;
      }
    }
  }
  pathPlanned = false;
}

bool
MovementModule::setStart(const State& left, const State& right)
{

  return pathPlanner->setStart(left, right);
}

bool
MovementModule::setStart()
{
  Matrix4d lFootT, rFootT;
  //! get real placement of the feet
  if (!getFootTransform(lFootT, LEFT)) {
    return false;
  }

  if (!getFootTransform(rFootT, RIGHT)) {
    return false;
  }
  State left(
    lFootT(0, 3),
    lFootT(1, 3),
    MathsUtils::matToEuler((Matrix3d) lFootT.block(0, 0, 3, 3))[2],
    LEFT);
  State right(
    rFootT(0, 3),
    rFootT(1, 3),
    MathsUtils::matToEuler((Matrix3d) rFootT.block(0, 0, 3, 3))[2],
    RIGHT);
  //cout << "Robot standing at: "
  //     << left.getX() << ", " << left.getY() << ", " << left.getTheta() << ", " << left.getLeg() << ", "
  //     << right.getX() << ", " << right.getY() << ", " << right.getTheta() << ", " << right.getLeg()
  //     << endl;

  return pathPlanner->setStart(left, right);
}

bool
MovementModule::getFootTransform(Matrix4d& foot, unsigned id)
{
  Matrix4d torsoFrameT;
  if (!pathPlanned) {
    RobotPose2D<float> robotPose2D =
      IVAR(RobotPose2D<float>, MotionModule::robotPose2D);
    Matrix3d rotation;
    MathsUtils::makeRotationZ(rotation, robotPose2D.theta);
    startFrameT = MathsUtils::makeTransformation(
      rotation,
      robotPose2D.x,
      robotPose2D.y,
      0.0);
    torsoFrameT = startFrameT;
  } else {
    if (id == LEFT) torsoFrameT =
      startFrameT * prevRFoot * MathsUtils::getTInverse(
        (Matrix4d) OVAR(Matrix4f, MotionModule::rFootOnGround).cast<double>());
    else if (id == RIGHT) torsoFrameT =
      startFrameT * prevLFoot * MathsUtils::getTInverse(
        (Matrix4d) OVAR(Matrix4f, MotionModule::lFootOnGround).cast<double>());
  }
  try {
    if (id == LEFT) {
      foot = torsoFrameT * OVAR(Matrix4f, MotionModule::lFootOnGround).cast<
        double>();
      prevLFoot = foot;
    } else if (id == RIGHT) {
      foot = torsoFrameT * OVAR(Matrix4f, MotionModule::rFootOnGround).cast<
        double>();
      prevRFoot = foot;
    }
  } catch (const exception& e) {
    return false;
  }
  return true;
}

bool
MovementModule::getFootstep(const Matrix4d& from, const State& fromPlanned,
  const State& to, State& footstep)
{
  Matrix3d toRotation;
  MathsUtils::makeRotationZ(toRotation, to.getTheta());
  Matrix4d step;
  step = MathsUtils::getTInverse(from) * MathsUtils::makeTransformation(
    toRotation,
    to.getX(),
    to.getY(),
    0.0);

  //! set the footstep
  footstep.setX(step(0, 3));
  footstep.setY(step(1, 3));
  footstep.setTheta(
    MathsUtils::matToEuler((Matrix3d) step.block(0, 0, 3, 3))[2]);
  footstep.setLeg(to.getLeg());

  //! check if the step lies within the executable range
  if (performable(footstep)) {
    return true;
  } else {
    //! check if there is only a minor divergence between the current support
    //! foot and the foot placement used during the plannig task: in such a case
    //! perform the step that has been used during the planning
    float stepDiffX = fabs(from(0, 3) - fromPlanned.getX());
    float stepDiffY = fabs(from(1, 3) - fromPlanned.getY());
    float stepDiffTheta = fabs(
      angles::shortestAngularDistance(
        MathsUtils::matToEuler((Matrix3d) from.block(0, 0, 3, 3))[2],
        fromPlanned.getTheta()));
    if (stepDiffX < accuracyX && stepDiffY < accuracyY && stepDiffTheta < accuracyTheta) {
      Matrix3d fromRotation;
      MathsUtils::makeRotationZ(fromRotation, fromPlanned.getTheta());
      Matrix4d step;
      step = MathsUtils::getTInverse(
        MathsUtils::makeTransformation(
          fromRotation,
          fromPlanned.getX(),
          fromPlanned.getY(),
          0.0)) * MathsUtils::makeTransformation(
        toRotation,
        to.getX(),
        to.getY(),
        0.0);
      footstep.setX(step(0, 3));
      footstep.setY(step(1, 3));
      footstep.setTheta(
        MathsUtils::matToEuler((Matrix3d) step.block(0, 0, 3, 3))[2]);
      return true;
    }

    return false;
  }
}

void
MovementModule::getFootstep(const State& from, const State& to, State& footstep)
{
  Matrix3d toRotation;
  MathsUtils::makeRotationZ(toRotation, to.getTheta());
  Matrix3d fromRotation;
  MathsUtils::makeRotationZ(fromRotation, from.getTheta());

  Matrix4d step;
  step =
    MathsUtils::getTInverse(
      MathsUtils::makeTransformation(
        fromRotation,
        from.getX(),
        from.getY(),
        0.0)) * MathsUtils::makeTransformation(
      toRotation,
      to.getX(),
      to.getY(),
      0.0);
  footstep.setX(step(0, 3));
  footstep.setY(step(1, 3));
  footstep.setTheta(
    MathsUtils::matToEuler((Matrix3d) step.block(0, 0, 3, 3))[2]);
  footstep.setLeg(to.getLeg());
}

bool
MovementModule::performable(const State& footstep)
{
  float stepX = footstep.getX();
  float stepY = footstep.getY();
  float stepTheta = footstep.getTheta();

  if (footstep.getLeg() == RIGHT) {
    stepY = -stepY;
    stepTheta = -stepTheta;
  }

  if (stepX + FLOAT_CMP_THR > footMaxStepX || stepX - FLOAT_CMP_THR < footMaxStepInvX) return false;
  if (stepY + FLOAT_CMP_THR > footMaxStepY || stepY - FLOAT_CMP_THR < footMaxStepInvY) return false;
  if (stepTheta + FLOAT_CMP_THR > footMaxStepInvTheta || stepTheta - FLOAT_CMP_THR < footMaxStepInvTheta) return false;

  return performable(stepX, stepY);
}

bool
MovementModule::performable(float stepX, float stepY)
{
  int cn = 0;
  //! loop through all stepRange of the polygon
  for (unsigned int i = 0; i < stepRange.size() - 1; ++i) {
    if ((stepRange[i].second <= stepY && stepRange[i + 1].second > stepY) || (stepRange[i].second >= stepY && stepRange[i + 1].second < stepY)) {
      float vt =
        (float) (stepY - stepRange[i].second) / (stepRange[i + 1].second - stepRange[i].second);
      if (stepX < stepRange[i].first + vt * (stepRange[i + 1].first - stepRange[i].first)) {
        ++cn;
      }
    }
  }
  return cn & 1;
}

void
MovementModule::updateWalk()
{
  if (getBehaviorCast()->ballTrack) {
    htModule->update();
  }
  //VisionUtils::displayImage(footstepsImage, "planned footsteps");
  //VisionUtils::displayImage(IVAR(OccupancyMap, MotionModule::occupancyMap).data, "map");
  //AL::ALValue fs = motionProxy->getFootSteps();
  //cout << fs[1] << endl;
  //cout << "goalx: " << goal.x << endl;
  //cout << "goaly: " << goal.y << endl;
  //cout << "goaltheta: " << goal.theta << endl;
  //PRINT("Start walking towards the goal.")
  if (OVAR(bool, MotionModule::robotInMotion)) {
    if (GET_DVAR(int, debug)) {
      if (GET_DVAR(int, sendFootsteps)) {
        vector < RotatedRect > footRects =
          OVAR(vector<RotatedRect>, MotionModule::footRects);
        if (footRects.size() > 0) {
          Mat_<float> footRectsData(footRects.size(), 8);
          for (size_t i = 0; i < footRects.size(); ++i) {
            Point2f vertices[4];
            footRects[i].points(vertices);
            footRectsData(i, 0) = vertices[0].x;
            footRectsData(i, 1) = vertices[0].y;
            footRectsData(i, 2) = vertices[1].x;
            footRectsData(i, 3) = vertices[1].y;
            footRectsData(i, 4) = vertices[2].x;
            footRectsData(i, 5) = vertices[2].y;
            footRectsData(i, 6) = vertices[3].x;
            footRectsData(i, 7) = vertices[3].y;
          }
          const unsigned char* ptr = footRectsData.data;
          int count = footRects.size() * 32;
          string msg = DataUtils::bytesToHexString(ptr, count);
          CommModule::addToSpecialMsgQueue(
            pair<int, string>(MSG_FOOTSTEPS, msg));
        }
      }
    }
    currentStep = IVAR(int, MotionModule::nFootsteps) - startStep;
    //PRINT("MovementModule.prevStep: " << prevStep)
    //PRINT("MovementModule.currentStep: " << currentStep)
    if (currentStep > pathPlanner->getPathSize() - 2) {
      //PRINT("Final Step.")
      if (!motionProxy->moveIsActive()) {
        OVAR(PostureState, MotionModule::postureState) =
          PostureState::STAND_WALK;
        if (getBehaviorCast()->ballTrack) {
          htModule->finishBehaviorSafely();
        }
        finishBehaviorSafely();
      }
      return;
    }

    if (currentStep != prevStep) {
      //PRINT("Setting Step.")
      startFSTime = motionModule->getModuleTime();
      toPlanned++;
      currentFeetMid = (*toPlanned).getLeg() == LEFT ? -0.05 : 0.05;
      Matrix3f toRotation;
      MathsUtils::makeRotationZ(toRotation, (*toPlanned).getTheta());
      Matrix4f toTransformation = MathsUtils::makeTransformation(
        toRotation,
        (*toPlanned).getX(),
        (*toPlanned).getY(),
        0.0);
      prevFeetMidP = currentFeetMidP;
      currentFeetMidP = MathsUtils::transformVector(
        toTransformation,
        Vector3f(0.0, currentFeetMid, 0.0));
      currentFeetMidP[2] = (*toPlanned).getTheta();
      diffFeetMidP = currentFeetMidP - prevFeetMidP;
      OVAR(PositionInput, MotionModule::positionInput).inputs.push_back(
        diffFeetMidP);
      prevStep = currentStep;
      return;
    }

    /*float currentTime = motionModule->getModuleTime() - startFSTime;
     bool stepRangeExists;
     if (currentStep == 0) {
     stepRangeExists = currentTime <= footStepTime + 0.2 || currentTime - footStepTime + 0.2 < 0.01;
     // First step always takes more time naoqi's fault. :/
     } else {
     stepRangeExists = currentTime <= footStepTime || currentTime - footStepTime < 0.01;
     }
     if (stepRangeExists) {
     float timeParam;
     if (currentStep == 0) {
     timeParam = currentTime / (footStepTime + 0.2);
     // First step always takes more time naoqi's fault. :/
     } else {
     timeParam = currentTime / footStepTime;
     }
     Vector3f cState = diffFeetMidP * timeParam + prevFeetMidP;
     PositionInput pInput;
     pInput.cState =
     RobotPose2D<float>(cState[0], cState[1], cState[2]);
     pInput.execTime = currentTime - prevStepTime;
     pInput.id = (motionModule->getModuleTime() + 1) * 100;
     //cout << "sending control command" << endl;
     OVAR(PositionInput, MotionModule::positionInput) = pInput;
     //PRINT("pInput.id: " << pInput.id)
     //PRINT("pInput.cState.x: " << pInput.cState.x)
     //PRINT("pInput.cState.y: " << pInput.cState.y)
     //PRINT("pInput.cState.theta: " << pInput.cState.theta)
     prevStepTime = currentTime;
     }    */
    checkPathValidity();
    /*if (motionModule->getModuleTime()  > 7 ) {
     static bool once = false;
     if (!once) {
     goal = RobotPose2D<float>(2.0, 0.0, 0.0);
     setGoal(goal);
     pathPlanned = false;
     OVAR(bool, MotionModule::robotInMotion) = false;
     once = true;
     return;
     }
     //rectangle(IVAR(OccupancyMap, MotionModule::occupancyMap).data, Point(500 + 150, 350 + 50), Point(500 + 250, 350 - 50), Scalar(0), -1);
     cout << "currentStep: " << currentStep << endl;
     cout << "currentState: \n";
     cout << "cstate:" << endl;
     cout << "csX: " << (*toPlanned).getX() << endl;
     cout << "csY: " << (*toPlanned).getY() << endl;
     cout << "csT: " << (*toPlanned).getTheta() << endl;
     cout << "csT: " << (*toPlanned).getLeg() << endl;
     }*/
    return;
  }
  const State* fromPlanned = toPlanned.base();
  toPlanned++;
  AL::ALValue footName;
  AL::ALValue footSteps;
  AL::ALValue timeList;
  footName.arraySetSize(pathPlanner->getPathSize() - 1);
  footSteps.arraySetSize(pathPlanner->getPathSize() - 1);
  timeList.arraySetSize(pathPlanner->getPathSize() - 1);
  /*cout << "state:" << endl;
   cout << "sX: " << fromPlanned->getX() << endl;
   cout << "sY: " << fromPlanned->getY() << endl;
   cout << "sT: " << fromPlanned->getTheta() << endl;
   cout << "sT: " << fromPlanned->getLeg() << endl;*/
  footstepsImage = Scalar(0);
  int i = 0;
  footRects.clear();
  auto rState = IVAR(RobotPose2D<float>, MotionModule::robotPose2D);
  while (toPlanned != pathPlanner->getPathEnd()) {
    State step;
    getFootstep(*fromPlanned, *toPlanned, step);
    footSteps[i].arraySetSize(3);
    if (step.getLeg() == LEFT) footName[i] = "LLeg";
    if (step.getLeg() == RIGHT) footName[i] = "RLeg";
    footSteps[i][0] = step.getX();
    footSteps[i][1] = step.getY();
    footSteps[i][2] = step.getTheta();
    timeList[i] = (i + 1) * footStepTime;
    RotatedRect footRect = RotatedRect(
      Point2f(500 + (*toPlanned).getX() * 100, 350 - (*toPlanned).getY() * 100),
      Size2f(0.16 * 100, 0.06 * 100),
      -(*toPlanned).getTheta() * 180 / M_PI);
    footRects.push_back(footRect);
    Point2f vertices[4];
    footRect.points(vertices);
    for (int j = 0; j < 4; j++) {
      Scalar color(255, 255, 255);
      if (i == 0) color = Scalar(255, 0, 0);
      else if (i == pathPlanner->getPathSize() - 2) color = Scalar(0, 255, 0);
      line(footstepsImage, vertices[j], vertices[(j + 1) % 4], color);
    }
    ++i;
    /*cout << "state:" << endl;
     cout << "sX: " << (*toPlanned).getX() << endl;
     cout << "sY: " << (*toPlanned).getY() << endl;
     cout << "sT: " << (*toPlanned).getTheta() << endl;
     cout << "sT: " << (*toPlanned).getLeg() << endl;*/
    fromPlanned = toPlanned.base();
    toPlanned++;
  }
  OVAR(vector<RotatedRect>, MotionModule::footRects) = footRects;
  startStep = IVAR(int, MotionModule::nFootsteps);
  //PRINT("MovementModule.startStep: " << startStep)
  currentStep = 0;
  prevStep = -1;
  toPlanned = pathPlanner->getPathBegin();
  float currentFeetMid = (*toPlanned).getLeg() == LEFT ? -0.05 : 0.05;
  Matrix3f toRotation;
  MathsUtils::makeRotationZ(toRotation, (*toPlanned).getTheta());
  Matrix4f toTransformation = MathsUtils::makeTransformation(
    toRotation,
    (*toPlanned).getX(),
    (*toPlanned).getY(),
    0.0);
  currentFeetMidP = MathsUtils::transformVector(
    toTransformation,
    Vector3f(0.0, currentFeetMid, 0.0));
  currentFeetMidP[2] = (*toPlanned).getTheta();
  motionProxy->setFootSteps(footName, footSteps, timeList, true);
  OVAR(PositionInput, MotionModule::positionInput) = PositionInput();
  OVAR(PositionInput, MotionModule::positionInput).assignId();
  OVAR(PositionInput, MotionModule::positionInput).size =
    pathPlanner->getPathSize() - 1;
  OVAR(bool, MotionModule::robotInMotion) = true;
  OVAR(RobotPose2D<float>, MotionModule::moveTarget) = goal;
}
