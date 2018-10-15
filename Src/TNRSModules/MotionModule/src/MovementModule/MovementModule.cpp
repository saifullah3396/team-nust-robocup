/**
 * @file MotionModule/MovementModule/MovementModule.h
 *
 * This file declares the class for the robot navigation.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 06 Oct 2017
 */


#include "MotionModule/include/MotionConfigs/MBHeadControlConfig.h"
#include "MotionModule/include/MotionConfigs/MBPostureConfig.h"
#include "MotionModule/include/MovementModule/MovementModule.h"

template <typename Scalar>
MovementModule<Scalar>::MovementModule(
  MotionModule* motionModule,
  const BehaviorConfigPtr& config,
  const string& name) :
  MotionBehavior<Scalar>(motionModule, config, name),
  DebugBase(name, this),
  pathPlanner(motionModule->getPathPlanner()), 
  pathPlanned(false),
  inWalk(false),
  currentStep(0)
{
  initDebugBase();
}

template <typename Scalar>
boost::shared_ptr<MBMovementConfig> MovementModule<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <MBMovementConfig> (this->config);
}

template <typename Scalar>
boost::shared_ptr<MovementModule<Scalar> > MovementModule<Scalar>::getType(
  MotionModule* motionModule, const BehaviorConfigPtr& cfg) 
{ 
  MovementModule<Scalar>* mm;
  switch (cfg->type) {
      case (unsigned) MBMovementTypes::GO_TO_TARGET: 
        mm = new MovementModule<Scalar>(motionModule, cfg); break;
      default: mm = new MovementModule<Scalar>(motionModule, cfg); break;
  }
  return boost::shared_ptr<MovementModule<Scalar> >(mm);
}

template <typename Scalar>
void MovementModule<Scalar>::loadExternalConfig()
{
  static bool loaded = false;
  if (!loaded) {
    int tempDebug;
    int tempSendFootsteps;
    GET_CONFIG(
      "MovementModule",
      (int, debug, tempDebug), 
      (int, sendFootsteps, tempSendFootsteps), 
    )
    SET_DVAR(int, debug, tempDebug);
    SET_DVAR(int, sendFootsteps, tempSendFootsteps);

    //! read parameters from config file:
    GET_CONFIG(
      "PathPlanner",
      (Scalar, Walk.footStepTime, footStepTime), 
      (bool, PathPlanner.forwardSearch, forwardSearch), 
    );
    footstepsImage = Mat(Size(1000, 700), CV_8UC3, cv::Scalar(0, 0, 0));
    loaded = true;
  }
}

template <typename Scalar>
void MovementModule<Scalar>::initiate()
{
  PRINT("Initiating MovementModule")
  goal = getBehaviorCast()->goal;
  if (getBehaviorCast()->htConfig)
    this->setupChildRequest(getBehaviorCast()->htConfig, true);
  
  if (getBehaviorCast()->postureConfig)
    behaviorState = setPosture;
  else
    behaviorState = move;
    
  if (setGoal(goal)) {
    PRINT("MovementModule.setGoal() successful. Continuing...")
  } else {
    if (getBehaviorCast()->reachClosest) {
      if (findPossibleGoal(goal)) {
        PRINT("MovementModule.setGoal() successful. Continuing...")
      } else {
        ERROR("MovementModule.setGoal() failed. Exiting...")
        this->inBehavior = false;
        return;
      }
    } else {
      ERROR("MovementModule.setGoal() failed. Exiting...")
    }
  }
  if (!setStart()) {
    ERROR("MovementModule.setStart() failed. Exiting...")
    this->inBehavior = false;
    return;
  }
  this->inBehavior = true;
  PRINT("MovementModule initiation successful...")
}

template <typename Scalar>
void MovementModule<Scalar>::reinitiate()
{
  if (getBehaviorCast()->htConfig)
    this->setupChildRequest(getBehaviorCast()->htConfig, true);
  goal = getBehaviorCast()->goal;
  if (setGoal(goal)) {
    PRINT("MovementModule.setGoal() successful. Continuing...")
    pathPlanned = false;
    inWalk = false;
  } else {
    if (getBehaviorCast()->reachClosest) {
      if (findPossibleGoal(goal)) {
        PRINT("MovementModule.setGoal() successful. Continuing...")
        pathPlanned = false;
        inWalk = false;
      } else {
        ERROR("MovementModule.setGoal() failed. Exiting...")
        this->inBehavior = false;
        return;
      }
    } else {
      ERROR("MovementModule.setGoal() failed. Exiting...")
    }
  }
}

template <typename Scalar>
void MovementModule<Scalar>::update()
{
  PRINT("Updating MovementModule")
  if (behaviorState == setPosture) {
    if (this->lastChildCfg && 
        this->lastChildCfg->id == (unsigned)MBIds::POSTURE) 
    {
      behaviorState = move;
    } else {
      if (getBehaviorCast()->postureConfig)
        this->setupChildRequest(getBehaviorCast()->postureConfig);
    }
  } else if (behaviorState == move) {   
    if (!pathPlanned) {
      if (forwardSearch) {
        replan();
      } else {
        plan();
      }
    }
    
    if (pathPlanned) 
      executeWalk();
    else 
      finish();
  } else if (behaviorState == stop) {
    if (this->lastChildCfg && 
        this->lastChildCfg->id == (unsigned)MBIds::POSTURE) 
    {
      this->inBehavior = false;
    } else {
      if (getBehaviorCast()->postureConfig)
        this->setupChildRequest(getBehaviorCast()->postureConfig);
    }
  }
  OVAR(bool, MotionModule::robotInMotion) = inWalk;
}

template <typename Scalar>
void MovementModule<Scalar>::finish()
{
  this->killChild();
  this->motionProxy->stopMove();
  inWalk = false;
  behaviorState = stop;
}

template <typename Scalar>
bool MovementModule<Scalar>::setStart()
{
  Matrix<Scalar, 4, 4> lFootT, rFootT;
  //! get real placement of the feet
  if (!getFootTransform(lFootT, LEFT) || 
      !getFootTransform(rFootT, RIGHT))
    return false;
  auto lTheta = 
    MathsUtils::matToEuler(
      (Matrix<Scalar, 3, 3>) lFootT.block(0, 0, 3, 3)
    )[2];
  State left(lFootT(0, 3), lFootT(1, 3), lTheta, LEFT);
  auto rTheta = 
    MathsUtils::matToEuler(
      (Matrix<Scalar, 3, 3>) rFootT.block(0, 0, 3, 3)
    )[2];
  State right(rFootT(0, 3), rFootT(1, 3), rTheta, RIGHT);
  //cout << "Robot standing at: "
  //     << left.getX() << ", " << left.getY() << ", " << left.getTheta() << ", " << left.getLeg() << ", "
  //     << right.getX() << ", " << right.getY() << ", " << right.getTheta() << ", " << right.getLeg()
  //     << endl;
  return pathPlanner->setStart(left, right);
}

template <typename Scalar>
bool MovementModule<Scalar>::setStart(const State& left, const State& right)
{

  return pathPlanner->setStart(left, right);
}

template <typename Scalar>
bool MovementModule<Scalar>::setGoal(const RobotPose2D<Scalar>& goal)
{
  return pathPlanner->setGoal(goal.x, goal.y, goal.theta);
}

template <typename Scalar>
bool MovementModule<Scalar>::findPossibleGoal(RobotPose2D<Scalar>& goal)
{
  cout << "goal.x: " << goal.x << endl;
  cout << "goal.y: " << goal.y << endl;
  cout << "goal.theta: " << goal.theta << endl;
  auto& rPose = 
    IVAR(RobotPose2D<float>, MotionModule::robotPose2D);
  Matrix<Scalar, 2, 1> unit;
  Scalar diffStep = 0.10;
  unit[0] = rPose.x - goal.x;
  unit[1] = rPose.y - goal.y;
  unit = unit / unit.norm();
  int iter = 0;
  while (true) {
    if (iter > 10) break;
    goal.x = goal.x + unit[0] * diffStep;
    goal.y = goal.y + unit[1] * diffStep;
    cout << "goal.x: " << goal.x << endl;
    cout << "goal.y: " << goal.y << endl;
    cout << "goal.theta: " << goal.theta << endl;
    if (setGoal(goal)) {
      return true;
    }
    ++iter;
  }
  return false;
}

template <typename Scalar>
bool MovementModule<Scalar>::getFootTransform(
  Matrix<Scalar, 4, 4>& foot, const unsigned& id)
{
  Matrix<Scalar, 4, 4> torsoFrameT;
  if (!pathPlanned) {
    auto& robotPose2D =
      IVAR(RobotPose2D<float>, MotionModule::robotPose2D);
    Matrix<Scalar, 3, 3> rotation;
    MathsUtils::makeRotationZ(rotation, robotPose2D.theta);
    torsoFrameT = 
      MathsUtils::makeTransformation(
        rotation,
        robotPose2D.x,
        robotPose2D.y,
        0.0
      );
  } else {
    if (id == LEFT) torsoFrameT =
      prevRFoot * MathsUtils::getTInverse(
        (Matrix<Scalar, 4, 4>) 
        OVAR(Matrix4f, MotionModule::rFootOnGround).template cast<Scalar>()
      );
    else if (id == RIGHT) torsoFrameT =
      prevLFoot * MathsUtils::getTInverse(
        (Matrix<Scalar, 4, 4>) 
        OVAR(Matrix4f, MotionModule::lFootOnGround).template cast<Scalar>()
      );
  }
  try {
    if (id == LEFT) {
      foot = 
        torsoFrameT * 
        OVAR(Matrix4f, MotionModule::lFootOnGround).template cast<Scalar>();
      prevLFoot = foot;
    } else if (id == RIGHT) {
      foot = 
        torsoFrameT * 
        OVAR(Matrix4f, MotionModule::rFootOnGround).template cast<Scalar>();
      prevRFoot = foot;
    }
  } catch (const exception& e) {
    return false;
  }
  return true;
}

template <typename Scalar>
void MovementModule<Scalar>::replanIfPathInvalid()
{
  if (!pathPlanner->checkGoal()) {
    PRINT("PathPlanner.checkGoal() failed. Goal not reachble anymore.")
    finish();
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
    inWalk = false;
  }
}

template <typename Scalar>
void MovementModule<Scalar>::plan()
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

template <typename Scalar>
void MovementModule<Scalar>::replan()
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

template <typename Scalar>
void MovementModule<Scalar>::setupNaoQiWalk()
{
  const State* fromPlanned = toPlanned.base();
  toPlanned++;
  AL::ALValue footName;
  AL::ALValue footSteps;
  AL::ALValue timeList;
  footName.arraySetSize(pathPlanner->getPathSize() - 1);
  footSteps.arraySetSize(pathPlanner->getPathSize() - 1);
  timeList.arraySetSize(pathPlanner->getPathSize() - 1);
  footstepsImage = Scalar(0);
  
  footRects.clear();
  stepPositionUpdates.clear();
  int stepIndex = 0;
  // This is used to find the projected position of robot with respect
  // to the stepping foot
  Scalar feetMidShift = (*toPlanned).getLeg() == LEFT ? -0.05 : 0.05;
  while (toPlanned != pathPlanner->getPathEnd()) {
    // Get a step from the planned path
    State step;
    pathPlanner->getFootstep<Scalar>(*fromPlanned, *toPlanned, step);
    
    Matrix<Scalar, 3, 1> torsoToSupportFoot;
    if (step.getLeg() == LEFT) {
      // From torso to right foot support frame 
      torsoToSupportFoot = 
        (OVAR(Matrix4f, MotionModule::rFootOnGround).template cast<Scalar>())
        .block(0, 3, 3, 1); 
    } else {
      // From torso to left foot support frame
      torsoToSupportFoot = 
        (OVAR(Matrix4f, MotionModule::lFootOnGround).template cast<Scalar>())
        .block(0, 3, 3, 1); 
    }
    PositionInput<float> pi = // float used since localization request requires float
      PositionInput<float>(
        torsoToSupportFoot[0] + step.getX(),
        torsoToSupportFoot[1] + step.getY(),
        torsoToSupportFoot[2] + step.getTheta()
      ); 
    stepPositionUpdates.push_back(pi);
    
    // Generate a NaoQi footstep array
    footSteps[stepIndex].arraySetSize(3);
    // Foot type
    if (step.getLeg() == LEFT) footName[stepIndex] = "LLeg";
    else if (step.getLeg() == RIGHT) footName[stepIndex] = "RLeg";
    // Foot position
    footSteps[stepIndex][0] = step.getX();
    footSteps[stepIndex][1] = step.getY();
    footSteps[stepIndex][2] = step.getTheta();
    // Time taken by footstep
    timeList[stepIndex] = (stepIndex + 1) * footStepTime; 
    
    RotatedRect footRect = 
      RotatedRect(
        Point2f(500 + (*toPlanned).getX() * 100, 350 - (*toPlanned).getY() * 100),
        Size2f(0.16 * 100, 0.06 * 100),
        -(*toPlanned).getTheta() * 180 / M_PI
      );
    footRects.push_back(footRect);
    if (stepIndex == 0) 
      VisionUtils::drawRRect(footstepsImage, footRect, cv::Scalar(255, 0, 0));
    else if (stepIndex == pathPlanner->getPathSize() - 2) 
      VisionUtils::drawRRect(footstepsImage, footRect, cv::Scalar(0, 0, 255));
    else
      VisionUtils::drawRRect(footstepsImage, footRect, cv::Scalar(255, 255, 255));
    ++stepIndex;
    /*cout << "state:" << endl;
     cout << "sX: " << (*toPlanned).getX() << endl;
     cout << "sY: " << (*toPlanned).getY() << endl;
     cout << "sT: " << (*toPlanned).getTheta() << endl;
     cout << "sT: " << (*toPlanned).getLeg() << endl;*/
    fromPlanned = toPlanned.base();
    toPlanned++;
  }
  //imshow("footstepsImage", footstepsImage);
  //waitKey(0);
  startStep = IVAR(int, MotionModule::nFootsteps);
  currentStep = 0;
  prevStep = -1;
  this->motionProxy->setFootSteps(footName, footSteps, timeList, true);
  inWalk = true;
  OVAR(RobotPose2D<float>, MotionModule::moveTarget) = goal;
  OVAR(vector<RotatedRect>, MotionModule::footRects) = footRects;
}

template <typename Scalar>
void MovementModule<Scalar>::sendFootStepsData()
{
  vector <RotatedRect> footRects =
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
    string footstepsStr = DataUtils::bytesToHexString(ptr, count);
    CommMessage msg(footstepsStr, CommMsgTypes::FOOTSTEPS);
    SendMsgRequestPtr smr = 
      boost::make_shared<SendMsgRequest>(msg);
    BaseModule::publishModuleRequest(smr);
  }
}

template <typename Scalar>
void MovementModule<Scalar>::executeWalk()
{
  //VisionUtils::displayImage(footstepsImage, "planned footsteps");
  //VisionUtils::displayImage(IVAR(OccupancyMap, MotionModule::occupancyMap).data, "map");
  //AL::ALValue fs = motionProxy->getFootSteps();
  //cout << fs[1] << endl;
  //cout << "goalx: " << goal.x << endl;
  //cout << "goaly: " << goal.y << endl;
  //cout << "goaltheta: " << goal.theta << endl;
  //PRINT("Start walking towards the goal.")
  if (!inWalk) {
    setupNaoQiWalk();
  } else {
    if (GET_DVAR(int, debug) &&
        GET_DVAR(int, sendFootsteps)) 
    {
      sendFootStepsData();
    }
    currentStep = IVAR(int, MotionModule::nFootsteps) - startStep;
    if (currentStep > pathPlanner->getPathSize() - 2) { // finished all steps
      if (!this->motionProxy->moveIsActive()) {
        OVAR(PostureState, MotionModule::postureState) = 
          PostureState::STAND_WALK;
        finish();
      }
      return;
    } else { // A new step is just started
      if (currentStep != prevStep) {
        // Get the time at which a new step is started
        footStepStartTime = this->motionModule->getModuleTime();
        prevStep = currentStep;
      } else { // A step is in motion
        // Get the time passed for the step in progress
        Scalar stepTimePassed = 
          this->motionModule->getModuleTime() - footStepStartTime;
        
        bool stepInProgress;
        if (currentStep == 0) { // If this is first step
          // First step always takes more time naoqi's fault.
          // Therefore add a 200 milliseconds of time in addition
          static Scalar overheadTime = 0.2;
          stepInProgress = 
            stepTimePassed <= footStepTime + overheadTime || 
            stepTimePassed - footStepTime + overheadTime < 0.01;
        } else { 
          stepInProgress = 
            stepTimePassed <= footStepTime || 
            stepTimePassed - footStepTime < 0.01;
        }
        if (stepInProgress) { // If the step is still in progress
          // ratio of time passed
          Scalar timeRatio;
          if (currentStep == 0) {
            // First step always takes more time.
            // Therefore add a 200 milliseconds of time in addition
            timeRatio = stepTimePassed / (footStepTime + 0.2); 
          } else {
            timeRatio = stepTimePassed / footStepTime;
          }
          // Set the position input to localization module from current
          // odometry data
          stepPositionUpdates[currentStep].x *= timeRatio;
          stepPositionUpdates[currentStep].y *= timeRatio;
          stepPositionUpdates[currentStep].theta *= timeRatio;
          PositionUpdatePtr pu = 
            boost::make_shared<PositionUpdate>(stepPositionUpdates[currentStep]);
          BaseModule::publishModuleRequest(pu);
        }
        replanIfPathInvalid();
        /*if (motionModule->getModuleTime()  > 7 ) {
           static bool once = false;
           if (!once) {
             goal = RobotPose2D<float>(2.0, 0.0, 0.0);
             setGoal(goal);
             pathPlanned = false;
             inWalk = false;
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
    }
  }
}

template class MovementModule<MType>;
