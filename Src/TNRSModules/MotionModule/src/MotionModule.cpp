/**
 * @file MotionModule/src/MotionModule.cpp
 *
 * This file implements the class MotionModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "MotionModule/include/FallDetector/FallDetector.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/MotionRequest.h"
#include "MotionModule/include/PathPlanner/PathPlanner.h"
#include "MotionModule/include/PathPlanner/GridMap2D.h"
#include "MotionModule/include/TrajectoryPlanner/TrajectoryPlanner.h"
#include "MotionModule/include/MotionConfigs/MBBalanceConfig.h"
#include "MotionModule/include/MotionConfigs/MBPostureConfig.h"
#include "MotionModule/include/MotionConfigs/MBKickConfig.h"
#include "Utils/include/PostureState.h"

MotionModule::MotionModule(void* parent, ALMotionProxyPtr motionProxy) :
  BaseModule(parent, (unsigned) TNSPLModules::MOTION, "MotionModule"),
  motionProxy(motionProxy)
{
}

void MotionModule::setThreadPeriod()
{
  setPeriodMinMS(IVAR(int, MotionModule::motionThreadPeriod));
}

void MotionModule::initMemoryConn()
{
  ASSERT_MSG(sharedMemory, "Shared Memory not found.");
  genericInputConnector = 
    new InputConnector(this, getModuleName() + "InputConnector");
  genericOutputConnector = 
    new OutputConnector(this, getModuleName() + "OutputConnector");
  genericInputConnector->initConnector();
  genericOutputConnector->initConnector();
}

void MotionModule::init()
{
  ASSERT_MSG(motionProxy, "Motion Memory not found.");
  motionProxy->setFallManagerEnabled(false);
  kinematicsModule = 
    KinematicsModulePtr(new KinematicsModule<MType>(this));
  PRINT("Initialized kinematics module.")
  fallDetector = 
    FallDetectorPtr(new FallDetector<MType>(this));
  PRINT("Initialized body pose finder module.")
  trajectoryPlanner = 
    TrajectoryPlannerPtr(new TrajectoryPlanner<MType>(this));
  PRINT("Initialized trajectory planner.")
  pathPlanner = 
    PathPlannerSpace::PathPlannerPtr(
      new PathPlannerSpace::PathPlanner());
  pathPlanner->setMapPtr(
    boost::make_shared <PathPlannerSpace::GridMap2D>(
      IVAR(OccupancyMap, MotionModule::occupancyMap))
  );
  PRINT("Initialized path planner.")
  mbManager = boost::make_shared<MBManager<MType> >(this);
  PRINT("Initialized motion behaviors manager.")
  PRINT("Setting up static variables of MotionBehaviors.")
  //! Reset output variables
  OVAR(Matrix4f, MotionModule::upperCamInFeet) = Matrix4f::Identity();
  OVAR(Matrix4f, MotionModule::lowerCamInFeet) = Matrix4f::Identity();
  OVAR(Matrix4f, MotionModule::lFootOnGround) = Matrix4f::Identity();
  OVAR(Matrix4f, MotionModule::rFootOnGround) = Matrix4f::Identity();
  OVAR(PostureState, MotionModule::postureState) = PostureState::UNKNOWN;
  OVAR(bool, MotionModule::robotFallen) = false;
  OVAR(RobotPose2D<float>, MotionModule::moveTarget) = 
    RobotPose2D<float>(0.f, 0.f, 0.f);
  OVAR(Point2f, MotionModule::kickTarget) = Point2f(0.f, 0.f);
  OVAR(vector<RotatedRect>, MotionModule::footRects) = 
    vector<RotatedRect>();
  OVAR(bool, MotionModule::robotInMotion) = false;
}

void MotionModule::handleRequests()
{
  //PRINT("MotionModule.handleRequests()")
  static bool once = true;
  if (!once) {
    //Point2f ball(0.165, -0.05);
    //auto kConfig =
    //  boost::make_shared < MBKickConfig > (MBKickTypes::FIXED_VELOCITY, ball);
    //float mag = 0.5;
    //kConfig->reqVel.x = 0.5 * cos(M_PI / 4);
    //kConfig->reqVel.y = 0.5 * sin(M_PI / 4);

    //auto config = 
      //boost::make_shared<MBPostureConfig>(PostureState::CROUCH, 2.f);
    //auto btConfig = 
    //  boost::make_shared<MBKFMDiveConfig>(KeyFrameDiveTypes::LEFT);
    //auto config = 
    //  boost::make_shared<MPComControlConfig>(CHAIN_R_LEG, 2.f);
    //auto balanceZmpRefGen = boost::make_shared<BalanceZmpRefGen>(this, CHAIN_L_LEG, 100, 1.f, Vector2f(0.025f, 0.f));
    //balanceZmpRefGen->initiate();
    //auto zmpCtrlConfig = 
    //  boost::make_shared<ZmpControlConfig>(balanceZmpRefGen);
    //auto getupConfig =
    //  boost::make_shared<KFMGetupConfig>(KeyFrameGetupTypes::FRONT);

    /*Point2f ball(0.165, -0.05);
    auto config =
      boost::make_shared <JSOImpKickConfig> (
        ball,
        boost::make_shared<MBPostureConfig>(PostureState::CROUCH, 1.f),
        boost::make_shared<MPComControlConfig>(CHAIN_L_LEG, 0.8f)
      );
    config->target.x = 1.f;
    config->target.y = -0.05f;*/
    /*auto config =
      boost::make_shared<MBPostureConfig>(PostureState::CROUCH, 2.f);
    MotionRequestPtr motionRequest = 
      boost::make_shared<RequestMotionBehavior>(config);*/
    //inRequests.pushToQueue(motionRequest);
    once = true;
  }
  if (inRequests.isEmpty())
    return;
  auto request = inRequests.queueFront();
  if (boost::static_pointer_cast <MotionRequest>(request)) {
    auto reqId = request->getId();
    if (reqId == (unsigned)MotionRequestIds::BEHAVIOR_REQUEST) {
      auto rmb = 
        boost::static_pointer_cast<RequestMotionBehavior>(request);
      mbManager->manageRequest(rmb);
    } else if (reqId == (unsigned)MotionRequestIds::KILL_BEHAVIOR) {
      mbManager->killBehavior();
    }
  }
  inRequests.popQueue();
}

void MotionModule::mainRoutine()
{
  PRINT("MotionModule.mainRoutine()")
  //auto tStart = high_resolution_clock::now();
  kinematicsModule->update();
  //fallDetector->update();
  //pathPlanner->updateMap();
  //mbManager->update();
  //OVAR(BehaviorInfo, MotionModule::mBehaviorInfo) =
  //  mbManager->getBehaviorInfo();
  //duration<double> timeSpan = high_resolution_clock::now() - tStart;
  //PRINT("Time: " << timeSpan.count() << "seconds.");
}

KinematicsModulePtr MotionModule::getKinematicsModule() 
{ 
  return kinematicsModule; 
}

TrajectoryPlannerPtr MotionModule::getTrajectoryPlanner() 
{ 
  return trajectoryPlanner; 
}

ALMotionProxyPtr MotionModule::getSharedMotionProxy() 
{
  return motionProxy; 
}

PathPlannerSpace::PathPlannerPtr MotionModule::getPathPlanner() 
{ 
  return pathPlanner; 
}
