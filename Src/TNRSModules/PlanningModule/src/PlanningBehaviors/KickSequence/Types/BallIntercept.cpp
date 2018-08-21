/**
 * @file PlanningBehaviors/KickSequence/Types/BallIntercept.cpp
 *
 * This file implements the class BallIntercept.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 16 Nov 2017
 */

#include "MotionModule/include/MotionConfigs/MBPostureConfig.h"
#include "MotionModule/include/MotionConfigs/MBBalanceConfig.h"
#include "MotionModule/include/MotionConfigs/MBKickConfig.h"
#include "MotionModule/include/MotionConfigs/MBHeadControlConfig.h"
#include "SBModule/include/SBConfigs.h"
#include "PlanningModule/include/PlanningBehaviors/KickSequence/Types/BallIntercept.h"
#include "PlanningModule/include/PlanningBehaviors/KickSequence/MotionEquationSolver.h"
#include "VisionModule/include/VisionRequest.h"
#include "Utils/include/VisionUtils.h"
#include <opencv2/core/eigen.hpp>

BallInterceptConfigPtr BallIntercept::getBehaviorCast()
{
  return boost::static_pointer_cast <BallInterceptConfig> (config);
}

void
BallIntercept::initiate()
{
  PRINT("BallIntercept.initiate()...")
  inBehavior = true;
}

void
BallIntercept::update()
{
  //PRINT("BallIntercept.update()...")
	if (requestInProgress()) return;
  updatePostureAndStiffness();
  if (behaviorState == startup) {
    startupAction();
  } else if (behaviorState == ballIncoming) {
    ballIncomingAction();
  }
}

void BallIntercept::finish()
{
  inBehavior = false;
}

void BallIntercept::startupAction()
{
  PRINT("BallIntercept.startupAction()...")
  if (posture == PostureState::STAND && 
      stiffness == StiffnessState::ROBOCUP) 
  {
    //auto vRequest = boost::make_shared<SwitchVision>(true);
    //BaseModule::publishModuleRequest(vRequest);
    behaviorState = ballIncoming;
  } else if (stiffness != StiffnessState::ROBOCUP) {
    if (!sbInProgress()) {
      PRINT("Setting stiffness high at startup.")
      auto sConfig =
        boost::make_shared <SBStiffnessConfig> (
          StiffnessState::ROBOCUP);
      setupSBRequest(sConfig);
    }
  } else if (posture != PostureState::STAND) {
    if (!mbInProgress()) {
      PRINT("Setting posture to stand at startup.")
      auto pConfig = 
        boost::make_shared<MBPostureConfig>(
          PostureState::STAND, 2.f);
      setupMBRequest(pConfig);
    }
  }
}

void BallIntercept::ballIncomingAction()
{
  //PRINT("BallIntercept.ballIncomingAction()...")
  static bool once = false;
  if (!once) {
    if (!mbInProgress()) {
      auto kConfig =
        boost::make_shared <JSOImpKickConfig> (
          Point2f(0.15, -0.05), 
          boost::make_shared<MPComControlConfig>(CHAIN_L_LEG, 1.f),
          1.f
        );
      kConfig->target = Point2f(1.f, -0.05f);
      setupMBRequest(kConfig);
    }
    once = true;
  }
  return;
  //PRINT("BallIntercept.waitForBallAction()...") 
  auto& bInfo = IVAR(BallInfo, PlanningModule::ballInfo);
  auto targetRight = Vector2f(0.15f, -0.05f);
  auto targetLeft = Vector2f(0.15f, -0.05f);
  double damping = 0.16;
  cout << "bInfo.posRel: " << bInfo.posRel << endl;
  cout << "bInfo.velRel: " << bInfo.velRel << endl;
  auto dmeSolver = DampedMESolver(
    targetRight, 
    Vector2f (bInfo.posRel.x, bInfo.posRel.y), 
    Vector2f (bInfo.velRel.x, bInfo.velRel.y), 
    damping
  );
  dmeSolver.optDef();
  return;
  if (true) { //bInfo.found) {
    // A rectangle with kick area of right foot
    vector<Vec4f> rectLines;
    rectLines.push_back(Vec4f(0.180, -0.065, 0.180,  0.065)); // front line
    rectLines.push_back(Vec4f(0.180,  0.065, 0.130,  0.065)); // left line
    rectLines.push_back(Vec4f(0.130,  0.065, 0.130, -0.065)); // back line
    rectLines.push_back(Vec4f(0.130, -0.065, 0.180, -0.065)); // right line
		auto pos = Point2f(2.f, -0.05f); // bInfo.posRel;
		auto vel = Point2f(-1.0, 0.f); // bInfo.velRel;
		auto acc = Point2f(0.022*9.81, 0.f);// bInfo.accRel;
		auto velMag = norm(vel);
    if (velMag >= 0.02f) {
      Point2f unitVel;
      unitVel.x = vel.x / velMag;
      unitVel.y = vel.y / velMag;
      
      Vec4f ballLine;
      ballLine[0] = pos.x;
      ballLine[1] = pos.y;
      ballLine[2] = pos.x + unitVel.x * 100.f; // A possibly infinite line
      ballLine[3] = pos.y + unitVel.y * 100.f; // A possibly infinite line
      
      Point2f inter;
      for (size_t i = 0; i < rectLines.size(); ++i) {
        inter = VisionUtils::findIntersection(rectLines[i], ballLine);
        if (inter.x != -100)
          break;
      }
      if (inter.x != -100) {
        // if intersection point is found, that means ball will end up in
        // the robot's kick area
        // Finding time now until it reaches the box
        auto accMag = norm(acc);
        Point2f unitAcc;
        unitAcc.x = acc.x / accMag;
        unitAcc.y = acc.y / accMag;
        cout << "unitAcc.x / unitVel.x: " << unitAcc.x / unitVel.x << endl;
        if (unitAcc.x / unitVel.x < 0) {
          // acc is opposite in direction to velocity
          // Find the time ball will reach the line 15 cm in front of robot. 
          // Use s = vit + 1/2at^2
          auto dist = norm(inter - pos);
          cout << "dist: " << dist << endl;
          float a, b, c; // for solving the quadratic equation
          a = -0.5f * accMag; // cuz the acc is opposite in direction to velocity
          b = velMag;
          c = -dist;
          float sqrtC = b * b - 4 * a * c;
          if (sqrtC > 0) { // Ball won't stop before reaching the goal
            PRINT("Ball won't stop before reaching the goal...")
            unsigned supportLeg = inter.y > 0.f ? CHAIN_R_LEG : CHAIN_L_LEG;
            if (supportLeg == CHAIN_L_LEG)
              cout << "sending balance to left leg" << endl;
            else
              cout << "sending balance to right leg" << endl;
              
            float timeToStop = velMag / accMag;
            cout << "timeToStop: " << timeToStop << endl;  
            float timeToReach = (-b + sqrt(sqrtC)) / (2 * a);
            cout << "timeToReach1: " << timeToReach << endl;  
            if (timeToReach > timeToStop) {
              timeToReach = (-b - sqrt(sqrtC)) / (2 * a);
            }
            cout << "timeToReach2: " << timeToReach << endl;  
              
            /*static bool comShifted = false;
            if (!comShifted) {
              if (!mbInProgress()) {
                auto bConfig = 
                  boost::make_shared<MPComControlConfig>(supportLeg, 2.f);
                setupMBRequest(bConfig);
              }
              comShifted = true;
            }*/
          } else {
            PRINT("Ball too slow for intercept")
          }
        }
      }
    }
	}
}
