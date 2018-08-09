/**
 * @file MotionModule/BalanceModule/ZmpControl.cpp
 *
 * This file implements the class ZmpControl
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#include "MotionModule/include/BalanceModule/Types/ZmpControl.h"
#include "MotionModule/include/BalanceModule/ZmpPreviewController.h"
#include "MotionModule/include/BalanceModule/ZmpRef.h"
#include "ConfigManager/include/ConfigManager.h"

unsigned ZmpControl::nPreviews;
float ZmpControl::comHeight;

ZmpControlConfigPtr ZmpControl::getBehaviorCast()
{
  return boost::static_pointer_cast <ZmpControlConfig> (config);
}

void
ZmpControl::initiate()
{
  PRINT("ZmpControl.initiate()")
  #ifdef DEBUG_BUILD
  comLog.open(
	(ConfigManager::getLogsDirPath() + string("ZmpControl/Com.txt")).c_str(), 
	std::ofstream::out | std::ofstream::trunc
  );
  comLog.close();
  #endif
  refGenerator = getBehaviorCast()->refGenerator.get();
  for (int i = 0; i < controllers.size(); ++i) {
    controllers[i] = new ZmpPreviewController();
    controllers[i]->setComHeight(comHeight); 
    controllers[i]->setSamplingTime(cycleTime);
    controllers[i]->setPreviewLength(nPreviews);
    controllers[i]->initController();
  }
  inBehavior = true;
}

void
ZmpControl::update()
{
  PRINT("ZmpControl.update()")
  auto desComState = computeNextComState();
  /**
   * Implement wholebody com ik for this step here
   */ 
}

void
ZmpControl::finish()
{
  motionProxy->killAll();
  inBehavior = false;
}

void
ZmpControl::loadExternalConfig()
{
  GET_CONFIG("MotionBehaviors", 
    (unsigned, ZmpControl.nPreviews, nPreviews),
    (float, ZmpControl.comHeight, comHeight),
  )
}

ComState
ZmpControl::computeNextComState()
{
  refGenerator->update();
  zmpRef = refGenerator->getCurrentRef();
  try {
    if (zmpRef.length != nPreviews) 
    {
      throw 
      BehaviorException(
        this,
        "Size of zmp references do not match zmp controller preview length",
        true,
        EXC_INVALID_BEHAVIOR_SETUP
      );
    } 
  } catch (BehaviorException& e) {
    cout << e.what();
    inBehavior = false;
  }
  //cout << "refGenerator->getRefFrame(): " <<refGenerator->getRefFrame() << endl;
  auto comState = kM->getComState(refGenerator->getRefFrame());
  //cout << "com.position: " << comState.position.transpose() << endl;
  //cout << "com.velocity: " << comState.velocity.transpose() << endl;
  //cout << "com.accel: " << comState.accel.transpose() << endl;
  for (int i = 0; i < controllers.size(); ++i) {
    auto com1D = // X and Y for x-y dimension controllers
      Vector3f(comState.position[i], comState.velocity[i], comState.accel[i]);
    auto nextState = controllers[i]->step(com1D, zmpRef.xy[i]);
    comState.position[i] = nextState[0];
    comState.velocity[i] = nextState[1];
    comState.accel[i] = nextState[2];
  }
  comState.position[2] = comHeight; // fixed for z 
  comState.velocity[2] = 0.f; // desired velocity of zero for z axis
  comState.accel[2] = 0.f; // desired acceleration of zero for z axis
  return comState;
}

 /*
  * Good for usage of center of mass ik for generation of complete com trajectory from zmp controller updates
  * void
ZmpControl::balanceZmp()
{
 PRINT("Balancing based on Zmp planner")
  kM->getComWrtBase(KinematicsModule::ACTUAL, supportLeg, ANKLE, comPosition);
  comInitPosition = comPosition;
  comPositionLast = comPosition;
  updatedCom.resize(6, 1);
  updatedCom.setZero();
  comVelocity.setZero();
  comVelocityLast.setZero();
  comAcceleration.setZero();
  xRef = 0.025f;
  yRef = 0.0f;
  controller = boost::shared_ptr < ZmpController > (new ZmpController());
  initZmpController();
  AL::ALValue jointTimes;
  AL::ALValue jointPositions;
  jointTimes.clear();
  jointPositions.clear();
  jointTimes.arraySetSize(NUM_JOINTS);
  jointPositions.arraySetSize(NUM_JOINTS);
  int balanceReachedCnt = 0;
  runTime = 0;
  vector<unsigned> jointIds;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    jointIds.push_back(i);
  }
  kM->setJointPositions(
    KinematicsModule::SIM,
    0,
    kM->getJointPositions(KinematicsModule::ACTUAL, 0, NUM_JOINTS));
  Vector2f test;
  kM->getComWrtBase(KinematicsModule::SIM, supportLeg, FEET_BASE, comPosition);
  cout << "comPostiion: " << comPosition << endl;
  comPositionLast = comInitPosition;
  while (true) {
    high_resolution_clock::time_point tStart = high_resolution_clock::now();
    VectorXf zmpPreviewedX;
    VectorXf zmpPreviewedY;
    zmpPreviewedX.resize(nPreviews + 1);
    zmpPreviewedY.resize(nPreviews + 1);
    cout << "cycleTime: " << cycleTime << endl;
    cout << "runTime: " << runTime << endl;
    for (unsigned i = 0; i < nPreviews + 1; ++i) {
      if (runTime + i * cycleTime >= 1.f) {
        zmpPreviewedX[i] = xRef;
        zmpPreviewedY[i] = yRef;
      } else {
        zmpPreviewedX[i] = comInitPosition[0];
        zmpPreviewedY[i] = comInitPosition[1];
      }
    }
    updatedCom = controller->step(
      comPositionLast,
      comVelocity,
      Vector2f(comAcceleration.segment(0, 2)),
      zmpPreviewedX,
      zmpPreviewedY);
    if (
    //abs(updatedCom[0] - xRef) < 1e-6 &&
    abs(updatedCom[3] - yRef) < 1e-5 &&
    //abs(updatedCom[1]) < 1e-6 &&
    abs(updatedCom[4]) < 1e-5 &&
    //abs(updatedCom[2]) < 1e-6 &&
    abs(updatedCom[5]) < 1e-5) {
      break;
    }
    Vector2f error;
    error[0] = updatedCom[0] - comPosition[0];
    error[1] = updatedCom[3] - comPosition[1];
    float rollError = -0.01 * M_PI / 180;
    Matrix<float, 6, 1> comVelocityD = Matrix<float, 6, 1>::Zero();
    comVelocityD[0] = 0.0; //(error[0] * 1.f + intError[0] * 0.001) / cycleTime; //PI control
    comVelocityD[1] = updatedCom[4]; //(error[1] * 0.0015f + intError[1] * 0.001) / cycleTime; //PI control
    //comVelocityD[4] = rollError;
    intError = intError + error;
    vector<unsigned> limbMotionSpace;
    limbMotionSpace.push_back(0);
    limbMotionSpace.push_back(0);
    limbMotionSpace.push_back(0);
    limbMotionSpace.push_back(1);
    limbMotionSpace.push_back(1);
    vector<VectorXf> limbVelocitiesD;
    limbVelocitiesD.push_back(Matrix<float, 2, 1>::Zero());
    limbVelocitiesD.push_back(Matrix<float, 5, 1>::Zero());
    limbVelocitiesD.push_back(Matrix<float, 5, 1>::Zero());
    limbVelocitiesD.push_back(Matrix<float, 6, 1>::Zero());
    limbVelocitiesD.push_back(Matrix<float, 6, 1>::Zero());
    //limbVelocitiesD[4][2] = 0.001;
    vector<int> eeIndices;
    eeIndices.push_back(0);
    eeIndices.push_back(0);
    eeIndices.push_back(0);
    eeIndices.push_back(FEET_BASE);
    eeIndices.push_back(FEET_BASE);
    high_resolution_clock::time_point tEnd1 = high_resolution_clock::now();
    duration<double> time_span1 = tEnd1 - tStart;
    PRINT(
      "RobotExtraction time1: " + DataUtils::varToString(time_span1.count()) + " seconds.");
    VectorXf jointsD = kM->solveComIK(
      KinematicsModule::SIM,
      supportLeg,
      comVelocityD,
      limbMotionSpace,
      limbVelocitiesD,
      eeIndices);
    high_resolution_clock::time_point tEnd2 = high_resolution_clock::now();
    duration<double> time_span2 = tEnd2 - tEnd1;
    PRINT(
      "RobotExtraction time2: " + DataUtils::varToString(time_span2.count()) + " seconds.");
    runTime = runTime + cycleTime;
    for (int i = 0; i < NUM_JOINTS; ++i) {
      jointPositions[i].arrayPush(jointsD[i]);
      jointTimes[i].arrayPush(runTime);
    }
    //cout << jointPositions << endl;
    //cout << jointTimes << endl;
    kM->setJointPositions(KinematicsModule::SIM, 0, jointsD);
    //comPositionLast = comPosition;
    //comVelocityLast = comVelocity;
    #ifdef DEBUG_BUILD
    comLog.open(
    (ConfigManager::getLogsDirPath() + string("ZmpControl/Com.txt")).c_str(), 
    std::ofstream::out | std::ofstream::trunc
    );
    comLog << comPosition[0] << "    " << comPosition[1] << "\n";
    comLog.close();
    #endif
    kM->getComWrtBase(
      KinematicsModule::SIM,
      supportLeg,
      FEET_BASE,
      comPosition);
    //comVelocity[0] = (comPosition[0] - comPositionLast[0]) / cycleTime;
    //comVelocity[1] = (comPosition[1] - comPositionLast[1]) / cycleTime;
    //comAcceleration[0] = (comVelocity[0] - comVelocityLast[0]) / cycleTime;
    //comAcceleration[1] = (comVelocity[1] - comVelocityLast[1]) / cycleTime;
    comPositionLast[0] = updatedCom[0];
    comPositionLast[1] = updatedCom[3];
    comVelocity[0] = updatedCom[1];
    comVelocity[1] = updatedCom[4];
    comAcceleration[0] = updatedCom[2];
    comAcceleration[1] = updatedCom[5];
    high_resolution_clock::time_point tEnd3 = high_resolution_clock::now();
    duration<double> time_span3 = tEnd3 - tEnd2;
    PRINT(
      "RobotExtraction time3: " + DataUtils::varToString(time_span3.count()) + " seconds.");
  }
  timeToReachB = runTime;
  runTime = 0.f;
  cout << "timeToReachB: " << timeToReachB << endl;
  naoqiJointInterpolation(jointIds, jointTimes, jointPositions, false);
  inBehavior = true;
}
*/
