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
#include "MotionModule/include/BalanceModule/BalanceZmpRefGen.h"
#include "MotionModule/include/KinematicsModule/ComState.h"
#include "MotionModule/include/KinematicsModule/Joint.h"
#include "ControlModule/include/ActuatorRequests.h"
#include "Utils/include/ConfigMacros.h"

template<typename Scalar>
unsigned ZmpControl<Scalar>::nPreviews;
template<typename Scalar>
Scalar ZmpControl<Scalar>::comHeight;

template<typename Scalar>
ZmpControlConfigPtr ZmpControl<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <ZmpControlConfig> (this->config);
}

template<typename Scalar>
void
ZmpControl<Scalar>::initiate()
{
  PRINT("ZmpControl.initiate()");
  //#ifdef DEBUG_BUILD
  comLog.open(
    (ConfigManager::getLogsDirPath() + string("ZmpControl/Com.txt")).c_str(),
    std::ofstream::out | std::ofstream::trunc
  );
  comLog << "# X Y" << endl;
  comLog.close();
  zmpRegLog.open(
    (ConfigManager::getLogsDirPath() + string("ZmpControl/ZmpRef.txt")).c_str(),
    std::ofstream::out | std::ofstream::trunc
  );
  zmpRegLog << "# X Y" << endl;
  zmpRegLog.close();
  //#endif
  //refGenerator = getBehaviorCast()->refGenerator.get();
  refGenerator = new BalanceZmpRefGen<Scalar>(
        this->motionModule,
        getBehaviorCast()->supportLeg,
        nPreviews,
        (Scalar)1.0,
        Matrix<Scalar, 2, 1>(0.03, 0.0)
    );
  refGenerator->initiate();
  for (int i = 0; i < controllers.size(); ++i) {
    controllers[i] = new ZmpPreviewController<Scalar>();
    controllers[i]->setComHeight(comHeight); 
    controllers[i]->setSamplingTime(this->cycleTime);
    controllers[i]->setPreviewLength(nPreviews);
    controllers[i]->initController();
  }
  this->inBehavior = true;
}

template<typename Scalar>
void
ZmpControl<Scalar>::update()
{
  //PRINT("ZmpControl.update()");
  //if (execTime > (Scalar)1.0) {
  //  finish();
  //} else {
    trackZmp();
  //  execTime += (Scalar)this->cycleTime;
  //}
}

template<typename Scalar>
void
ZmpControl<Scalar>::finish()
{
  this->motionProxy->killAll();
  this->inBehavior = false;
}

template<typename Scalar>
void
ZmpControl<Scalar>::loadExternalConfig()
{
  static bool loaded = false;
  if (!loaded) {
    GET_CONFIG("MotionBehaviors", 
      (unsigned, ZmpControl.nPreviews, nPreviews),
      (Scalar, ZmpControl.comHeight, comHeight),
    )
    loaded = true;
  }
}

template<typename Scalar>
ComState<Scalar>
ZmpControl<Scalar>::computeNextComState()
{
  refGenerator->update();
  zmpRef = refGenerator->getCurrentRef();
  zmpRegLog.open(
    (ConfigManager::getLogsDirPath() + string("ZmpControl/ZmpRef.txt")).c_str(),
    std::ofstream::out | std::ofstream::app
  );
  zmpRegLog << zmpRef.xy[0][0] << " " << zmpRef.xy[1][0] << endl;
  zmpRegLog.close();
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
    this->inBehavior = false;
  }
  //cout << "refGenerator->getRefFrame(): " <<refGenerator->getRefFrame() << endl;
  static auto comState = this->kM->getComStateWrtFrame(refGenerator->getRefFrame(), FEET_BASE);
  ///cout << "comState: " << comState.position << endl;
  ///cout << "comState: " << comState.velocity << endl;
  ///cout << "comState: " << comState.accel << endl;
  ///cout << "zmpState: " << comState.position - comHeight / gConst * comState.accel << endl;
  static unsigned counter = 0;
  if (counter > 5) {
    comState = this->kM->getComStateWrtFrame(refGenerator->getRefFrame(), FEET_BASE);
    cout << "comState.position: " << comState.position.transpose() << endl;
    cout << "comState.vel: " << comState.velocity.transpose() << endl;
    cout << "comState.accel: " << comState.accel.transpose() << endl;
    counter = 0;
  }
  //counter++;
  for (int i = 0; i < controllers.size(); ++i) {
    auto com1D = // X and Y for x-y dimension controllers
      Matrix<Scalar, 3, 1>(comState.position[i], comState.velocity[i], comState.accel[i]);
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

template<typename Scalar>
void ZmpControl<Scalar>::trackZmp()
{
  AL::ALValue names;
  AL::ALValue jointTimes;
  AL::ALValue jointPositions;
  jointTimes.clear();
  jointPositions.clear();
  names.arraySetSize(NUM_JOINTS);
  jointTimes.arraySetSize(NUM_JOINTS);
  jointPositions.arraySetSize(NUM_JOINTS);
  vector<unsigned> jointIds;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    jointIds.push_back(i);
    names[i] = jointNameConsts[i];
  }
  auto desComState = computeNextComState();
  auto comState = this->kM->getComStateWrtFrame(refGenerator->getRefFrame(), FEET_BASE);
  auto zmp = this->kM->computeFsrZmp(getBehaviorCast()->supportLeg);
  comLog.open(
    (ConfigManager::getLogsDirPath() + string("ZmpControl/Com.txt")).c_str(),
    std::ofstream::out | std::ofstream::app
  );
  comLog << this->motionModule->getModuleTime() << "  " << comState.position[0] << " " << comState.position[1] << " " << desComState.position[0] << " " << desComState.position[1] << " " << zmp[0] << " " << zmp[1] << endl;
  comLog.close();
  //auto desVel = (desComState.position - comState.position) / this->cycleTime;
  Matrix<Scalar, 6, 1> comVelocityD = Matrix<Scalar, 6, 1>::Zero();
  comVelocityD[0] = desComState.velocity[0];// desVel[0];//desComState.velocity[0];//desVel[0];
  comVelocityD[1] = desComState.velocity[1];//desVel[1];// desComState.velocity[1];//desVel[1];
  vector<unsigned> limbMotionSpace;
  limbMotionSpace.push_back(0); // Joint space
  limbMotionSpace.push_back(0); // Joint space
  limbMotionSpace.push_back(0); // Joint space
  limbMotionSpace.push_back(1); // Cartesian space
  limbMotionSpace.push_back(1); // Cartesian space
  vector<Matrix<Scalar, Dynamic, 1>> limbVelocitiesD;
  limbVelocitiesD.push_back(Matrix<Scalar, 2, 1>::Zero());
  limbVelocitiesD.push_back(Matrix<Scalar, 5, 1>::Zero());
  limbVelocitiesD.push_back(Matrix<Scalar, 5, 1>::Zero());
  limbVelocitiesD.push_back(Matrix<Scalar, 6, 1>::Zero());
  limbVelocitiesD.push_back(Matrix<Scalar, 6, 1>::Zero());
  vector<int> eeIndices;
  eeIndices.push_back(0);
  eeIndices.push_back(0);
  eeIndices.push_back(0);
  eeIndices.push_back(FEET_BASE);
  eeIndices.push_back(FEET_BASE);
  Matrix<Scalar, Dynamic, 1> jointsD = 
    this->kM->solveComIK(
      getBehaviorCast()->supportLeg,
      comVelocityD,
      limbMotionSpace,
      limbVelocitiesD,
      eeIndices,
      JointStateType::ACTUAL
    );
  /*Matrix<Scalar, Dynamic, 1> jointsD = 
  this->kM->solveComIkTwoVar(
    desComState.position, 
    getBehaviorCast()->supportLeg, 
    FEET_BASE,
    JointStateType::ACTUAL
  );*/
  //cout << "jointsD: " << jointsD.transpose() << endl;
  auto jointRequest = boost::make_shared<JointRequest>();
  for (int i = 0; i < NUM_JOINTS; ++i) {
    jointRequest->setValue(jointsD[i], i);
    //jointPositions[i] = jointsD[i];
    //jointTimes[i] = this->cycleTime;
  }
  BaseModule::publishModuleRequest(jointRequest);
  //this->naoqiJointInterpolation(jointIds, jointTimes, jointPositions, true);
  //cout << this->kM->getComStateWrtFrame(getBehaviorCast()->supportLeg, FEET_BASE).position << endl;
  //this->motionProxy->setAngles(names, jointPositions, 0.5f);
}

template class ZmpControl<MType>;
