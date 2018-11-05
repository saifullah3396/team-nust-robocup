/**

 * @file MotionModule/BalanceModule/ZmpControl.cpp
 *
 * This file implements the class ZmpControl
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#include "MotionModule/include/BalanceModule/BalanceDefinitions.h"
#include "MotionModule/include/BalanceModule/Types/ZmpControl.h"
#include "MotionModule/include/BalanceModule/ZmpPreviewController.h"
#include "MotionModule/include/BalanceModule/ZmpRef.h"
#include "MotionModule/include/BalanceModule/BalanceZmpRefGen.h"
#include "MotionModule/include/KinematicsModule/ComState.h"
#include "MotionModule/include/KinematicsModule/Joint.h"
#include "MotionModule/include/KinematicsModule/MotionTask.h"
#include "MotionModule/include/JointRequest.h"
#include "Utils/include/ConfigMacros.h"

template<typename Scalar>
unsigned ZmpControl<Scalar>::nPreviews;
template<typename Scalar>
Scalar ZmpControl<Scalar>::comHeight;

template<typename Scalar>
ZmpControl<Scalar>::ZmpControl(
  MotionModule* motionModule,
  const BehaviorConfigPtr& config) :
  BalanceModule<Scalar>(motionModule, config, "ZmpControl"),
  execTime(Scalar(0.0))
{
  controllers.resize(2); // x-y dimensions
}

template<typename Scalar>
ZmpControl<Scalar>::~ZmpControl()
{
  for (size_t i = 0; i < controllers.size(); ++i)
    delete controllers[i];
}

template<typename Scalar>
ZmpControlConfigPtr ZmpControl<Scalar>::getBehaviorCast()
{
  return boost::static_pointer_cast <ZmpControlConfig> (this->config);
}

template<typename Scalar>
void
ZmpControl<Scalar>::initiate()
{
  LOG_INFO("ZmpControl.initiate()");
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
        Matrix<Scalar, 2, 1>(0.022, 0.0)
    );
  refGenerator->initiate();
  auto comState = this->kM->getComStateWrtFrame(refGenerator->getRefFrame(), FEET_BASE);
  for (int i = 0; i < controllers.size(); ++i) {
    controllers[i] = new ZmpPreviewController<Scalar>(this->kM->getComModel(i));
    controllers[i]->setPreviewLength(nPreviews);
    controllers[i]->initController();
  }
  unsigned supportLeg = getBehaviorCast()->supportLeg;
  unsigned otherLeg = getBehaviorCast()->supportLeg == CHAIN_L_LEG ? CHAIN_R_LEG : CHAIN_L_LEG;
  vector<bool> activeJoints(NUM_JOINTS, false);

  // use all support leg joints for ik
  for (int i = 1; i < this->kM->getLinkChain(supportLeg)->size; ++i)
    activeJoints[this->kM->getLinkChain(supportLeg)->start + i] = true;

  // use all other leg joints for ik
  for (int i = 1; i < this->kM->getLinkChain(otherLeg)->size; ++i)
    activeJoints[this->kM->getLinkChain(otherLeg)->start + i] = true;

  tasks.resize(NUM_TASKS);
  // make a center of mass task to control its desired trajectory
  tasks[COM_TASK] = this->kM->makeComTask(CHAIN_L_LEG, comState.position, activeJoints, 1e-2, 0.9);

  if (getBehaviorCast()->keepOtherLegContact) {
    // keep the other leg on ground contact with high priority
    tasks[CONTACT_TASK] = this->kM->makeContactTask(otherLeg, FEET_BASE, activeJoints, 10, 0.85);
  }

  if (getBehaviorCast()->regularizePosture) {
    // Use predefined joints as target for end posture after balance
    // This is necessary to get a good posture
    Matrix<Scalar, Dynamic, 1> targetJoints(NUM_JOINTS);
    if (getBehaviorCast()->supportLeg == CHAIN_L_LEG) {
      targetJoints = Matrix<Scalar, Dynamic, 1>::Map(
        &balanceDefs[0][0],
        sizeof(balanceDefs[0]) / sizeof(balanceDefs[0][0]));
    } else {
      targetJoints = Matrix<Scalar, Dynamic, 1>::Map(
        &balanceDefs[1][0],
        sizeof(balanceDefs[1]) / sizeof(balanceDefs[1][0]));
    }
    // Add a posture task to be reached during balance
    tasks[POSTURE_TASK] = this->kM->makePostureTask(targetJoints, activeJoints, 1e-5, 0.85);
  }

  if (getBehaviorCast()->keepTorsoUpright) {
    // Torso task to keep torso orientation to the initial orientation
    // This can be useful in some cases
    Matrix<Scalar, 4, 4> target = MathsUtils::getTInverse(this->kM->getForwardEffector(supportLeg, FEET_BASE));
    activeJoints = vector<bool>(NUM_JOINTS, false);
    for (int i = 1; i < this->kM->getLinkChain(supportLeg)->size; ++i)
      activeJoints[this->kM->getLinkChain(supportLeg)->start + i] = true;
    tasks[TORSO_TASK] =  this->kM->makeTorsoTask(supportLeg, FEET_BASE, target, activeJoints, 1e-3, 0.85);
  }
  this->inBehavior = true;
}

template <typename Scalar>
void ZmpControl<Scalar>::reinitiate(const BehaviorConfigPtr& cfg)
{
  this->config = cfg;
  if (!getBehaviorCast()->keepOtherLegContact)
    tasks[CONTACT_TASK].reset();
  if (!getBehaviorCast()->regularizePosture)
    tasks[POSTURE_TASK].reset();
  if (!getBehaviorCast()->keepTorsoUpright)
    tasks[TORSO_TASK].reset();
}

template<typename Scalar>
void ZmpControl<Scalar>::update()
{
  LOG_INFO("Tracking...")
  trackZmp();
  LOG_INFO("ZmpControl.update()");
  //if (execTime > (Scalar)1.0) {
  //  finish();
  //} else {
    //trackZmp();
  //  execTime += (Scalar)this->cycleTime;
  //}
}

template<typename Scalar>
void
ZmpControl<Scalar>::finish()
{
  this->killAllMotions();
  this->inBehavior = false;
}

template<typename Scalar>
void
ZmpControl<Scalar>::loadExternalConfig()
{
  static bool loaded = false;
  if (!loaded) {
    GET_CONFIG(
      "MotionBehaviors",
      (unsigned, ZmpControl.nPreviews, nPreviews),
    )
    loaded = true;
  }
}

template<typename Scalar>
void ZmpControl<Scalar>::trackZmp()
{
  refGenerator->update();
  zmpRef = refGenerator->getCurrentRef();
  ASSERT(zmpRef.length == nPreviews)
  zmpRegLog.open(
    (ConfigManager::getLogsDirPath() + string("ZmpControl/ZmpRef.txt")).c_str(),
    std::ofstream::out | std::ofstream::app
  );
  zmpRegLog << zmpRef.xy[0][0] << " " << zmpRef.xy[1][0] << endl;
  zmpRegLog.close();
  Matrix<Scalar, 3, 1> desComPosition;
  for (int i = 0; i < controllers.size(); ++i) {
    desComPosition[i] = controllers[i]->step(zmpRef.xy[i])[0];
  }
  auto comState = this->kM->getComStateWrtFrame(refGenerator->getRefFrame(), FEET_BASE);
  desComPosition[2] = comState.position[2];
  comLog.open(
    (ConfigManager::getLogsDirPath() + string("ZmpControl/Com.txt")).c_str(),
    std::ofstream::out | std::ofstream::app
  );
  comLog << this->motionModule->getModuleTime() << "  " << comState.position[0] << " " << comState.position[1] << " " << desComPosition[0] << " " << desComPosition[1] << " " << comState.zmp[0] << " " << comState.zmp[1] << endl;
  comLog.close();
  boost::static_pointer_cast<ComTask<Scalar> >(tasks[COM_TASK])->setTargetCom(desComPosition);
  for (size_t i = 0; i < tasks.size(); ++i) {
    if (tasks[i])
      this->addMotionTask(tasks[i]);
  }
}

template class ZmpControl<MType>;
