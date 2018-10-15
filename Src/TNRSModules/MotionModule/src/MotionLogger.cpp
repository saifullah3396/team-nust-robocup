/**
 * @file MotionModule/src/MotionLogger.cpp
 *
 * This file implements the class MotionLogger
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 02 Aug 2018
 */

#include "MotionModule/include/MTypeHeader.h"
#include "MotionModule/include/MotionLogger.h"
#include "MotionModule/include/KinematicsModule/KinematicsConsts.h"
#include "Utils/include/DebugUtils.h"
#include "Utils/include/HardwareIds.h"

void MotionLogger::recordJointCmds(
  const AL::ALValue& time, const AL::ALValue& cmds, const vector<unsigned>& ids)
{
  if (!logJointCmds)
    return;
  ASSERT(time.getSize() != 0);
  ASSERT(cmds.getSize() != 0);
  high_resolution_clock::time_point timeNow = high_resolution_clock::now();
  double timeStart = (duration<double>(timeNow - refTime)).count();  
  vector<int> jointIndices(NUM_JOINTS, -1);
  for (size_t i = 0; i < ids.size(); ++i) {
    jointIndices[ids[i]] = i;
  }
  for (size_t i = 0; i < time[0].getSize(); ++i) {
    JSON_APPEND(
      root["jointCommands"], 
      "time", 
      double(time[0][i]) + timeStart
    );
    for (size_t j = 0; j < jointIndices.size(); ++j) {
      if (jointIndices[j] < 0) {
        JSON_APPEND(
          root["jointCommands"], 
          jointNameConsts[j] + "Cmd", 
          "NAN"
        );
      } else {
        JSON_APPEND(
          root["jointCommands"], 
          jointNameConsts[j] + "Cmd", 
          (MType)cmds[jointIndices[j]][i]
        );
      }
    }
  }
}

template <typename Scalar>
void MotionLogger::recordJointStates(const vector<Scalar>& joints)
{
  if (!logJointStates)
    return;
  ASSERT(joints.size() == NUM_JOINTS);
  high_resolution_clock::time_point timeNow = high_resolution_clock::now();
  double time = (duration<double>(timeNow - refTime)).count();
  Json::Value jointStates; // Positions only
  JSON_APPEND(
    jointStates, 
    "time", 
    time
  );
  for (size_t i = 0; i < NUM_JOINTS; ++i) {
    JSON_APPEND(
      jointStates, 
      jointNameConsts[i] + "State",
      joints[i]
    );
    JSON_ASSIGN(root, "jointStates", jointStates);
  }
}  

template void MotionLogger::recordJointStates<MType>(const vector<MType>&);
