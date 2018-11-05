/**
 * @file SBModule/src/StaticBehavior.cpp
 *
 * This file implements the class StaticBehavior
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#include "SBModule/include/StaticBehavior.h"

void StaticBehavior::naoqiStiffnessInterpolation(
  const vector<unsigned>& ids,
  const AL::ALValue& timeLists, const AL::ALValue& stiffnessLists,
  const bool& postCommand)
{
  ASSERT(
    ids.size() == timeLists.getSize() && timeLists.getSize() == stiffnessLists.getSize());
  AL::ALValue names;
  names.clear();
  names.arraySetSize(ids.size());
  for (int i = 0; i < ids.size(); ++i) {
    names[i] = stiffnessJointNames[ids[i]];
  }
  try {
    if (postCommand) {
      motionProxy->post.stiffnessInterpolation(
        names,
        stiffnessLists,
        timeLists);
    } else {
      motionProxy->stiffnessInterpolation(names, stiffnessLists, timeLists);
    }
  } catch (exception &e) {
    ERROR(e.what())
  }
}
