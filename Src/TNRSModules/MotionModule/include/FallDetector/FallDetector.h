/**
 * @file MotionModule/include/FallDetector/FallDetector.h
 *
 * This file declares the class FallDetector
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 27 Sep 2017
 */

#pragma once

#include <boost/circular_buffer.hpp>
#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/PostureModule/PostureDefinitions.h"
#include "Utils/include/PostureState.h"

using namespace Utils;

/**
 * @class FallDetector
 * @brief A class for determining whether the robot is fallen
 */
class FallDetector : public MemoryBase
{
public:
  /**
   * Constructor
   *
   * @param motionModule: pointer to parent.
   */
  FallDetector(MotionModule* motionModule) :
    MemoryBase(motionModule), 
    bufferSize(15)
  {
    kM = motionModule->getKinematicsModule();
    torsoAccBuffer.set_capacity(bufferSize);
  }

  /**
   * Destructor
   */
  ~FallDetector()
  {
  }

  /**
   * Updates the fall state of the robot.
   */
  void
  update();

private:
  //! Size of the buffer torso imu sensor buffer
  size_t bufferSize;

  //! A circular buffer for averaging out the imu accelerations
  boost::circular_buffer<Vector3f> torsoAccBuffer;

  //! Kinematics module object.
  boost::shared_ptr<KinematicsModule> kM;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<FallDetector> FallDetectorPtr;
