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
#include "MotionModule/include/MTypeHeader.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"
#include "MotionModule/include/PostureModule/PostureDefinitions.h"
#include "TNRSBase/include/MemoryBase.h"
#include "Utils/include/PostureState.h"

using namespace Utils;

#define BUFFER_SIZE 15

class MotionModule;

/**
 * @class FallDetector
 * @brief A class for determining whether the robot is fallen
 */
template <typename Scalar>
class FallDetector : public MemoryBase
{
public:
  /**
   * Constructor
   *
   * @param motionModule: pointer to parent.
   */
  FallDetector(MotionModule* motionModule);

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
  boost::circular_buffer<Matrix<Scalar, 3, 1> > torsoAccBuffer;

  //! Kinematics module object.
  boost::shared_ptr<KinematicsModule<Scalar> > kM;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<FallDetector<MType> > FallDetectorPtr;
