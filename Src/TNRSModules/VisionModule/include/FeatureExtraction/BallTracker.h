/**
 * @file FeatureExtraction/BallTracker.h
 *
 * This file declares the class BallTracker
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Jan 2018
 */

#include "Utils/include/DataUtils.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/VisionUtils.h"
#include "VisionModule/include/CameraTransform.h"
#include "VisionModule/include/VisionModule.h"

/**
 * @class BallTracker
 * @brief A class that provides a definition of a ball tracker based on
 *   kalman filter.
 */
class BallTracker : public MemoryBase
{
public:
  /**
   * Default constructor for this class.
   * 
   * @param visionModule: Pointer to parent VisionModule.
   */
  BallTracker(VisionModule* visionModule) :
    MemoryBase(visionModule), visionModule(visionModule),
      cameraTransform(visionModule->getCameraTransform()),
      cycleTime(visionModule->getPeriodMinMS() / ((float) 1000)), stateSize(12),
      measSize(6), motionStateSize(6), contrSize(0), timeSinceLost(-1.f), timeAtLost(-1.f),
      losingBall(false)
  {
  }

  /**
   * Default destructor for this class.
   */
  ~BallTracker()
  {
  }

  void
  init(const unsigned& camIndex = 0);  
  
  void 
  reset(const unsigned& camIndex = 0);
  
  void 
  reset(const unsigned& camIndex, const Mat& state);
  
  Mat
  predict();

  Mat
  getEstimatedState();

  void
  updateFilter(vector<Rect> ballRect);

  void
  setupKalmanFilter();
  
  void 
  getBallPreview(
	vector<Mat>& ballPreview, 
	const unsigned& n, 
	const float& dt);
  
  bool
  getBallFound() const
  {
    return !lost;
  }

  float
  getTimeSinceLost() const
  {
    return timeSinceLost;
  }

  void
  setCamIndex(const unsigned& camIndex)
  {
    this->camIndex = camIndex;
  }

private:
  //! OpenCv based kalman filter object.
  KalmanFilter kFilter;

  //! Vector of state variables.
  Mat state;
  
  //! Vector of motion state variables.
  //! [posX, posY, velocityX, velocityY, accX, accY]
  Mat motionState;
  
  //! State transition matrix for motion update for trajectory 
  //! extrapolation
  Mat motionStateTransition;
  
  //! Vector of measurement variables.
  Mat meas;

  //! A matrix defining the actual noise covariance matrix for measurements
  Mat measNoiseCov;
  
  //! A matrix defining noise covariance matrix for when no measurement input is recieved
  Mat infMeasNoiseCov;

  //! Number of state variables: 
  //! [posX, posY, velocityX, velocityY, accX, accY,
  //!  posImageX, posImageY, velocityImageX, velocityImageY, 
  //!  wImage, hImage]
  int stateSize;

  //! Number of measurement variables: 
  //! [measPosX, measPosY, measPosImageX, measPosImageY, 
  //!  measWImage, measHImage]
  int measSize;
  
  //! Number of motion state variables
  int motionStateSize;

  //! Number of variables in the contr state vector. 
  int contrSize;

  //! Time at which the ball was lost.
  float timeAtLost;

  //! Time since the ball was lost.
  float timeSinceLost;

  //! Whether the filter is to be reinitialized.
  bool reinitialize;

  //! Whether the ball is lost.
  bool lost;

  //! Whether we are currently getting no ball observation.
  bool losingBall;

  //! Time step for the kalman filter updates.
  float cycleTime;

  //! Image transform object.
  boost::shared_ptr<CameraTransform> cameraTransform;

  //! Cam index for ball tracker information.
  unsigned camIndex;

  //! Vision module pointer object.
  VisionModule* visionModule;
};
