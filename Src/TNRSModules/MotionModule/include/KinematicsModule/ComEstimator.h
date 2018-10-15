/**
 * @file MotionModule/include/KinematicsModule/ComEstimator.h
 *
 * This file declares the class ComEstimator
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Jan 2018
 */

#include "Utils/include/DataUtils.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/VisionUtils.h"

/**
 * @class ComEstimator
 * @brief A class that provides a definition for center of mass state
 *   estimator
 */
class ComEstimator
{
public:
  /**
   * Default constructor for this class.
   */
  ComEstimator() :
    stateSize(6), measSize(4), contrSize(0), initiated(false)
  {
  }

  /**
   * Default destructor for this class.
   */
  ~ComEstimator()
  {
  }
 
  /**
   * Sets up the filter and initiates it with the given state vector
   * 
   * @param initState: The initial state for the filter
   * @param comHeight: Com height for the inverted-cart table model
   * @param dT: Time step for the filter
   */ 
  void init(const Mat& initState, const double& comHeight, const double& dT);
  
  /**
   * Resets the filter with the given state vector
   * 
   * @param state: The reset state for the filter
   */ 
  void reset(const Mat& state);  
  
  /**
   * Wrapper function for opencv KalmanFilter::predict()
   * 
   * @return returns the predicted state
   */ 
  Mat predict();

  /**
   * Returns the estimated state
   * 
   * @return Mat
   */ 
  Mat getEstimatedState();

  /**
   * Performs the correction step on the kalman filter
   * 
   * @param meas: The input measurement vector
   */ 
  void correct(const Mat& meas, const double& comHeight);

  /**
   * Sets up the opencv kalman filter 
   */ 
  void setupKalmanFilter();
  
  /**
   * Returns true if the filter is already initiated
   */ 
  bool isInitiated() { return initiated; }
  
private:
  //! OpenCv based kalman filter object.
  KalmanFilter kFilter;

  //! Vector of state variables.
  //! [com_pos_x, com_pos_y, com_velocity_x, 
  //!  com_velocity_y, com_acc_x, com_acc_y]
  Mat state;
  
  //! State transition matrix for motion update for trajectory 
  //! extrapolation
  Mat stateTransition;
  
  //! Vector of measurement variables.
  //! [meas_com_pos_x, meas_com_pos_y, meas_zmp_x, meas_zmp_y]
  Mat meas;

  //! A matrix defining the actual noise covariance matrix for measurements
  Mat measNoiseCov;

  //! Number of state variables: 
  int stateSize;

  //! Number of measurement variables: 
  int measSize;

  //! Number of variables in the contr state vector. 
  int contrSize;
  
  //! Whether the filter has been initiated or not
  bool initiated;
};
