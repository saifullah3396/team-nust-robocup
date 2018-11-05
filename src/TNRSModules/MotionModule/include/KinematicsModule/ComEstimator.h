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
#include "Utils/include/Filters/KalmanFilter.h"
#include "Utils/include/EnvConsts.h"

/**
 * @class ComEstimator
 * @brief A class that provides a definition for center of mass state
 *   estimator
 */
template <typename Scalar>
class ComEstimator
{
public:
  /**
   * @brief ComEstimator Constructor
   */
  ComEstimator() :
    stateSize(3), measSize(3), inputSize(1), initiated(false)
  {
  }

  /**
   * @brief ~ComEstimator Destructor
   */
  ~ComEstimator()
  {
  }
 
  /**
   * @brief init Sets up the filter and initiates it with the given state vector
   * 
   * @param initState the initial state for the filter
   * @param comHeight com height for the inverted-cart table model
   * @param dT time step for the filter
   */
  void init(
    const Matrix<Scalar, Dynamic, 1>& initState,
    const Scalar& comHeight,
    const Scalar& dT);
  
  /**
   * @brief reset Resets the filter with the given state vector
   * 
   * @param state the reset state for the filter
   */ 
  void reset(const Matrix<Scalar, Dynamic, 1>& state) { model->setState(state); }
  
  /**
   * @brief update Updates the filter and estimator for the given
   *   measurement input
   *
   * @param meas The new measurement
   */ 
  void update(Matrix<Scalar, Dynamic, 1>& meas);

  /**
   * @brief getState Returns the current system state
   */ 
  Matrix<Scalar, Dynamic, 1> getState() { return model->getState(); }

  /**
   * @brief getOutput Returns the current system output
   */
  Matrix<Scalar, Dynamic, 1> getOutput() { return model->getOutput(); }
  
  /**
   * @brief getModel Returns the process model used
   */
  boost::shared_ptr<ProcessModel<Scalar> > getModel() { return model; }

  /**
   * @brief updateComHeight Updates the model for the given constant com height
   * @param comHeight Height of the center of mass
   */
  void updateComHeight(const Scalar& comHeight) {
    C(0, 2) = -comHeight / gConst;
    model->setOutputMatrix(C);
    H(2, 2) = -comHeight / gConst;
    filter->setMeasMatrix(H);
  }

  /**
   * Returns true if the filter is already initiated
   */ 
  bool isInitiated() { return initiated; }
  
private:
  //! Kalman filter
  boost::shared_ptr<KalmanFilter<Scalar> > filter;

  //! Com state transition model
  boost::shared_ptr<ProcessModel<Scalar> > model;

  //! Com state output matrix
  Matrix<Scalar, Dynamic, Dynamic> C;

  //! Com measurement matrix
  Matrix<Scalar, Dynamic, Dynamic> H;

  //! State size
  unsigned stateSize;

  //! Measurement size
  unsigned measSize;

  //! Input size
  unsigned inputSize;
  
  //! Whether the filter has been initiated or not
  bool initiated;

  //! Measurement noise covariance matrix which can be varied through subsequent cycles
  Matrix<Scalar, Dynamic, Dynamic> R;

  //! Last measurement vector
  Matrix<Scalar, Dynamic, 1> lastMeas;
};
