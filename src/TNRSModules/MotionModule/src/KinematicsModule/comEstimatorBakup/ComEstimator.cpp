/**
 * @file MotionModule/src/KinematicsModule/ComEstimator.cpp
 *
 * This file implements the class ComEstimator
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Jan 2018
 */

#include "MotionModule/include/KinematicsModule/ComEstimator.h"
#include "Utils/include/EnvConsts.h"
#include "Utils/include/DataUtils.h"

using namespace Utils;

void ComEstimator::init(const Mat& initState, const double& comHeight, const double& dT)
{
  unsigned int type = CV_64F;
  kFilter = KalmanFilter(stateSize, measSize, contrSize, type);
	state = Mat(stateSize, 1, type);
	meas = Mat(measSize, 1, type);
	measNoiseCov = Mat(measSize, measSize, type);

	//! Transition State Matrix A
  //! [ 1   dT  dT^2/2 ] [  x     ]
  //! [ 0   1   dT     ] [  xdot  ]
  //! [ 0   0   1      ] [  xddot ]
  setIdentity(kFilter.transitionMatrix);
  kFilter.transitionMatrix.at<double>(1) = dT;
  kFilter.transitionMatrix.at<double>(2) = dT * dT / 2;
  kFilter.transitionMatrix.at<double>(5) = dT;

	//! Measure Matrix H
  //! [ 1 0 0 ] [  x   ]
  //! [ 0 0 1 ] [  xdot ]
  //! [ 1 0 -z/g ] [  xddot  ]
  kFilter.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
  kFilter.measurementMatrix.at<double>(0) = 1.0;
  kFilter.measurementMatrix.at<double>(5) = 1.0;
  kFilter.measurementMatrix.at<double>(6) = 1.0;
  kFilter.measurementMatrix.at<double>(8) = -comHeight / gConst;

	//! Process Noise Covariance Matrix Q
  //! [ eX   0     0   ]
  //! [ 0    eVX   0   ]
  //! [ 0    0     eAx ]
  kFilter.processNoiseCov = cv::Mat::zeros(stateSize, stateSize, type);
  kFilter.processNoiseCov.at<double>(0) = 1e-3;
  kFilter.processNoiseCov.at<double>(4) = 1e-3;
  kFilter.processNoiseCov.at<double>(8) = 1e-3;

	//! Measurement Noise Covariance Matrix Q
  //! [ eX   0     0  ]
  //! [ 0    eAx   0  ]
  //! [ 0    0     eZx]
  measNoiseCov = cv::Mat::zeros(measSize, measSize, type);
  measNoiseCov.at<double>(0) = 1e-2;
  measNoiseCov.at<double>(4) = 1e-1;
  measNoiseCov.at<double>(8) = 999;
  kFilter.measurementNoiseCov = measNoiseCov.clone();
  setIdentity(kFilter.errorCovPre);
  state = initState;
  kFilter.statePost = initState;
  kFilter.statePre = initState;
  meas = kFilter.measurementMatrix * initState;
  initiated = true;
}

void ComEstimator::reset(const Mat& state)
{
	this->state = state;
  kFilter.statePre = state;
  kFilter.statePost = state;
}

Mat ComEstimator::predict()
{
  return kFilter.predict();
}

Mat ComEstimator::getEstimatedState()
{
  return kFilter.statePost;
}

Mat ComEstimator::getOutput()
{
  return kFilter.measurementMatrix * kFilter.statePost;
}

void ComEstimator::correct(const Mat& meas, const double& comHeight)
{
  this->meas = meas;
  //LOG_INFO("meas:")
  //LOG_INFO(DataUtils::varToString(meas.at<double>(0)))
  //LOG_INFO(DataUtils::varToString(meas.at<double>(1)))
  //LOG_INFO(DataUtils::varToString(meas.at<double>(2)))

  for (size_t i = 0; i < this->meas.rows; ++i) {
    if (this->meas.at<double>(i) != this->meas.at<double>(i)) {
      kFilter.measurementNoiseCov.at<double>(i*4) = 1e9;
      this->meas.at<double>(i) = 0;
    } else {
      kFilter.measurementNoiseCov.at<double>(i*4) = measNoiseCov.at<double>(i*4);
    }
  }
  //LOG_INFO("statePre:")
  //LOG_INFO(DataUtils::varToString(kFilter.statePre.at<double>(0)))
  //LOG_INFO(DataUtils::varToString(kFilter.statePre.at<double>(1)))
  //LOG_INFO(DataUtils::varToString(kFilter.statePre.at<double>(2)))
  kFilter.correct(this->meas); //! Kalman Correction
  //LOG_INFO("statePost:")
  //LOG_INFO(DataUtils::varToString(kFilter.statePost.at<double>(0)))
  //LOG_INFO(DataUtils::varToString(kFilter.statePost.at<double>(1)))
  //LOG_INFO(DataUtils::varToString(kFilter.statePost.at<double>(2)))
  /*cout << "Meas:\n" << meas << endl;
  cout << "kFilter.statePre:\n" << kFilter.statePre << endl;
  cout << "kFilter.statePost:\n" << kFilter.statePost << endl;
  cout << "kFilter.transitionMatrix:\n" << kFilter.transitionMatrix << endl;
  cout << "kFilter.measurementMatrix:\n" << kFilter.measurementMatrix << endl;
  cout << "kFilter.processNoiseCov:\n" << kFilter.processNoiseCov << endl;
  cout << "kFilter.measNoiseCov:\n" << kFilter.measurementNoiseCov << endl;
  cout << " (z(k)-H*x'(k)) " << meas - kFilter.measurementMatrix * kFilter.statePre << endl;
  cout << " K(k) " << kFilter.gain << endl;*/
}
