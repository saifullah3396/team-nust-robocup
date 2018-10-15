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

void ComEstimator::init(const Mat& initState, const double& comHeight, const double& dT)
{
  unsigned int type = CV_64F;
	kFilter = KalmanFilter(stateSize, measSize, contrSize, type);
	state = Mat(stateSize, 1, type);
	meas = Mat(measSize, 1, type);
	measNoiseCov = Mat(measSize, measSize, type);

	//! Transition State Matrix A
	//! [ 1   dT  dT^2/2 0   0   0      ] [  x     ]
	//! [ 0   1   dT     0   0   0      ] [  xdot  ]
	//! [ 0   0   1      0   0   0      ] [  xddot ]
  //! [ 0   0   0      1   dT  dT^2/2 ] [  y     ]
	//! [ 0   0   0      0   1   dT     ] [  ydot  ]
	//! [ 0   0   0      0   0   1      ] [  yddot ]
  setIdentity(kFilter.transitionMatrix);
	kFilter.transitionMatrix.at<double>(1) = dT;
	kFilter.transitionMatrix.at<double>(2) = dT * dT / 2;
	kFilter.transitionMatrix.at<double>(8) = dT;
  kFilter.transitionMatrix.at<double>(22) = dT;
	kFilter.transitionMatrix.at<double>(23) = dT * dT / 2;
	kFilter.transitionMatrix.at<double>(29) = dT;

	//! Measure Matrix H
	//! [ 1 0 0 0 0 0    ] [  x     ]
  //! [ 0 0 0 1 0 0    ] [  xdot  ]
  //! [ 1 0 -z/g 0 0 0 ] [  xddot ]
  //! [ 0 0 0 1 0 -z/g ] [  y     ]
  //!                    [  ydot  ]
  //!                    [  yddot ]
	kFilter.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
	kFilter.measurementMatrix.at<double>(0) = 1.0f;
	kFilter.measurementMatrix.at<double>(9) = 1.0f;
	kFilter.measurementMatrix.at<double>(12) = 1.0f;
  kFilter.measurementMatrix.at<double>(14) = -comHeight / gConst;
	kFilter.measurementMatrix.at<double>(21) = 1.0f;
  kFilter.measurementMatrix.at<double>(23) = -comHeight / gConst;

	//! Process Noise Covariance Matrix Q
	//! [ eX   0    0     0     0     0    ]
	//! [ 0    eY   0     0     0     0    ]
	//! [ 0    0    eVX   0     0     0    ]
	//! [ 0    0    0     eVY   0     0    ]
	//! [ 0    0    0     0     eAx   0    ]
	//! [ 0    0    0     0     0     eAY  ]
	kFilter.processNoiseCov.at<double>(0) = 0.001;
	kFilter.processNoiseCov.at<double>(7) = 0.001;
	kFilter.processNoiseCov.at<double>(14) = 0.01;
	kFilter.processNoiseCov.at<double>(21) = 0.01;
	kFilter.processNoiseCov.at<double>(28) = 0.01;
	kFilter.processNoiseCov.at<double>(35) = 0.01;

	//! Measurement Noise Covariance Matrix Q
	//! [ eX   0    0       0    ]
	//! [ 0    eY   0       0    ]
	//! [ 0    0    ezmpx   0    ]
	//! [ 0    0    0     ezmpy  ] 
	measNoiseCov.at<double>(0) = 5e-3;
	measNoiseCov.at<double>(5) = 5e-3;
	measNoiseCov.at<double>(10) = 5e-3;
	measNoiseCov.at<double>(15) = 5e-3;
	kFilter.measurementNoiseCov = measNoiseCov;
  setIdentity(kFilter.errorCovPre);
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

void ComEstimator::correct(const Mat& meas, const double& comHeight)
{
  this->meas = meas;
  for (size_t i = 0; i < this->meas.rows; ++i) {
    if (this->meas.at<double>(i) != this->meas.at<double>(i)) {
      kFilter.measurementNoiseCov.at<double>((int)i*5) = 1e9;
      this->meas.at<double>(i) = 0;
    } else {
      kFilter.measurementNoiseCov.at<double>((int)i*5) = measNoiseCov.at<double>((int)i*5);
    }
  }
  kFilter.correct(this->meas); //! Kalman Correction
}
