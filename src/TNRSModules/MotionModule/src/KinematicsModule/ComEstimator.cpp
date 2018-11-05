/**
 * @file MotionModule/src/KinematicsModule/ComEstimator.cpp
 *
 * This file implements the class ComEstimator
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Jan 2018
 */

#include <boost/make_shared.hpp>
#include "MotionModule/include/KinematicsModule/ComEstimator.h"
#include "MotionModule/include/MTypeHeader.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/ConfigMacros.h"

using namespace Utils;

template <typename Scalar>
void ComEstimator<Scalar>::init(
  const Matrix<Scalar, Dynamic, 1>& initState,
  const Scalar& comHeight,
  const Scalar& dT)
{
  model = boost::make_shared<ProcessModel<Scalar> >(stateSize, inputSize);
  Matrix<Scalar, Dynamic, Dynamic> A(stateSize, stateSize), Q(stateSize, stateSize);
  Matrix<Scalar, Dynamic, 1> B(stateSize);
  C.resize(1, stateSize);
  //! State Transition Matrix A
  //! [ 1   dT  dT^2/2 ] [  x     ]
  //! [ 0   1   dT     ] [  xdot  ]
  //! [ 0   0   1      ] [  xddot ]
  A << 1.f, dT, dT * dT / 2, 0.f, 1.f, dT, 0.f, 0.f, 1.f;
  //! Control Input Matrix B
  //! [ dT^3/6 ]
  //! [ dT^2/2 ]
  //! [ dT ]
  B << dT * dT * dT/ 6, dT * dT / 2, dT;
  model->setStateMatrix(A);
  model->setControlMatrix(B);
  C << 1, 0, -comHeight / gConst;
  model->setOutputMatrix(C);
  //! Process Noise Covariance Matrix Q
  //! [ eX   0     0   ]
  //! [ 0    eVX   0   ]
  //! [ 0    0     eAx ]
  Scalar procVar1, procVar2, procVar3;
  GET_CONFIG(
    "KinCalibration",
    (Scalar, ComEstimator.procPosVar, procVar1),
    (Scalar, ComEstimator.procVelVar, procVar2),
    (Scalar, ComEstimator.procAccVar, procVar3),
  );
  Q.setZero();
  Q(0, 0) = procVar1;
  Q(1, 1) = procVar2;
  Q(2, 2) = procVar3;
  model->setNoiseCovMatrix(Q);

  //! Initial state
  model->setState(initState);
  H.resize(measSize, measSize);
  filter = boost::make_shared<KalmanFilter<Scalar> >(model, measSize);
  //! Measure Matrix H
  //! [ x    ] [ 1 0    0 ] [ x    ]
  //! [ xddot] [ 0 0    1 ] [ xdot ]
  //! [ zmp  ] [ 1 0 -z/g ] [ xddot]
  H << 1, 0, 0, 0, 0, 1, 1, 0, -comHeight / gConst;
  filter->setMeasMatrix(H);
  this->R.resize(measSize, measSize);
  this->R.setZero();
  Scalar measVar1, measVar2, measVar3;
  GET_CONFIG(
    "KinCalibration",
    (Scalar, ComEstimator.measPosVar, measVar1),
    (Scalar, ComEstimator.measAccVar, measVar2),
    (Scalar, ComEstimator.measZmpVar, measVar3),
  );
  this->R(0, 0) = measVar1;
  this->R(1, 1) = measVar2;
  this->R(2, 2) = measVar3;
  lastMeas.resize(measSize);
  lastMeas.setZero();
  filter->setMeasNoiseCov(R);
  initiated = true;
}

template <typename Scalar>
void ComEstimator<Scalar>::update(Matrix<Scalar, Dynamic, 1>& meas)
{
  for (size_t i = 0; i < meas.rows(); ++i) {
    if (meas[i] != meas[i] || MathsUtils::almostEqual(meas[i], lastMeas[i], 1e-5)) {
      // High covariance for unknown measurement
      filter->setMeasNoiseCov(1e6, i, i);
      meas[i] = 0.0;
    } else {
      filter->setMeasNoiseCov(R(i, i), i, i);
    }
  }
  filter->predict();
  filter->correct(meas);
  lastMeas = meas;
}

template class ComEstimator<MType>;
