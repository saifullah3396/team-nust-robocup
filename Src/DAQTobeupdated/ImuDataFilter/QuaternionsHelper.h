/**
 * @file ImuDataFilter/ImuDataFilter.h
 *
 * This file declares a class for filteration of Imu data.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 30 Sep 2017
 */

#ifndef _QUATERNIONS_HELPER_H_
#define _QUATERNIONS_HELPER_H_

#include "Utils/MathsUtils.h"

namespace Utils
{

//diff_(p*q) /diff_q
  template<typename T>
    Matrix<T, 4, 4>
    diff_pq_q(Quaternion<T> p)
    {
      T p0 = p.w();
      Matrix<T, 3, 1> pv = p.vec();
      Matrix<T, 4, 4> D;
      D(0, 0) = p0;
      D.block(0, 1, 1, 3) = -pv.transpose();
      D.block(1, 0, 3, 1) = pv;
      D.block(1, 1, 3, 3) = Matrix < T, 3, 3 > ::Identity() * p0 + MathsUtils::makeSkewMat(
        pv);
      return D;
    }

//diff_(p*q)/ diff_p
  template<typename T>
    Matrix<T, 4, 4>
    diff_pq_p(Quaternion<T> q)
    {
      T q0 = q.w();
      Matrix<T, 3, 1> qv = q.vec();
      Matrix<T, 4, 4> D;
      D(0, 0) = q0;
      D.block(0, 1, 1, 3) = -qv.transpose();
      D.block(1, 0, 3, 1) = qv;
      D.block(1, 1, 3, 3) = Matrix < T, 3, 3 > ::Identity() * q0 - MathsUtils::makeSkewMat(
        qv);
      return D;
    }

//diff_(q*v*q_star)/ diff_q
  template<typename T>
    Matrix<T, Dynamic, Dynamic>
    diff_qvqstar_q(Quaternion<T> q, Matrix<T, 3, 1> v)
    {
      T q0 = q.w();
      Matrix<T, 3, 1> qv = q.vec();
      Matrix<T, Dynamic, Dynamic> D(3, 4);
      D.col(0) = 2 * (q0 * v + MathsUtils::makeSkewMat(qv) * v);
      D.block(0, 1, 3, 3) =
        2 * (-v * qv.transpose() + v.dot(qv) * Matrix < T, 3, 3 > ::Identity() - q0 * MathsUtils::makeSkewMat(
          v));
      return D;
    }

//diff_(qstar*v*q)/ diff_q
  template<typename T>
    Matrix<T, Dynamic, Dynamic>
    diff_qstarvq_q(Quaternion<T> q, Matrix<T, 3, 1> v)
    {
      T q0 = q.w();
      Matrix<T, 3, 1> qv = q.vec();
      Matrix<T, Dynamic, Dynamic> D(3, 4);
      D.col(0) = 2 * (q0 * v - MathsUtils::makeSkewMat(qv) * v);
      D.block(0, 1, 3, 3) =
        2 * (-v * qv.transpose() + v.dot(qv) * Matrix < T, 3, 3 > ::Identity() + q0 * MathsUtils::makeSkewMat(
          v));
      return D;
    }

//diff_(q*v*q_star)/ diff_v
  template<typename T>
    Matrix<T, 3, 3>
    diff_qvqstar_v(Quaternion<T> q)
    {
      T q0 = q.w();
      Matrix<T, 3, 1> qv = q.vec();
      Matrix<T, 3, 3> D;
      D = (q0 * q0 - qv.dot(qv)) * Matrix < T, 3, 3 > ::Identity() + 2 * qv * qv.transpose() + 2 * q0 * MathsUtils::makeSkewMat(
        qv);
      return D;
    }

}
#endif //! _QUATERNIONS_HELPER_H_
