/**
 * @file Utils/include/MathsUtils.h
 *
 * This file defines the class MathsUtils.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#ifndef _MATHS_UTILS_H_
#define _MATHS_UTILS_H_

#include "Eigen/Dense"
#include "Eigen/SVD"
#include "Eigen/StdVector"
#include "Utils/include/DebugUtils.h"

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2f)
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix<float, 6, 6>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix<float, 3, 4>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(vector<Eigen::Matrix4f>)

using namespace Eigen;
using namespace std;

namespace Utils
{

#define U_TWO_PI 2 * U_PI
#define U_PI_2 M_PI / 2
#define U_PI_4 M_PI / 4
#define U_PI M_PI

  /**
   * @class MathsUtils
   * @brief Class that provides functions for handling mathematical
   *   computations.
   */
  class MathsUtils
  {
  public:

    /**
     * @brief Class constructor.
     */
    MathsUtils()
    {
    }
    ;

    /**
     * @brief Class destructor.
     */
    ~MathsUtils()
    {
    }
    ;

    /**
     * @brief Returns the a-cotangent of a variable.
     * @param var Input variable
     * @return Scalar A-cotangent of the variable.
     */
    template<typename Scalar>
      static inline Scalar
      aCotan(const Scalar& var)
      {
        return atan(1 / var);
      }

    /**
     * @brief Converts an angle from range (0 to 2 * U_PI) to the
     *   range (-U_PI to U_PI).
     * @param angle Input angle between (0 to U_TWO_PI)
     * @return Scalar Output angle between (-U_PI to U_PI)
     */
    template<typename Scalar>
      static inline Scalar
      rangeToPi(const Scalar& angle)
      {
        if (angle > U_PI) return angle - U_TWO_PI;
        else if (angle < -U_PI) return angle + U_TWO_PI;
        return angle;
      }

    /**
     * @brief Finds the smallest difference between two angles in range
     *   (-U_PI to U_PI)
     * @param a1 First angle
     * @param a2 Second angle
     * @return Scalar Difference of angles
     */
    template<typename Scalar>
      static inline Scalar
      diffAngle(const Scalar& a1, const Scalar& a2)
      {
        ASSERT(abs(a1) < U_PI && abs(a2) < U_PI);
        if (a1 * a2 >= 0) return rangeToPi(a1 - a2);
        if (a1 < 0) return rangeToPi(U_TWO_PI + a1 - a2);
        return rangeToPi(a1 - (U_TWO_PI + a2));
      }

    /**
     * @brief Adds two angles in range (-U_PI to U_PI)
     * @param a1 First angle
     * @param a2 Second angle
     * @return Scalar Addition of angles
     */
    template<typename Scalar>
      static inline Scalar
      addAngles(const Scalar& a1, const Scalar& a2, const Scalar& factor2)
      {
        ASSERT(abs(a1) < U_PI && abs(a2) < U_PI);
        auto ca1 = rangeToPi(a1);
        auto ca2 = rangeToPi(a2);
        ca1 = (ca1 < 0) ? ca1 + U_TWO_PI : ca1;
        ca2 = (ca2 < 0) ? ca2 + U_TWO_PI : ca2;
        return rangeToPi(ca1 + factor2 * a2);
      }

    /**
     * @brief Returns the Euclidean distance between two 2D points.
     * @param x1 X coordinate of first point
     * @param y1 Y coordinate of first point
     * @param x2 X coordinate of second point
     * @param y2 Y coordinate of second point
     * @return Scalar Euclidean distance
     */
    template<typename Scalar>
      static inline Scalar
      dist(const Scalar& x1, const Scalar& y1, const Scalar& x2,
        const Scalar& y2)
      {
        return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
      }

    /**
     * @brief Returns the quaternion representation of given euler angles.
     * @param euler Euler angles with sequence of rotation ZYX.
     * @return Quaternion
     */
    template<typename Derived>
      static inline Quaternion<typename Derived::Scalar>
      eulerToQuaternion(const MatrixBase<Derived>& euler)
      {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
        EIGEN_STATIC_ASSERT(
          Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 1,
          THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
        typedef typename Derived::Scalar Scalar;
        Scalar cr = cos(euler[0] / 2);
        Scalar sr = sin(euler[0] / 2);
        Scalar cp = cos(euler[1] / 2);
        Scalar sp = sin(euler[1] / 2);
        Scalar cy = cos(euler[2] / 2);
        Scalar sy = sin(euler[2] / 2);
        Quaternion<Scalar> q;
        q.w() = cr * cp * cy + sr * sp * sy;
        q.x() = sr * cp * cy - cr * sp * sy;
        q.y() = cr * sp * cy + sr * cp * sy;
        q.z() = cr * cp * sy - sr * sp * cy;
        return q;
      }

    /**
     * @brief Returns the rotation matrix for a given quaternion.
     * @param q Input quaternion
     * @return MatrixBase<Derived> Rotation matrix
     */
    template<typename Derived>
      static inline MatrixBase<Derived>
      quaternionToMat(const QuaternionBase<Derived>& q)
      {
        return q.toRotationMatrix().matrix();
      }

    /**
     * @brief Returns the euler angles for a given rotation matrix.
     * @param rot Input rotation matrix
     * @return Matrix<typename Derived::Scalar, 3, 1> Euler angles
     */
    template<typename Derived>
      static inline Matrix<typename Derived::Scalar, 3, 1>
      matToEuler(const MatrixBase<Derived>& rot)
      {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
        EIGEN_STATIC_ASSERT(
          Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 3,
          THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
        auto r = atan2(rot(2, 1), rot(2, 2));
        auto p = asin(-rot(2, 0));
        auto y = atan2(rot(1, 0), rot(0, 0));
        return Matrix<typename Derived::Scalar, 3, 1>(r, p, y);
      }

    /**
     * @brief Returns a quaternion for a given rotation matrix.
     * @param rot Input rotation matrix
     * @return Quaternion
     */
    template<typename Derived>
      static inline Quaternion<typename Derived::Scalar>
      matToQuaternion(const MatrixBase<Derived>& rot)
      {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
        EIGEN_STATIC_ASSERT(
          Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 3,
          THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
        Quaternion<typename Derived::Scalar> q;
        auto q0 = sqrt(1 + rot(0, 0) + rot(1, 1) + rot(2, 2)) / 2;
        auto q1 = (rot(2, 1) - rot(1, 2)) / (4 * q0);
        auto q2 = (rot(0, 2) - rot(2, 0)) / (4 * q0);
        auto q3 = (rot(1, 0) - rot(0, 1)) / (4 * q0);
        q.w() = q0;
        q.x() = q1;
        q.y() = q2;
        q.z() = q3;
        return q;
      }

    /**
     * @brief Returns the rotation matrix representation for the given
     *   euler angles.
     * @param euler Euler angles with sequence of rotation ZYX
     * @return MatrixBase<Derived> Rotaion matrix
     */
    template<typename Derived>
      static inline MatrixBase<Derived>
      eulerToMat(const MatrixBase<Derived>& euler)
      {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
        EIGEN_STATIC_ASSERT(
          Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 1,
          THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
        auto cr = cos(euler[0]);
        auto sr = sin(euler[0]);
        auto cp = cos(euler[1]);
        auto sp = sin(euler[1]);
        auto cy = cos(euler[2]);
        auto sy = sin(euler[2]);
        Matrix<Derived, 3, 3> m;
        m << cp * cy, -cr * sy + sr * sp * cy, sr * sy + cr * sp * cy, cp * sy, cr * cy + sr * sp * sy, -sr * cy + cr * sp * sy, -sp, sr * cp, cr * cp;
        return m;
      }

    /**
     * @brief Returns the euler representation for a given quaternion.
     * @param q Input quaternion
     * @return Matrix<Scalar, 3, 1> Euler angles
     */
    template<typename Derived>
      static inline Matrix<typename Derived::Scalar, 3, 1>
      quaternionToEuler(const QuaternionBase<Derived>& q)
      {
        return matToEuler(quaternionToMat(q));
      }

    /**
     * @brief Returns the difference of orientation between two transformation matrices
     * @param t1: First transformation matrix
     * @param t2: Second transformation matrix
     * @return Matrix<Scalar, 3, 1> Difference of orientation along rho-theta-phi
     */
    template<typename Derived>
      static inline Matrix<typename Derived::Scalar, 3, 1>
    getOrientationDiff(const MatrixBase<Derived>& t1, const MatrixBase<Derived>& t2)
    {
        typedef typename Derived::Scalar Scalar;
        EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
        EIGEN_STATIC_ASSERT(
          (Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4) || (Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 3) || (Derived::RowsAtCompileTime == 2 && Derived::ColsAtCompileTime == 2),
          THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
        Matrix<Scalar, 3, 3> skew1t, skew2t, skew3t;
        Matrix<Scalar, 3, 3> skew1i, skew2i, skew3i;
        skew1t = MathsUtils::makeSkewMat((Matrix<Scalar, 3, 1>) t2.block(0, 0, 3, 1));
        skew2t = MathsUtils::makeSkewMat((Matrix<Scalar, 3, 1>) t2.block(0, 1, 3, 1));
        skew3t = MathsUtils::makeSkewMat((Matrix<Scalar, 3, 1>) t2.block(0, 2, 3, 1));
        skew1i = MathsUtils::makeSkewMat((Matrix<Scalar, 3, 1>) t1.block(0, 0, 3, 1));
        skew2i = MathsUtils::makeSkewMat((Matrix<Scalar, 3, 1>) t1.block(0, 1, 3, 1));
        skew3i = MathsUtils::makeSkewMat((Matrix<Scalar, 3, 1>) t1.block(0, 2, 3, 1));
        Matrix<Scalar, 3, 3> L = -0.5 * (skew1t * skew1i + skew2t * skew2i + skew3t * skew3i);
        Matrix<Scalar, 3, 1> orientError =
            0.5 * (skew1i * t2.block(0, 0, 3, 1) + skew2i * t2.block(0, 1, 3, 1) + skew3i * t2.block(0, 2, 3, 1));
        return L.inverse() * orientError;
    }

    /**
     * @brief Sets mat as a 4x4 transformation matrix based on the
     *   X-Y-Z translational coordinates.
     * @param mat: Output matrix
     * @param x X-translation
     * @param y Y-translation
     * @param z Z-translation
     * @return void
     */
    template<typename Derived>
      static inline void
      makeTranslation(MatrixBase<Derived>& mat,
        const typename Derived::Scalar& x, const typename Derived::Scalar& y,
        const typename Derived::Scalar& z)
      {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
        EIGEN_STATIC_ASSERT(
          Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4,
          THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
        mat.setIdentity();
        mat(0, 3) = x;
        mat(1, 3) = y;
        mat(2, 3) = z;
      }

    /**
     * @brief Sets mat as a 4x4 transformation matrix or a 3x3 rotation
     *   matrix for rotation about Z-axis.
     * @param mat Output matrix
     * @param angle Angle of rotation
     * @return void
     */
    template<typename Derived>
      static void
      makeRotationZ(MatrixBase<Derived>& mat,
        const typename Derived::Scalar& angle)
      {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
        EIGEN_STATIC_ASSERT(
          (Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4) || (Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 3) || (Derived::RowsAtCompileTime == 2 && Derived::ColsAtCompileTime == 2),
          THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
        mat.setIdentity();
        auto ca = cos(angle);
        auto sa = sin(angle);
        mat(0, 0) = ca;
        mat(0, 1) = -sa;
        mat(1, 0) = sa;
        mat(1, 1) = ca;
      }

    /**
     * @brief Sets mat as a 4x4 transformation matrix or a 3x3 rotation
     *   matrix for rotation about Y-axis.
     * @param mat Output matrix
     * @param angle Angle of rotation
     * @return void
     */
    template<typename Derived>
      static inline void
      makeRotationY(MatrixBase<Derived>& mat,
        const typename Derived::Scalar& angle)
      {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
        EIGEN_STATIC_ASSERT(
          (Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4) || (Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 3),
          THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
        mat.setIdentity();
        auto ca = cos(angle);
        auto sa = sin(angle);
        mat(0, 0) = ca;
        mat(0, 2) = sa;
        mat(2, 0) = -sa;
        mat(2, 2) = ca;
      }

    /**
     * @brief Sets mat as a 4x4 transformation matrix or a 3x3 rotation
     *   matrix for rotation about X-axis.
     * @param mat Output matrix
     * @param angle Angle of rotation
     * @return void
     */
    template<typename Derived>
      static inline void
      makeRotationX(MatrixBase<Derived>& mat,
        const typename Derived::Scalar& angle)
      {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
        EIGEN_STATIC_ASSERT(
          (Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4) || (Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 3),
          THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
        mat.setIdentity();
        auto ca = cos(angle);
        auto sa = sin(angle);
        mat(1, 1) = ca;
        mat(1, 2) = -sa;
        mat(2, 1) = sa;
        mat(2, 2) = ca;
      }

    /**
     * @brief Sets mat as a 4x4 transformation matrix or a 3x3 rotation
     *   matrix for rotations about X-Y-Z axes.
     * @param mat Output matrix
     * @param xAngle Angle of rotation about the x-axis
     * @param yAngle Angle of rotation about the y-axis
     * @param zAngle Angle of rotation about the z-axis
     * @return void
     */
    template<typename Derived>
      static inline void
      makeRotationXYZ(MatrixBase<Derived>& mat,
        const typename Derived::Scalar& xAngle,
        const typename Derived::Scalar& yAngle,
        const typename Derived::Scalar& zAngle)
      {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
        EIGEN_STATIC_ASSERT(
          (Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4) || (Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 3),
          THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
        Derived rX, rY, rZ;
        makeRotationX(rX, xAngle);
        makeRotationY(rY, yAngle);
        makeRotationZ(rZ, zAngle);
        rX *= rY;
        rX *= rZ;
        mat = rX;
      }

    /**
     * @brief Sets mat as a 4x4 transformation matrix or a 3x3 rotation
     *   matrix for rotations about Z-Y-X axes.
     * @param mat Output matrix
     * @param zAngle Angle of rotation about the z-axis
     * @param yAngle Angle of rotation about the y-axis
     * @param xAngle Angle of rotation about the x-axis
     * @return void
     */
    template<typename Derived>
      static inline void
      makeRotationZYX(MatrixBase<Derived>& mat,
        const typename Derived::Scalar& zAngle,
        const typename Derived::Scalar& yAngle,
        const typename Derived::Scalar& xAngle)
      {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
        EIGEN_STATIC_ASSERT(
          (Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4) || (Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 3),
          THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
        Derived rX, rY, rZ;
        makeRotationX(rX, xAngle);
        makeRotationY(rY, yAngle);
        makeRotationZ(rZ, zAngle);
        rZ *= rY;
        rZ *= rX;
        mat = rZ;
      }

    /**
     * @brief Sets mat as a 4x4 transformation matrix for a given
     *   3x3 rotation matrix and X-Y-Z translational coordinates.
     * @param mat Output matrix
     * @param rot Rotation matrix
     * @param x X-translation
     * @param y Y-translation
     * @param z Z-translation
     * @return void
     */
    template<typename Derived>
      static inline Matrix<typename Derived::Scalar, 4, 4>
      makeTransformation(const MatrixBase<Derived>& rot,
        const typename Derived::Scalar& x, const typename Derived::Scalar& y,
        const typename Derived::Scalar& z)
      {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
        EIGEN_STATIC_ASSERT(
          Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 3,
          THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
        Matrix<typename Derived::Scalar, 4, 4> mat;
        mat.setIdentity();
        mat.block(0, 0, 3, 3) = rot;
        mat(0, 3) = x;
        mat(1, 3) = y;
        mat(2, 3) = z;
        return mat;
      }

    /**
     * @brief Returns a 3x1 vector of euler angles of sequence ZYX from a
     *   given 4x4 transformation pr 3x3 rotation matrix.
     * @param mat Input matrix
     * @return Matrix<typename Derived::Scalar, 3, 1>
     */
    template<typename Derived>
      static inline Matrix<typename Derived::Scalar, 3, 1>
      getEulerAngles(const MatrixBase<Derived>& mat)
      {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
        EIGEN_STATIC_ASSERT(
          (Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4) || (Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 3),
          THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
        Matrix<typename Derived::Scalar, 3, 1> euler;
        Matrix<typename Derived::Scalar, 3, 3> rot = mat.block(0, 0, 3, 3);
        euler = rot.eulerAngles(2, 1, 0);
        /*if (mat(2, 0) < 1) {
         if (mat(2, 0) > -1) {
         euler[0] = atan2(mat(2, 1), mat(2, 2));
         euler[1] = asin(-mat(2, 0));
         euler[2] = atan2(mat(1, 0), mat(0, 0));
         } else {
         euler[0] = 0;
         euler[1] = +U_PI / 2;
         euler[2] = -atan2(-mat(1, 2), mat(1, 1));
         }
         } else {
         euler[0] = 0;
         euler[1] = -U_PI / 2;
         euler[2] = atan2(-mat(1, 2), mat(1, 1));
         }*/
        return euler;
      }

    /**
     * @brief Returns the inverse of a 4x4 transformation matrix.
     * @param mat Input matrix
     * @return Derived
     */
    template<typename Derived>
      static inline Derived
      getTInverse(const MatrixBase<Derived>& mat)
      {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
        EIGEN_STATIC_ASSERT(
          Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4,
          THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
        auto Rt = mat.block(0, 0, 3, 3).transpose();
        auto t = mat.block(0, 3, 3, 1);
        Derived inv;
        inv.setIdentity();
        inv.block(0, 0, 3, 3) = Rt;
        inv.block(0, 3, 3, 1) = -Rt * t;
        return inv;
      }

    /**
     * @brief Sets mat as a 4x4 DH-transformation matrix for the given
     *   Denavit-Hartenbarg (DH) parameters.
     * @param mat Output transformation matrix
     * @param a Link length
     * @param alpha Link twist
     * @param d Joint offset
     * @param theta Joint angle
     * @return void
     */
    template<typename Derived>
      static inline void
      makeDHTransformation(MatrixBase<Derived>& mat,
        const typename Derived::Scalar& a,
        const typename Derived::Scalar& alpha,
        const typename Derived::Scalar& d,
        const typename Derived::Scalar& theta)
      {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
        EIGEN_STATIC_ASSERT(
          Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4,
          THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
        mat.setIdentity();
        auto ct = cos(theta);
        auto st = sin(theta);
        auto ca = cos(alpha);
        auto sa = sin(alpha);
        mat(0, 0) = ct;
        mat(0, 1) = -st;
        mat(0, 2) = 0;
        mat(0, 3) = a;
        mat(1, 0) = st * ca;
        mat(1, 1) = ct * ca;
        mat(1, 2) = -sa;
        mat(1, 3) = -sa * d;
        mat(2, 0) = st * sa;
        mat(2, 1) = ct * sa;
        mat(2, 2) = ca;
        mat(2, 3) = ca * d;
      }

    /**
     * @brief Transforms a 3x1 vector by a 4x4 transformation matrix.
     * @param mat Transformation matrix
     * @param vec Transformed vector
     * @return Matrix<typename Derived::Scalar, 3, 1>
     */
    template<typename Derived>
      static inline Matrix<typename Derived::Scalar, 3, 1>
      transformVector(const MatrixBase<Derived>& mat,
        const Matrix<typename Derived::Scalar, 3, 1> & vec)
      {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
        EIGEN_STATIC_ASSERT(
          Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4,
          THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
        Matrix<typename Derived::Scalar, 4, 1> temp;
        temp << vec, 1;
        temp = mat * temp;
        return temp.segment(0, 3);
      }

    /**
     * @brief Returns a 3x3 skew symmetric matrix from a given 3x1 vector.
     * @param vec Input vector
     * @return Matrix<Derived::Scalar, 3, 3>
     */
    template<typename Derived>
      static inline Matrix<typename Derived::Scalar, 3, 3>
      makeSkewMat(const MatrixBase<Derived>& vec)
      {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
        EIGEN_STATIC_ASSERT(
          Derived::RowsAtCompileTime == 3 && Derived::ColsAtCompileTime == 1,
          THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
        Matrix<typename Derived::Scalar, 3, 3> res;
        res.setZero();
        res(0, 1) = -vec[2];
        res(0, 2) = vec[1];
        res(1, 0) = vec[2];
        res(1, 2) = -vec[0];
        res(2, 0) = -vec[1];
        res(2, 1) = vec[0];
        return res;
      }

    /**
     * @brief Sets the input transformation matrix to its mirrored
     *   transformation.
     * @param mat Transformation matrix
     * @return void
     */
    template<typename Derived>
      static inline Derived
      mirrorTransformation(const MatrixBase<Derived>& mat)
      {
        EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
        EIGEN_STATIC_ASSERT(
          Derived::RowsAtCompileTime == 4 && Derived::ColsAtCompileTime == 4,
          THIS_METHOD_IS_ONLY_FOR_MATRICES_OF_A_SPECIFIC_SIZE);
        Derived tmp = mat;
        tmp(0, 1) = -mat(0, 1);
        tmp(1, 0) = -mat(1, 0);
        tmp(1, 2) = -mat(1, 2);
        tmp(2, 1) = -mat(2, 1);
        tmp(1, 3) = -mat(1, 3);
        return tmp;
      }

    /**
     * @brief Returns true if the two matrices are almost equal.
     * @param first First matrix
     * @param second Second matrix
     * @param tol Tolerance value for magnitude comparison
     * @return boolean
     */
    template<typename Derived>
      static inline bool
      almostEqual(const Derived& first, const Derived& second,
        const typename Derived::Scalar tol = typename Derived::Scalar(0.5))
    {
      typename Derived::Scalar n1 = (getEulerAngles(first) - getEulerAngles(
        second)).norm();
      typename Derived::Scalar n2 = (first.block(0, 3, 3, 1) - second.block(
        0,
        3,
        3,
        1)).norm();
      return (n1 < tol && n2 < 1e-3);
    }
    
    /**
     * @brief Returns true if the scalar values are almost equal
     * @param first First value
     * @param second Second value
     * @param tol Tolerance value for magnitude comparison
     * @return boolean
     */
    template<typename Scalar>
    static inline bool
      almostEqual(
        const Scalar& first, 
        const Scalar& second, 
        const Scalar tol)
    {
      return std::abs(first - second) < tol;
    }

    /**
     * @brief Returns cosine of a value within range of -1 to 1.
     * @param var Input variable
     * @return Scalar
     */
    template<typename Scalar>
      static inline Scalar
      safeAcos(Scalar var)
      {
        if (var < -1.0) var = -1.0;
        else if (var > 1.0) var = 1.0;
        return acos(var);
      }

    /**
     * @brief Returns the pseudo inverse of a matrix using singular value
     *   decomposition.
     * @param mat Input matrix
     * @param tolerance The pseudoInverse result tolerance
     * @return Eigen::MatrixBase<Derived>
     */
    template<typename Derived>
      static inline Eigen::Matrix<typename Derived::Scalar,
        Derived::ColsAtCompileTime, Derived::RowsAtCompileTime>
      pseudoInverse(const MatrixBase<Derived>& mat,
        const typename Derived::Scalar tolerance = typename Derived::Scalar(
          1e-4))
      {
        typedef typename Derived::Scalar Scalar;
        typedef typename internal::plain_row_type<Derived>::type RowVectorType;
        JacobiSVD<Derived> svd(mat, ComputeThinU | ComputeThinV);
        RowVectorType singularValues = svd.singularValues();
        Derived singularValuesInv(singularValues.size(), singularValues.size());
        singularValuesInv.setZero();
        for (unsigned int i = 0; i < singularValues.size(); ++i) {
          if (singularValues[i] > tolerance) singularValuesInv(i, i) =
            1 / singularValues[i];
          else singularValuesInv(i, i) = 0;
        }
        return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
      }

    /**
     * @brief Returns the pseudo inverse multiplication with an right hand
     *   side matrix.
     * @param mat Input matrix
     * @param rhs Right hand side matrix
     * @param tolerance PseudoInverse result tolerance.
     * @return Eigen::MatrixBase<Derived>
     */
    template<typename Derived>
      static inline Eigen::Matrix<typename Derived::Scalar,
        Derived::ColsAtCompileTime, Derived::RowsAtCompileTime>
      pseudoInverseSolve(const MatrixBase<Derived>& mat,
        const Matrix<typename Derived::Scalar, Derived::ColsAtCompileTime,
          Derived::RowsAtCompileTime> &rhs,
        const typename Derived::Scalar tolerance = typename Derived::Scalar(
          1e-4))
      {
        JacobiSVD<Derived> svd(mat, ComputeThinU | ComputeThinV);
        return svd.solve(rhs);
      }
  };

  /**
   * @brief Addition operator for standard vectors.
   * @param first First vector
   * @param second Second vector
   * @return vector
   */
  template<typename Scalar>
    static inline vector<Scalar>
    operator+(const vector<Scalar>& first, const vector<Scalar>& second)
    {
      ASSERT(first.size() == second.size());
      vector<Scalar> result;
      result.reserve(first.size());
      transform(
        first.begin(),
        first.end(),
        second.begin(),
        back_inserter(result),
        plus<Scalar>());
      return result;
    }

  /**
   * @brief Difference operator for standard vectors.
   * @param first First vector
   * @param second Second vector
   * @return vector
   */
  template<typename Scalar>
    static inline vector<Scalar>
    operator-(const vector<Scalar>& first, const vector<Scalar>& second)
    {
      ASSERT(first.size() == second.size());
      vector<Scalar> result;
      result.reserve(first.size());
      transform(
        first.begin(),
        first.end(),
        second.begin(),
        back_inserter(result),
        minus<Scalar>());
      return result;
    }

  /**
   * @brief Scalar multiplication operator for standard vectors.
   * @param first First vector
   * @param second Second vector
   * @return vector
   */template<typename Scalar>
    static inline vector<Scalar>
    operator*(const vector<Scalar>& vec, const Scalar& constant)
    {
      vector<Scalar> result;
      result.reserve(vec.size());
      transform(
        vec.begin(),
        vec.end(),
        result.begin(),
        bind2nd(multiplies<Scalar>(), constant));
      return result;
    }

}
#endif //! _MATHS_UTILS_H_
