/**
 * @file ImuDataFilter/ImuDataFilter.cpp
 *
 * This file implements a class for filteration of Imu data.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 30 Sep 2017
 */

#include "ImuDataFilter.h"

template<typename T>
  ImuDataFilter<T>::ImuDataFilter(DAQModule* daqModule) :
    Filter(daqModule)
  {
    x.resize(nState);
    x.setZero();
    x.head(4) << 1, 0, 0, 0;
    P = Matrix < T, Dynamic, Dynamic > ::Identity(nState, nState);
    Q = Matrix < T, Dynamic, Dynamic > ::Zero(6, 6);
    Q.block(0, 0, 3, 3) = Matrix < T, 3, 3 > ::Identity() * gyroCov;
    Q.block(3, 3, 3, 3) = Matrix < T, 3, 3 > ::Identity() * accCov;
    initiated = false;
  }

template<typename T>
  void
  ImuDataFilter<T>::initiate()
  {
    timeStep = 0;
    acc[0] = IVAR(vector<float>, DAQModule::inertialSensors)[6];
    acc[1] = IVAR(vector<float>, DAQModule::inertialSensors)[7];
    acc[2] = IVAR(vector<float>, DAQModule::inertialSensors)[8];

    acc[0] = 0;
    acc[1] = 0;
    acc[2] = -9.81;

    Random gen;
    NormalDistribution<double> accRX;
    NormalDistribution<double> accRY;
    NormalDistribution<double> accRZ;
    acc[0] = accRX(gen, acc[0], 0.32);
    acc[1] = accRY(gen, acc[1], 0.32);
    acc[2] = accRZ(gen, acc[2], 0.32);

    cout << "Acceleration: " << endl << acc << endl;
    auto roll = atan2(acc[1], acc[2]);
    auto pitch = atan2(-acc[0], acc[2]);
    Matrix<T, 3, 1> rpy(roll, pitch, 0);
    auto q = eulerToQuaternion(rpy);
    x.setZero();
    x[0] = q.w();
    x.segment(1, 3) = q.vec();
    cout << "rpy: " << roll << " " << pitch << endl;
    cout << "q: " << x.segment(0, 4) << endl;
    Matrix<T, 3, 1> euler = quaternionToEuler(q);
    cout << "StartAngle: " << endl;
    cout << euler << endl;

    initiated = true;
  }

template<typename T>
  void
  ImuDataFilter<T>::update()
  {
    gyro[0] = 0; //IVAR(vector<float>, DAQModule::inertialSensors)[0];
    gyro[1] = 0; //IVAR(vector<float>, DAQModule::inertialSensors)[1];
    gyro[2] = 0; //IVAR(vector<float>, DAQModule::inertialSensors)[2];
    acc[0] = 0; //IVAR(vector<float>, DAQModule::inertialSensors)[6];
    acc[1] = 0; //IVAR(vector<float>, DAQModule::inertialSensors)[7];
    acc[2] = -9.81; //IVAR(vector<float>, DAQModule::inertialSensors)[8];

    Random gen;
    NormalDistribution<double> gyroRX;
    NormalDistribution<double> gyroRY;
    NormalDistribution<double> gyroRZ;
    NormalDistribution<double> accRX;
    NormalDistribution<double> accRY;
    NormalDistribution<double> accRZ;
    gyro[0] = gyroRX(gen, gyro[0], 0.1);
    gyro[1] = gyroRY(gen, gyro[1], 0.1);
    gyro[2] = gyroRZ(gen, gyro[2], 0.1);
    acc[0] = accRX(gen, acc[0], 0.32);
    acc[1] = accRY(gen, acc[1], 0.32);
    acc[2] = accRZ(gen, acc[2], 0.32);

    cout << "Acceleration: " << endl << acc << endl;
    cout << "Gyro: " << endl << gyro << endl;

    predict();
    //correctAccMeasurement();
    Quaternion<T> q;
    Matrix<T, 3, 1> p, v, bw, ba;
    getState(q, p, v, bw, ba);
    Matrix<T, 3, 1> euler = quaternionToEuler(q);
    cout << "StateAngle: " << endl;
    cout << euler << endl;
  }

template<typename T>
  void
  ImuDataFilter<T>::predict()
  {
    Matrix<T, Dynamic, 1> xdot(nState);
    Matrix<T, Dynamic, Dynamic> F(nState, nState);
    Matrix<T, Dynamic, Dynamic> G(nState, 6);
    process(xdot, F, G);
    x += xdot * cycleTime;
    F = Matrix < T, Dynamic, Dynamic > ::Identity(nState, nState) + F * cycleTime;
    G = G * cycleTime;
    P = F * P * F.transpose() + G * Q * G.transpose();
    x.head(4).normalize();
    timeStep = timeStep + cycleTime;
  }

template<typename T>
  void
  ImuDataFilter<T>::process(Matrix<T, Dynamic, 1>& xdot,
    Matrix<T, Dynamic, Dynamic>& F, Matrix<T, Dynamic, Dynamic>& G)
  {
    Quaternion<T> q;
    Matrix<T, 3, 1> p, v, bw, ba;
    getState(q, p, v, bw, ba);
    xdot.setZero();
    F.setZero();
    G.setZero();

    Quaternion<T> gyroQ(0, 0, 0, 0);
    gyroQ.vec() = gyro - bw;
    Quaternion<T> qdot = q * gyroQ;
    qdot.w() /= 2;
    qdot.vec() /= 2;
    xdot(0) = qdot.w();
    xdot.segment(1, 3) = qdot.vec();
    xdot.segment(4, 3) = v;

    Quaternion<T> accbQ(0, 0, 0, 0);
    accbQ.vec() = acc - ba;
    Quaternion<T> accnQ = q * accbQ * q.inverse();
    xdot.segment(7, 3) = accnQ.vec() - gravVec;

    F.block(0, 0, 4, 4) = 0.5 * diff_pq_p(gyroQ);
    F.block(0, 10, 4, 3) = -0.5 * (diff_pq_q(q).block(0, 1, 4, 3));
    F.block(4, 7, 3, 3) = Matrix < T, 3, 3 > ::Identity();
    F.block(7, 0, 3, 4) = diff_qvqstar_q(q, Matrix<T, 3, 1>(accbQ.vec()));
    F.block(7, 13, 3, 3) = -diff_qvqstar_v(q);

    //G = d_xdot/du
    G.block(0, 0, 4, 3) = 0.5 * diff_pq_q(q).block(0, 1, 4, 3); //diff(0.5*q*gyro_q)/diff(gyro_q)
    G.block(7, 3, 3, 3) = diff_qvqstar_v(q); //diff(q*a*qstar)/diff(a)
  }

template<typename T>
  void
  ImuDataFilter<T>::getState(Quaternion<T>& q, Matrix<T, 3, 1>& pos,
    Matrix<T, 3, 1>& vel, Matrix<T, 3, 1>& omegaBias, Matrix<T, 3, 1>& accBias)
  {
    q.w() = x[0];
    q.vec() = x.segment(1, 3);
    pos = x.segment(4, 3);
    vel = x.segment(7, 3);
    omegaBias = x.segment(10, 3);
    accBias = x.segment(13, 3);
  }

template<typename T>
  void
  ImuDataFilter<T>::measurement(Matrix<T, 3, 1>& acc,
    Matrix<T, Dynamic, Dynamic>& H)
  {
    Quaternion<T> q;
    q.w() = x(0);
    q.vec() = x.segment(1, 3);
    Matrix<T, 3, 1> ba = x.segment(13, 3);
    Quaternion<T> gnQ;
    gnQ.w() = 0;
    gnQ.vec() = Matrix < T, 3, 1 > (0, 0, 1); //only direction is used
    auto accQ = q.inverse() * gnQ * q; //r_n to r_b
    acc = accQ.vec();
    H = Matrix < T, Dynamic, Dynamic > ::Zero(3, nState);
    H.block(0, 0, 3, 4) = diff_qstarvq_q(q, gravVec);
  }

template<typename T>
  void
  ImuDataFilter<T>::correction(Matrix<T, Dynamic, 1> z,
    Matrix<T, Dynamic, 1> zhat, Matrix<T, Dynamic, Dynamic> H,
    Matrix<T, Dynamic, Dynamic> R)
  {
    Matrix<T, Dynamic, Dynamic> K =
      P * H.transpose() * (H * P * H.transpose() + R).inverse();
    x += K * (z - zhat);
    Matrix<T, Dynamic, Dynamic> I = Matrix<T, Dynamic, Dynamic>::Identity(
      nState,
      nState);
    P = (I - K * H) * P;
    x.head(4).normalize();
  }

template<typename T>
  void
  ImuDataFilter<T>::correctAccMeasurement()
  {
    Matrix<T, 3, 1> z = acc / acc.norm();
    Matrix<T, 3, 1> zhat;
    Matrix<T, Dynamic, Dynamic> H;
    measurement(zhat, H);
    correction(z, zhat, H, gravR);
  }

template class ImuDataFilter<double> ;
template class ImuDataFilter<float> ;
