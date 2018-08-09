/**
 * @file MotionModule/TrajectoryPlanner/CubicSpline.h
 *
 * This file implements the class to generate cubic splines based on 
 * required conditions.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Aug 2017  
 */

#include "MotionModule/include/TrajectoryPlanner/CubicSpline.h"
#include "MotionModule/include/TrajectoryPlanner/TrajectoryPlanner.h"

CubicSpline::CubicSpline(const float& stepSize, const unsigned& splineDim,
  const unsigned& maxDerivative, const MatrixXf& controlPoints,
  const VectorXf& knots, const MatrixXf& boundaryVels
        ) :
  stepSize(stepSize), splineDim(splineDim), controlPoints(controlPoints),
    knots(knots), boundaryVels(boundaryVels)
{
  //!Dimensionality check
  ASSERT(controlPoints.rows() == (knots.size() + 1));
  ASSERT(controlPoints.cols() == splineDim);
  ASSERT(boundaryVels.rows() == 2);
  ASSERT(boundaryVels.cols() == splineDim);
  coeffs.resize(4);
  nKnots = knots.size();
  controlPointsU = controlPoints.block(1, 0, nKnots, splineDim);
  controlPointsL = controlPoints.block(0, 0, nKnots, splineDim);
  controlPointsDiff = controlPointsU - controlPointsL;
  A.resize(nKnots + 1, nKnots + 1);
  b.resize(nKnots + 1, splineDim);
  bAccels.resize(nKnots + 1, splineDim);
  bAccelsU.resize(nKnots, splineDim);
  bAccelsL.resize(nKnots, splineDim);
  A.setZero();
  b.setZero();
  bAccels.setZero();
  bAccelsU.setZero();
  bAccelsL.setZero();
  genParams();
  //plotSpline(100, 0.0);
}

void
CubicSpline::genParams()
{
  try {
    knotsRep = knots.replicate(1, splineDim);
    //cout << "knots:" << knots << endl;
    MatrixXf diff = controlPointsDiff.cwiseQuotient(knotsRep);
    for (int i = 0; i < splineDim; ++i) {
      b.block(0, i, nKnots + 1, 1) << 6 * (diff(0, i) - boundaryVels(0, i)), 6 * (diff.block(
        1,
        i,
        nKnots - 1,
        1) - diff.block(0, i, nKnots - 1, 1)), 6 * (boundaryVels(1, i) - diff(
        nKnots - 1,
        i));
    }
    genInvAMat();
    genCoeffs();
    //plotSpline(100, 0.0);
  } catch (exception &e) {
    cout << e.what() << endl;
  }
}

void
CubicSpline::genInvAMat()
{
  VectorXf upperLower, middle;
  upperLower.resize(nKnots);
  middle.resize(nKnots + 1);
  upperLower << knots;
  middle << 2 * knots(0), 2 * (knots.block(0, 0, nKnots - 1, 1) + knots.block(
    1,
    0,
    nKnots - 1,
    1)), 2 * knots(nKnots - 1);
  for (int i = 0; i < nKnots + 1; ++i) {
    for (int j = 0; j < nKnots + 1; ++j) {
      if (i == j) A(i, j) = middle(i);
      else if (i - j == -1) A(i, j) = upperLower(i);
      else if (i - j == 1) A(i, j) = upperLower(j);
    }
  }
  invA = A.inverse();
}

void
CubicSpline::evaluate(const VectorXf& knots)
{
  ASSERT(knots.size() == nKnots);
  this->knots = knots;
  genParams();
}

void
CubicSpline::genCoeffs()
{
  for (int i = 0; i < splineDim; ++i) {
    bAccels.block(0, i, nKnots + 1, 1) << invA * b.block(0, i, nKnots + 1, 1);
  }
  bAccels = invA * b;
  bAccelsL = bAccels.block(0, 0, nKnots, splineDim);
  bAccelsU = bAccels.block(1, 0, nKnots, splineDim);
  coeffs[0] = bAccelsL.cwiseQuotient(6 * knotsRep);
  coeffs[1] = bAccelsU.cwiseQuotient(6 * knotsRep);
  coeffs[2] = controlPointsU.cwiseQuotient(knotsRep) - bAccelsU.cwiseProduct(
    knotsRep / 6);
  coeffs[3] = controlPointsL.cwiseQuotient(knotsRep) - bAccelsL.cwiseProduct(
    knotsRep / 6);
}

void
CubicSpline::plotSpline(const unsigned& nInnerPoints, const float& startTime)
{
  ASSERT(coeffs[0].size() != 0);
  Gnuplot gp;
  float stepSize;
  stepSize = 1.0 / nInnerPoints;
  vector<float> times;
  vector < pair<float, float> > times_pos;
  vector < pair<float, float> > times_vel;
  vector < pair<float, float> > times_acc;
  times.resize(nKnots + 1);
  times[0] = startTime;
  for (int i = 1; i < times.size(); ++i) {
    times[i] = times[i - 1] + knots[i - 1];
  }

  gp << "set xrange [0:20]\nset yrange [0:20]\n";
  gp << "plot" << gp.file1d(times_pos) << "with lines title 'Cubic splines'" << endl;
  for (int k = 0; k < splineDim; ++k) {
    gp << "set terminal wxt " << k << endl;
    times_pos.clear();
    times_vel.clear();
    times_acc.clear();
    for (int i = 0; i < nKnots; ++i) {
      for (float t = times[i]; t < times[i + 1]; t = t + stepSize) {
        float pos =
          coeffs[0](i, k) * pow(times[i + 1] - t, 3) + coeffs[1](i, k) * pow(
            t - times[i],
            3) + coeffs[2](i, k) * (t - times[i]) + coeffs[3](i, k) * (times[i + 1] - t);
        float vel =
          -3 * coeffs[0](i, k) * pow(times[i + 1] - t, 2) + 3 * coeffs[1](i, k) * pow(
            t - times[i],
            2) + coeffs[2](i, k) - coeffs[3](i, k);
        float acc = 6 * coeffs[0](i, k) * (times[i + 1] - t) - 6 * coeffs[1](
          i,
          k) * (t - times[i]);
        times_pos.push_back(make_pair(t, pos));
        times_vel.push_back(make_pair(t, vel));
        times_acc.push_back(make_pair(t, acc));
      }
    }
    float t = times.back();
    float pos =
      coeffs[0](nKnots - 1, k) * pow(times[nKnots - 1 + 1] - t, 3) + coeffs[1](
        nKnots - 1,
        k) * pow(t - times[nKnots - 1], 3) + coeffs[2](nKnots - 1, k) * (t - times[nKnots - 1]) + coeffs[3](
        nKnots - 1,
        k) * (times[nKnots - 1 + 1] - t);
    float vel =
      -3 * coeffs[0](nKnots - 1, k) * pow(times[nKnots - 1 + 1] - t, 2) + 3 * coeffs[1](
        nKnots - 1,
        k) * pow(t - times[nKnots - 1], 2) + coeffs[2](nKnots - 1, k) - coeffs[3](
        nKnots - 1,
        k);
    float acc =
      6 * coeffs[0](nKnots - 1, k) * (times[nKnots - 1 + 1] - t) - 6 * coeffs[1](
        nKnots - 1,
        k) * (t - times[nKnots - 1]);
    times_pos.push_back(make_pair(t, pos));
    times_vel.push_back(make_pair(t, vel));
    times_acc.push_back(make_pair(t, acc));
    //gp << "plot" << gp.file1d(times_pos) << "with lines title 'cubic spline " << k << " position.'" << endl;
    gp << "replot" << gp.file1d(times_vel) << "with lines title 'cubic spline " << k << " velocity.'" << endl;
    //gp << "replot" << gp.file1d(times_acc) << "with lines title 'cubic spline " << k << " acceleration.'" << endl;
    cin.get();
  }
  cout << "pos plot: \n";
  for (int i = 0; i < times_pos.size(); ++i) {
    cout << times_pos[i].second << endl;
  }
  cout << "vel plot: \n";
  for (int i = 0; i < times_vel.size(); ++i) {
    cout << times_vel[i].second << endl;
  }
  cout << "acc plot: \n";
  for (int i = 0; i < times_acc.size(); ++i) {
    cout << times_acc[i].second << endl;
  }
}

void
CubicSpline::getTrajectories(vector<vector<float> >& traj,
  vector<float>& trajTime, const unsigned& derivative)
{
  trajTime.clear();
  float totalTime = 0;
  for (int i = 0; i < knots.size(); ++i)
    totalTime += knots[i];
  float tTime = 0;
  while (tTime + stepSize <= totalTime + 1e-6) {
    tTime += stepSize;
    trajTime.push_back(tTime);
  }

  //cout << "totalTime: " << totalTime << endl;
  //cout << "tTime: " << tTime << endl;

  vector<float> times;
  times.resize(nKnots + 1);
  times[0] = 0.f;
  for (int i = 1; i < times.size(); ++i) {
    times[i] = times[i - 1] + knots[i - 1];
  }

  traj.clear();
  traj.resize(splineDim);
  for (int i = 0; i < trajTime.size(); i++) {
    float t = trajTime[i];
    int knot = 0;
    for (int j = 1; j < times.size(); ++j) {
      if (t <= times[j] + stepSize / 2) {
        knot = j - 1;
        break;
      }
    }
    for (int k = 0; k < splineDim; ++k) {
      if (derivative == 0) {
        float pos =
          coeffs[0](knot, k) * pow(times[knot + 1] - t, 3) + coeffs[1](knot, k) * pow(
            t - times[knot],
            3) + coeffs[2](knot, k) * (t - times[knot]) + coeffs[3](knot, k) * (times[knot + 1] - t);
        traj[k].push_back(pos);
      } else if (derivative == 1) {
        float vel =
          -3 * coeffs[0](knot, k) * pow(times[knot + 1] - t, 2) + 3 * coeffs[1](
            knot,
            k) * pow(t - times[knot], 2) + coeffs[2](knot, k) - coeffs[3](
            knot,
            k);
        traj[k].push_back(vel);
      } else if (derivative == 2) {
        float acc =
          6 * coeffs[0](knot, k) * (times[knot + 1] - t) - 6 * coeffs[1](
            knot,
            k) * (t - times[knot]);
        traj[k].push_back(acc);
      }
    }
  }

  //cout << "size: " << traj[0].size() << endl;
  //for (int i = 0; i < trajTime.size(); ++i)
  //{
  //	cout << "Time[" << i << "]: " << trajTime[i] << endl;
  //}
}
