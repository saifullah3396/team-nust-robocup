/**
 * @file Utils/include/PolySpline.h
 *
 * This file implements the class PolySpline
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Aug 2017  
 */

#include "Utils/include/PolySpline.h"
#include "Utils/include/GnuPlotter.h"

template <typename Scalar>
PolySpline<Scalar>::PolySpline(
  const unsigned& degree,
  const unsigned& dim,
  const Matrix<Scalar, Dynamic, Dynamic>& controlPoints,
  const Matrix<Scalar, Dynamic, 1>& knots, 
  const Scalar& stepSize, 
  const Matrix<Scalar, Dynamic, Dynamic>& boundaryConds) :
  Spline<Scalar>(
    degree,
    dim, 
    controlPoints, 
    knots,
    stepSize,  
    PIECE_WISE_POLY),
  boundaryConds(boundaryConds)
{
}

template <typename Scalar>
PolySpline<Scalar>::PolySpline(
  const unsigned& degree,
  const unsigned& dim,
  const Matrix<Scalar, Dynamic, Dynamic>& controlPoints,
  const Scalar& splineTime, 
  const Scalar& stepSize, 
  const Matrix<Scalar, Dynamic, Dynamic>& boundaryConds) :
  Spline<Scalar>(
    degree,
    dim, 
    controlPoints, 
    splineTime,
    stepSize,  
    PIECE_WISE_POLY),
  boundaryConds(boundaryConds)
{
}

template <typename Scalar>
PolySpline<Scalar>::PolySpline(const string& filePath) : 
  Spline<Scalar>(filePath)
{
  PolySpline<Scalar>::splineFromXml(filePath);
}

template <typename Scalar>
void
PolySpline<Scalar>::validateParameters()
{
  ASSERT(this->controlPoints.rows() == (this->knots.size() + 1));
  ASSERT(this->controlPoints.cols() == this->dim);
  ASSERT(boundaryConds.rows() == 2);
  ASSERT(boundaryConds.cols() == this->dim);
}

template <typename Scalar>
void
PolySpline<Scalar>::evaluateSpline(vector<vector<Scalar> >& spline,
  vector<Scalar>& splineTime, const unsigned& derivative)
{
  splineTime.clear();
  Scalar totalTime = 0;
  for (int i = 0; i < this->knots.size(); ++i)
    totalTime += this->knots[i];
  Scalar tTime = 0;
  while (tTime + this->stepSize <= totalTime + 1e-6) {
    tTime += this->stepSize;
    splineTime.push_back(tTime);
  }

  //cout << "totalTime: " << totalTime << endl;
  //cout << "tTime: " << tTime << endl;

  vector<Scalar> times;
  times.resize(this->nKnots + 1);
  times[0] = 0.f;
  for (int i = 1; i < times.size(); ++i) {
    times[i] = times[i - 1] + this->knots[i - 1];
  }

  spline.clear();
  spline.resize(this->dim);
  for (int i = 0; i < splineTime.size(); i++) {
    Scalar t = splineTime[i];
    int knot = 0;
    for (int j = 1; j < times.size(); ++j) {
      if (t <= times[j] + this->stepSize / 2) {
        knot = j - 1;
        break;
      }
    }
    for (int k = 0; k < this->dim; ++k) {
      if (derivative == 0) {
        Scalar pos =
          this->coeffs[0](knot, k) * pow(times[knot + 1] - t, 3) + this->coeffs[1](knot, k) * pow(
            t - times[knot],
            3) + this->coeffs[2](knot, k) * (t - times[knot]) + this->coeffs[3](knot, k) * (times[knot + 1] - t);
        spline[k].push_back(pos);
      } else if (derivative == 1) {
        Scalar vel =
          -3 * this->coeffs[0](knot, k) * pow(times[knot + 1] - t, 2) + 3 * this->coeffs[1](
            knot,
            k) * pow(t - times[knot], 2) + this->coeffs[2](knot, k) - this->coeffs[3](
            knot,
            k);
        spline[k].push_back(vel);
      } else if (derivative == 2) {
        Scalar acc =
          6 * this->coeffs[0](knot, k) * (times[knot + 1] - t) - 6 * this->coeffs[1](
            knot,
            k) * (t - times[knot]);
        spline[k].push_back(acc);
      }
    }
  }

  //cout << "size: " << spline[0].size() << endl;
  //for (int i = 0; i < splineTime.size(); ++i)
  //{
  //	cout << "Time[" << i << "]: " << splineTime[i] << endl;
  //}
}

template <typename Scalar>
void
PolySpline<Scalar>::evaluateCoeffs(const Matrix<Scalar, Dynamic, 1>& knots)
{
  ASSERT(knots.size() == this->nKnots);
  this->knots = knots;
  genParams();
}

template <typename Scalar>
void
PolySpline<Scalar>::plotSpline(const unsigned& nInnerPoints, const Scalar& startTime)
{
  ASSERT(this->coeffs[0].size() != 0);
  Gnuplot gp;
  Scalar stepSize;
  stepSize = 1.0 / nInnerPoints;
  vector<Scalar> times;
  vector < pair<Scalar, Scalar> > times_pos;
  vector < pair<Scalar, Scalar> > times_vel;
  vector < pair<Scalar, Scalar> > times_acc;
  times.resize(this->nKnots + 1);
  times[0] = startTime;
  for (int i = 1; i < times.size(); ++i) {
    times[i] = times[i - 1] + this->knots[i - 1];
  }

  gp << "set xrange [0:20]\nset yrange [0:20]\n";
  gp << "plot" << gp.file1d(times_pos) << "with lines title 'Cubic splines'" << endl;
  for (int k = 0; k < this->dim; ++k) {
    gp << "set terminal wxt " << k << endl;
    times_pos.clear();
    times_vel.clear();
    times_acc.clear();
    for (int i = 0; i < this->nKnots; ++i) {
      for (Scalar t = times[i]; t < times[i + 1]; t = t + this->stepSize) {
        Scalar pos =
          this->coeffs[0](i, k) * pow(times[i + 1] - t, 3) + this->coeffs[1](i, k) * pow(
            t - times[i],
            3) + this->coeffs[2](i, k) * (t - times[i]) + this->coeffs[3](i, k) * (times[i + 1] - t);
        Scalar vel =
          -3 * this->coeffs[0](i, k) * pow(times[i + 1] - t, 2) + 3 * this->coeffs[1](i, k) * pow(
            t - times[i],
            2) + this->coeffs[2](i, k) - this->coeffs[3](i, k);
        Scalar acc = 6 * this->coeffs[0](i, k) * (times[i + 1] - t) - 6 * this->coeffs[1](
          i,
          k) * (t - times[i]);
        times_pos.push_back(make_pair(t, pos));
        times_vel.push_back(make_pair(t, vel));
        times_acc.push_back(make_pair(t, acc));
      }
    }
    Scalar t = times.back();
    Scalar pos =
      this->coeffs[0](this->nKnots - 1, k) * pow(times[this->nKnots - 1 + 1] - t, 3) + this->coeffs[1](
        this->nKnots - 1,
        k) * pow(t - times[this->nKnots - 1], 3) + this->coeffs[2](this->nKnots - 1, k) * (t - times[this->nKnots - 1]) + this->coeffs[3](
        this->nKnots - 1,
        k) * (times[this->nKnots - 1 + 1] - t);
    Scalar vel =
      -3 * this->coeffs[0](this->nKnots - 1, k) * pow(times[this->nKnots - 1 + 1] - t, 2) + 3 * this->coeffs[1](
        this->nKnots - 1,
        k) * pow(t - times[this->nKnots - 1], 2) + this->coeffs[2](this->nKnots - 1, k) - this->coeffs[3](
        this->nKnots - 1,
        k);
    Scalar acc =
      6 * this->coeffs[0](this->nKnots - 1, k) * (times[this->nKnots - 1 + 1] - t) - 6 * this->coeffs[1](
        this->nKnots - 1,
        k) * (t - times[this->nKnots - 1]);
    times_pos.push_back(make_pair(t, pos));
    times_vel.push_back(make_pair(t, vel));
    times_acc.push_back(make_pair(t, acc));
    gp << "plot" << gp.file1d(times_pos) << "with lines title 'cubic spline " << k << " position.'" << endl;
    cout <<  this->controlPoints << endl;
    for (int i = 0; i < this->controlPoints.rows(); ++i) {
      cout << "Point: " << times[i] << " " << this->controlPoints(i, k) << endl;
      gp << "replot \"<echo '" << times[i] << " " << this->controlPoints(i, k) << "'\" with points ls " << 1 << "ps " << 1.0 << endl;
    }
    //gp << "replot" << gp.file1d(times_vel) << "with lines title 'cubic spline " << k << " velocity.'" << endl;
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

template <typename Scalar>
void PolySpline<Scalar>::splineFromXml(const string& filePath)
{
  fstream fs;
  fs.open(filePath);
  boost::property_tree::ptree pt;
  read_xml(fs, pt);
  fs.close();
  try {
    BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, pt.get_child("spline")) {
      //! Parsing attributes
      if (v.first == "boundary_conditions") { //! parsing coefficients
        size_t i = 0;
        BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, v.second) {
          if (v.first == "<xmlattr>") {
            boundaryType = v.second.get_child("type").data();
            if (boundaryType != "clamped" && boundaryType != "natural")
              throw SplineException<Scalar>(
                this, 
                "Invalid type specified for boundary conditions.", 
                false, 
                SplineExceptionType::EXC_INVALID_XML);
            boundaryConds.resize(2, this->dim);
          } else {
            if (i >= boundaryConds.rows()) {
              throw SplineException<Scalar>(
                this, 
                "Boundary conditions matrix row size mismatch.", 
                false, 
                SplineExceptionType::EXC_INVALID_XML);
            }
            size_t j = 0;
            BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, v.second) {
              if (j >= this->dim) {
                throw SplineException<Scalar>(
                  this, 
                  "Boundary conditions matrix column size mismatch.", 
                  false, 
                  SplineExceptionType::EXC_INVALID_XML
                );
              }
              boundaryConds(i, j) = boost::lexical_cast<Scalar>(v.second.data());
              ++j;
            }
            ++i;
          }
        }
      }
    }
  } catch (SplineException<Scalar> &e) {
    cout << e.what();
  } catch (exception &e) {
    cout << "Failed to parse spline." << endl;
    cout << e.what() << endl;
  }
  validateParameters();
}

template class PolySpline<float>;
template class PolySpline<double>;
