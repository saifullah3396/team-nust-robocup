/**
 * @file Utils/include/Spline.h
 *
 * This file implements the class Spline
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Aug 2017  
 */

#include "Utils/include/Spline.h"

template <typename Scalar>
SplineException<Scalar>::SplineException(
  Spline<Scalar>* spline,
  const string& message,
  const bool& bSysMsg,
  const SplineExceptionType& type) throw () :
  TNRSException(message, bSysMsg),
  type(type)
{
}

template <typename Scalar>
Spline<Scalar>::Spline(
  const unsigned& degree, 
  const unsigned& dim,
  const Matrix<Scalar, Dynamic, Dynamic>& controlPoints,
  const Matrix<Scalar, Dynamic, 1>& knots, 
  const Scalar& stepSize,
  const unsigned& type) :
  degree(degree),
  dim(dim), 
  controlPoints(controlPoints),
  knots(knots), 
  stepSize(stepSize),
  nKnots(knots.size())
{
}

template <typename Scalar>
Spline<Scalar>::Spline(
  const unsigned& degree, 
  const unsigned& dim,
  const Matrix<Scalar, Dynamic, Dynamic>& controlPoints,
  const Scalar& splineTime, 
  const Scalar& stepSize,
  const unsigned& type) :
  degree(degree),
  dim(dim),
  controlPoints(controlPoints),
  stepSize(stepSize),
  nKnots(knots.size())
{
}

template <typename Scalar>
Spline<Scalar>::Spline(const string& filePath) : stepSize(0.05)
{
  Spline<Scalar>::splineFromXml(filePath);
}

template <typename Scalar>
void Spline<Scalar>::splineFromXml(const string& filePath)
{
  fstream fs;
  fs.open(filePath);
  boost::property_tree::ptree pt;
  read_xml(fs, pt);
  fs.close();
  try {
    BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, pt.get_child("spline")) {
      //! Parsing attributes
      if (v.first == "<xmlattr>") {
        string typeStr = v.second.get_child("type").data();
        if (typeStr == "poly") type = PIECE_WISE_POLY;
        else if (typeStr == "bspline") type = BASIS;
        else type = -1;
        dim = boost::lexical_cast<unsigned>(v.second.get_child("dim").data()); 
        degree = boost::lexical_cast<unsigned>(v.second.get_child("degree").data()); // order is degree + 1
      } else {
        if (type == -1) {
          throw SplineException<Scalar>(
              this, 
              "Spline type attribute not found.", 
              false, 
              SplineExceptionType::EXC_INVALID_XML
          );
        }
        if (v.first == "knots") { //! Parsing knots sequence 
          size_t i = 0;
          BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, v.second) {
            if (v.first == "<xmlattr>") {
              unsigned size = boost::lexical_cast<unsigned>(v.second.get_child("size").data());
              if (size)  {
                nKnots = size;
                knots.resize(nKnots);
              } else {
                throw SplineException<Scalar>(
                  this, 
                  "Empty knot sequence specified", 
                  false, 
                  SplineExceptionType::EXC_INVALID_XML);
              }
            } else {
              if (i < knots.size()) {
                knots[i] = boost::lexical_cast<Scalar>(v.second.data());
              }
              ++i;
            }
          }
          if (i != knots.size()) {
            throw SplineException<Scalar>(
              this, 
              "Knots vector/value size mismatch.", 
              false, 
              SplineExceptionType::EXC_INVALID_XML);
          }
        } else if (v.first == "control_points") { //! parsing coefficients
          size_t j = 0;
          BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, v.second) {
            if (v.first == "<xmlattr>") {
              unsigned size = boost::lexical_cast<unsigned>(v.second.get_child("size").data());
              if (size > 0)  {
                controlPoints.resize(size, dim);
              } else {
                 throw SplineException<Scalar>(
                  this, 
                  "Empty control points vector specified", 
                  false, 
                  SplineExceptionType::EXC_INVALID_XML);
              }
            } else {
              if (j >= dim) {
                throw SplineException<Scalar>(
                  this, 
                  "Control points matrix column size mismatch.", 
                  false, 
                  SplineExceptionType::EXC_INVALID_XML
                );
              }
              size_t i = 0;
              BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, v.second) {
                if (i >= controlPoints.rows()) {
                  throw SplineException<Scalar>(
                    this, 
                    "Control points matrix row size mismatch.", 
                    false, 
                    SplineExceptionType::EXC_INVALID_XML);
                }
                controlPoints(i,j) = boost::lexical_cast<Scalar>(v.second.data());
                ++i;
              }
              ++j;
            }
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
}

template class SplineException<float>;
template class SplineException<double>;
template class Spline<float>;
template class Spline<double>;


