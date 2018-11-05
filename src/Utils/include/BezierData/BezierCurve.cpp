/**
 * @file Utils/src/BezierCurve.cpp
 *
 * This file implements the class BezierCurve
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Aug 2017
 */

#pragma once

#include <boost/math/special_functions/binomial.hpp>
#include "Utils/include/BezierCurve.h"

BezierCurveException::BezierCurveException(
  BezierCurve* curve,
  const string& message,
  const bool& bSysMsg,
  const BezierCurveExceptionType& type) throw () :
  TNRSException(message, bSysMsg),
  type(type)
{
}

BezierCurve::BezierCurve(
  const unsigned& degree,
  const unsigned& dim,
  const MatrixXf& controlPoints,
  const float& stepSize)
{
  
}

BezierCurve::BezierCurve(const string& filePath)
{
  
}
