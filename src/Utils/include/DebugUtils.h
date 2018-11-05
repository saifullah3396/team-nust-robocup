/**
 * @file Utils/include/DebugUtils.h
 *
 * This file defines the macro for handling debugging.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 07 Feb 2017
 */

#ifndef _DEBUG_UTILS_H_
#define _DEBUG_UTILS_H_

#include <assert.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include "Utils/include/ConfigManager.h"
#include "Utils/include/VariadicMacros.h"
#include "Eigen/Dense"

using namespace std;
using namespace Eigen;

extern int USE_LOGGED_IMAGES;
extern int SAVE_IMAGES;
extern int PROJECT_GIELD;
extern int SIMULATION;
extern string ROBOT_NAME;

using namespace std;
using namespace std::chrono;

#define ASSERT(condition) \
{ \
  assert(condition); \
}

#define ASSERT_MSG(condition, msg) \
{ \
  assert(condition && #msg); \
}

class DebugUtils
{
public:
  DebugUtils() {}
  ~DebugUtils() { mainLog.close(); }
  static void logData(const string& data);
  template<typename Scalar>
  static void logData(const Matrix<Scalar, Dynamic, Dynamic>& mat);
  template<typename Scalar>
  static void logData(const Matrix<Scalar, 4, 4>& mat);
  static fstream mainLog;
};

#define LOG_INFO(x) DebugUtils::mainLog << "[INFO]: "; DebugUtils::logData(x);
#define ERROR(x) DebugUtils::mainLog << "[ERROR]: "; DebugUtils::logData(x);

#endif //!_DEBUG_UTILS_H_
