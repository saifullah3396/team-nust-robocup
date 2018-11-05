/**
 * @file Utils/src/DebugUtils.cpp
 *
 * This file defines the macro for handling debugging.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 07 Feb 2017
 */

#include "Utils/include/DebugUtils.h"

fstream DebugUtils::mainLog;

void DebugUtils::logData(const string& data) {
  if (mainLog.is_open())
    mainLog << data << endl;
  else
    mainLog.open(
      ConfigManager::getLogsDirPath() + "Output.txt",
      std::ofstream::out | std::ofstream::trunc
    );
}

template<typename Scalar>
void DebugUtils::logData(const Matrix<Scalar, Dynamic, Dynamic>& mat)
{
  if (mainLog.is_open())
    mainLog << "\n" << mat << endl;
  else
    mainLog.open(
      ConfigManager::getLogsDirPath() + "Output.txt",
      std::ofstream::out | std::ofstream::trunc
    );
}

template<typename Scalar>
void DebugUtils::logData(const Matrix<Scalar, 4, 4>& mat)
{
  if (mainLog.is_open())
    mainLog << "\n" << mat << endl;
  else
    mainLog.open(
      ConfigManager::getLogsDirPath() + "Output.txt",
      std::ofstream::out | std::ofstream::trunc
    );
}

template void DebugUtils::logData<float>(const Matrix<float, 4, 4>& mat);
template void DebugUtils::logData<double>(const Matrix<double, 4, 4>& mat);
