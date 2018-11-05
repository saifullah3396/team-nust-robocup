/**
 * @file TNRSBase/include/BaseModuleHandler.h
 *
 * This file declares and implements the class BaseModuleHandler
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 07 Nov 2017
 */

#pragma once

#include "TNRSBase/include/BaseModule.h"

using namespace Utils;

/**
 * @class BaseModuleHandler
 * @brief A class that provides child BaseModule vector and its handling
 *   utilities
 */
class BaseModuleHandler
{
public:
  /**
   * Constructor
   */
  BaseModuleHandler()
  {
  }

  /**
   * Virtual destruct
   */
  virtual
  ~BaseModuleHandler()
  {
  }

  /**
   * Returns the vector of child BaseModules
   *
   * @return a vector of BaseModulePtr
   */
  vector<BaseModulePtr>
  getChildModules()
  {
    return childModules;
  }
protected:
  /**
   * Starts the threads of all child BaseModules
   *
   * @return void
   */
  void
  start()
  {
    for (size_t i = 0; i < childModules.size(); ++i)
      childModules[i]->startModule();
  }

  /**
   * Suspends the threads of all child BaseModules
   *
   * @return void
   */
  void
  suspend()
  {
    for (size_t i = 0; i < childModules.size(); ++i)
      childModules[i]->suspendMe();
  }

  /**
   * Resumes the threads of all child BaseModules
   *
   * @return void
   */
  void
  resume()
  {
    for (size_t i = 0; i < childModules.size(); ++i)
      childModules[i]->resumeMe();
  }

  //! A vector of child BaseModules.
  vector<BaseModulePtr> childModules;
};
