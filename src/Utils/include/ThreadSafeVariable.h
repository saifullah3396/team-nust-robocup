/**
 * @file Utils/include/ThreadSafeVariable.h
 *
 * This file declares the class ThreadSafeVariable.
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017
 */

#pragma once

#include <string>

using namespace std;

/**
 * @class ThreadSafeVariable
 * @brief A class for defining variables with threadsafe access.
 */
class ThreadSafeVariable
{
public:
  /**
   * Definition of a memory variable.
   *
   * @param string variableName: Name of the variable.
   */
  ThreadSafeVariable(string variableName);

  /**
   * Virtual destructor for redifinition in derived classes.
   */
  virtual
  ~ThreadSafeVariable();

  /**
   * Pure virtual function for setting the value of the variable.
   * Must be implemented in derived classes.
   *
   * @param void* ptrValue: A pointer to the input value.
   */
  virtual void
  setValueFromPtr(void * ptrValue) = 0;

  /**
   * Pure virtual function for getting the value of the variable.
   * Must be implemented in derived classes.
   *
   * @param void* ptrValue: A pointer to the output value.
   */
  virtual void
  getValueIntoPtr(void * ptrValue) = 0;

  /**
   * Virtual function for converting the value of the variable to
   * output string.
   *
   * @param string& out: Reference to the output string.
   */
  virtual void
  getString(string & out)
  {
  }

  /**
   * Returns the name of the variable.
   *
   * @return string
   */
  string
  getVariableName()
  {
    return variableName;
  }

protected:

  /**
   * Initiates a threadsafe variable.
   */
  void
  initVariable()
  {
    pthread_mutex_init(&accessMutex, NULL);
    conditionInitialised = false;
  }

  //! Name of the variable.
  string variableName;

  //! Mutex for thread safe access to variable.
  pthread_mutex_t accessMutex;

  //! Wait condition for thread safe access to variable.
  pthread_cond_t waitCond;

  //! Wait condition status.
  bool conditionInitialised;
};
