/**
 * @file TNRSBase/include/SharedMemory.h
 *
 * This file declares the class SharedMemory
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017
 */

#pragma once

#include "TNRSBase/include/MemoryConnector.h"
#include "TNRSBase/include/MemoryDefinition.h"
#include "Utils/include/Variable.h"

using namespace GlobalMemory;

/**
 * @class SharedMemory
 * @brief The class that holds all the shared data from different threads
 */
class SharedMemory
{
public:
  /**
   * Constructor
   */
  SharedMemory()
  {
  }

  /**
   * Destructor
   */
  ~SharedMemory()
  {
    for (size_t i = 0; i < variables.size(); ++i)
      delete variables[i];
  }

  /**
   * Initializes the memory blackboard. All the memory variables are
   * declared/defined in this function.
   *
   * @return void
   */
  void
  init();

  /**
   * Virtual function for getting all the variable values to a comma
   * separated output string
   *
   * @param out output string
   * @return void
   */
  virtual void
  getString(string& out);

  /**
   * Virtual function for getting all the variable names to a comma
   * separated output string
   *
   * @param out output string
   * @return void
   */
  virtual void
  getStringHeader(string & out);

  /**
   * Returns a memory variable by matching its name
   *
   * @param name variable name
   * @return Pointer to ThreadSafeVariable
   */
  ThreadSafeVariable*
  findVariableFromName(const string& name);

  /**
   * Returns the vector of all the memory variables
   *
   * @return vector<ThreadSafeVariable*>
   */
  vector<ThreadSafeVariable*>
  getVariables()
  {
    return variables;
  }

  //! Vector of pointers to threadsafe memory variables
  vector<ThreadSafeVariable*> variables;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<SharedMemory> SharedMemoryPtr;
