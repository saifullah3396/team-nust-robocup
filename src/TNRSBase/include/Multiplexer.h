/**
 * @file TNRSBase/include/Multiplexer.h
 *
 * This file declares a class that deals with the input memory
 * variables. It defines multiplexed I/O callback functions for input
 * variable access.
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017
 */

#pragma once

#include <vector>
#include "Utils/include/ThreadSafeVariable.h"

using namespace std;

/**
 * @class Multiplexer
 * @brief A class that deals with the input memory variables. It defines
 *   multiplexed I/O callback functions for input variable access.
 */
class Multiplexer
{
public:
  /**
   * Constructor
   */
  Multiplexer() :
    selectLine(0), nextSelectLine(0)
  {
  }

  /**
   * Destructor
   */
  virtual
  ~Multiplexer()
  {
  }

  /**
   * Add a variable to inputs
   *
   * @param input variable to be added
   * @param select whether the variable should be selected next
   * @return int
   */
  int
  addInput(ThreadSafeVariable* input, const bool& select = false);

  /**
   * Finds and sets the selectLine associated with the selectVariable
   * to nextSelectLine
   *
   * @param selectVariable variable to be selected currently
   * @return void
   */
  void
  setNextSelectLine(ThreadSafeVariable* selectVariable);

  /**
   * Returns the threadsafe input memory variable at the specified index
   *
   * @param index input variable index
   * @return Pointer to ThreadSafeVariable
   */
  ThreadSafeVariable*
  getInputFromIndex(const unsigned& index)
  {
    if (index < inputs.size()) return inputs[index];
    else return NULL;
  }

  /**
   * Increments the input memory variable name to the output string
   *
   * @param out output string
   * @return void
   */
  virtual void
  getStringHeader(string& out);

  /**
   * Returns the current select line
   *
   * @return int
   */
  int
  getSelectLine()
  {
    return selectLine;
  }

  /**
   * Sets the current select line
   *
   * @param selectLine selectLine to set
   * @return void
   */
  void
  setSelectLine(const int& selectLine)
  {
    this->nextSelectLine = selectLine;
  }

  /**
   * Returns the threadsafe input memory variable
   *
   * @return Pointer to ThreadSafeVariable
   */
  ThreadSafeVariable*
  getThreadSafeVariable();

protected:
  /**
   * Assigns the next select line to current select line
   *
   * @return void
   */
  void
  updateSelectLine()
  {
    selectLine = nextSelectLine;
  }

  //! Vector of thread safe input memory variables
  vector<ThreadSafeVariable*> inputs;

  //! Current select line
  int selectLine;

  //! Next select line
  int nextSelectLine;
};
