/**
 * @file TNRSBase/src/Multiplexer.cpp
 *
 * This file implements the class that deals with the input memory
 * variables. It defines multiplexed I/O callback functions for input
 * variable access.
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017
 */

#include "TNRSBase/include/Multiplexer.h"

int
Multiplexer::addInput(ThreadSafeVariable* input, const bool& select)
{
  inputs.push_back(input);
  if (select) nextSelectLine = inputs.size() - 1;
  return inputs.size() - 1;
}

ThreadSafeVariable*
Multiplexer::getThreadSafeVariable()
{
  updateSelectLine();
  if (selectLine < inputs.size()) return inputs[selectLine];
  else {
    if (inputs.size() != 0) return inputs[0];
    else return NULL;
  }
}

void
Multiplexer::setNextSelectLine(ThreadSafeVariable* selectVariable)
{
  for (size_t i = inputs.size() - 1; i >= 0; ++i) {
    if (inputs[i] == selectVariable) {
      nextSelectLine = i;
      return;
    }
  }
  nextSelectLine = 0;
}

void
Multiplexer::getStringHeader(string& out)
{
  out += inputs.at(0)->getVariableName();
}
