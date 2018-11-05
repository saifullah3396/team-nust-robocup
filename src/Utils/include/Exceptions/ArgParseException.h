/**
 * @file Utils/include/ArgParseException.h
 *
 * This file defines the class ArgParseException
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include "TNRSException.h"

#pragma once

DEFINE_ENUM_WITH_STRING_CONVERSIONS(
	APExceptionType, 
	(EXC_NO_ARG_SPECIFIED)
	(EXC_INVALID_ARG_VALUE)
)

/**
 * @class ArgParseException
 * @brief A class for generating exceptions based on invalid options
 *   or values given to the arguments parser
 */
class ArgParseException : public TNRSException
{
public:
  /**
   * Constructs an exception with an explanatory message
   * @param message explanatory message
   * @param bSysMsg true if the system message (from strerror(errno))
   *   should be postfixed to the user provided message
   * @param type Argument parser exception type
   */
  ArgParseException(
		const string& message,
		const bool& bSysMsg, 
		const APExceptionType& type) throw () :
		TNRSException(message, bSysMsg),
		type(type)
  {
  }

  /**
   * Destructor
   */
  ~ArgParseException() throw () {}
  
  string getExcPrefix()
		{ return "Exception caught while parsing an argument: "; }

private:
	APExceptionType type;
};
