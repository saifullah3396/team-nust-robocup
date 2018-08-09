/**
 * @file Utils/include/TNRSException.h
 *
 * This file defines the class TNRSException
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#include <exception>
#include <iostream>
#include <string>
#include "../EnumToString.h"

using namespace std;

#pragma once

/**
 * @class TNRSException
 * @brief A base class for generating different kinds of 
 * 	child exceptions within the softwarea.
 */
class TNRSException : public exception
{
public:
  /**
   * Constructs an exception with an explanatory message
   *
   * @param message explanatory message
   * @param bSysMsg true if the system message (from strerror(errno))
   *   should be postfixed to the user provided message
   */
  TNRSException(const string& message, const bool& bSysMsg) throw () :
		message(message)
	{
	}

  /**
   * Virtual destructor
   */
  virtual ~TNRSException() throw () {}

  /**
   * Returns a pointer to the (constant) error description
   *
   * @return a pointer to a const char
   */
  virtual const char*
  what() throw ()
  {
		auto prefix = "[TNRSException]: " + getExcPrefix();
		prefix.append(this->message);
		this->message = prefix;
		this->message += "\n";
    return this->message.c_str();
  }
  
  virtual string getExcPrefix() { return ""; }

protected:
  //! Error message.
  string message;
};
