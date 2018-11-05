/**
 * @file CommModule/include/CommMessage.h
 *
 * This file defines the class CommMessage
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <string>
#include "CommModule/include/CommMsgTypes.h"

using namespace std;

/**
 * @class CommMessage
 * @brief Defines a message that can be sent through a CommRequest
 */
class CommMessage
{
public:
  /**
   * Constructor
   *
   * @param msg: Message string
   * @param type: Message type
   */
  CommMessage(
    const string& msg,
    const CommMsgTypes& type) :
    msg(msg),
    type(type)
  {
  }

  /**
   * Returns the message
   *
   * @return string
   */
  string getMessage() const
  {
    return msg;
  }
  
  /**
   * Returns the message type
   *
   * @return CommMsgTypes
   */
  CommMsgTypes getType() const
  {
    return type;
  }

private:
  //! Message 
  string msg;
  
  //! Message type
  CommMsgTypes type;
};
