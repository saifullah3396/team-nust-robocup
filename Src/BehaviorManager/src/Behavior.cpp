/**
 * @file BehaviorManager/src/Behavior.cpp
 *
 * This file implements the classes Behavior and BehaviorException
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 10 Sep 2017
 */

#include "BehaviorManager/include/Behavior.h"

BehaviorException::BehaviorException(
  Behavior* behavior,
  const string& message,
  const bool& bSysMsg,
  const BehaviorExceptionType& type) throw () :
  TNRSException(message, bSysMsg),
  name(behavior->getName()),
  type(type)
{
}
