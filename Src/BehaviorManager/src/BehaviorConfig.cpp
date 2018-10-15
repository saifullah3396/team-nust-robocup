/**
 * @file BehaviorManager/src/BehaviorConfig.cpp
 *
 * This file implements the classes BehaviorConfig and BConfigException
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 10 Sep 2017
 */

#include "BehaviorManager/include/BehaviorConfig.h"

BConfigException::BConfigException(
  BehaviorConfig* config, 
  const string& message,
  const bool& bSysMsg, 
  const BConfigExceptionType& type) throw () :
  TNRSException(message, bSysMsg),
  config(config),
  type(type)
{
}

string BConfigException::getExcPrefix()
{ 
  return 
    string("Exception caught in behavior config \n\tType: ") + 
    EnumToString(config->baseType) + 
    string(", Id: ") + 
    DataUtils::varToString(config->id) + 
    string(";\t"); 
}
