/**
 * @file CommModule/CommMsgTypes.h
 *
 * This file declares the enumeration CommMsgTypes
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

/**
 * Enumeration for all the communication msg types we can send to the
 * debugger
 *
 * @enum CommMsgTypes
 */
enum CommMsgTypes
{
  MSG_HEARTBEAT = 1,
  MSG_MEMORY_DATA,
  MSG_MEMORY_HEADER,
  MSG_MEMORY_DEBUG, //Deprecated
  MSG_THREADS_HEADER,
  MSG_THREADS_SELECT_LINES,
  REQUEST_MSGS = MSG_THREADS_SELECT_LINES,
  MSG_LOG_TEXT,
  MSG_IMAGE,
  MSG_FOOTSTEPS,
  MSG_PF_STATES
};
