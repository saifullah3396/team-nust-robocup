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
enum class CommMsgTypes : unsigned int
{
  HEART_BEAT = 1,
  MEMORY_HEADER,
  MEMORY_DATA,
  THREADS_HEADER,
  THREADS_SELECT_LINES,
  REQUEST_MSGS = THREADS_SELECT_LINES,
  LOG_TEXT,
  IMAGE,
  FOOTSTEPS,
  PF_STATES,
  OCCUPANCY_GRID,
  NIHA_COGNITION_HEADER,
  NIHA_COGNITION_DATA,
};
