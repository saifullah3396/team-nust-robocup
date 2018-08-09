/**
 * @file LocalizationModule/include/FieldLandmarkIds.h
 *
 * This file declares the Enumeration Ids for all field landmarks.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 06 Sep 2017
 */

#pragma once

/**
 * Enumeration for the field landmarks.
 *
 * @enum FieldLandmarkIds
 */
enum FieldLandmarkIds
{
  FL_LT_GOALPOST = 0,
  FL_RT_GOALPOST,
  FL_LB_GOALPOST,
  FL_RB_GOALPOST,
  FL_GOAL_POSTS,
  FL_L_MIDDLE_T = 0,
  FL_R_MIDDLE_T,
  FL_LT_BOX_T,
  FL_RT_BOX_T,
  FL_LB_BOX_T,
  FL_RB_BOX_T,
  FL_T_CORNERS,
  FL_LT_CORNER = 0,
  FL_RT_CORNER,
  FL_LB_CORNER,
  FL_RB_CORNER,
  FL_LT_BOX_L,
  FL_RT_BOX_L,
  FL_LB_BOX_L,
  FL_LR_BOX_L,
  FL_L_CORNERS,
  FL_CIRCLE = 0,
  FL_CIRCLES,
  FL_PENALTY_MARK_T = 0,
  FL_PENALTY_MARK_B,
  FL_PENALTY_MARKS,
  NUM_LANDMARKS = FL_GOAL_POSTS + FL_T_CORNERS + FL_L_CORNERS + FL_CIRCLES + FL_PENALTY_MARKS
};

/**
 * Enumeration for the type of field landmarks.
 *
 * @enum FieldLandmarkTypes
 */
enum FieldLandmarkTypes
{
  FL_TYPE_GOAL_POST = 0,
  FL_TYPE_LINES,
  FL_TYPE_T_CORNER,
  FL_TYPE_L_CORNER,
  FL_TYPE_CIRCLE,
  FL_TYPE_PENALTY_MARK,
  NUM_LANDMARK_TYPES
};
