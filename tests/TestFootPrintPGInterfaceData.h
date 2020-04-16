/*
 * Copyright 2005, 2006, 2007, 2008, 2009, 2010,
 *
 * Francois Keith
 * Olivier Stasse
 *
 * JRL, CNRS/AIST
 *
 * This file is part of walkGenJrl.
 * walkGenJrl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * walkGenJrl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with walkGenJrl.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the
 *  Joint Japanese-French Robotics Laboratory (JRL)
 */
#ifndef _TEST_FOOT_PRINT_PG_INTERFACE_H_
#define _TEST_FOOT_PRINT_PG_INTERFACE_H_

#include <string>

enum Profiles_t {
  PROFIL_PB_FLORENT,                               //  0
  PROFIL_STEPPING_OVER,                            //  1
  PROFIL_SHORT_STRAIGHT_WALKING,                   //  2
  PROFIL_SHORT_STRAIGHT_WALKING_ONE_STAGE,         //  3
  PROFIL_CURVED_WALKING_PBW2,                      //  4
  PROFIL_KINEOWORKS,                               //  5
  PROFIL_STRAIGHT_WALKING,                         //  6
  PROFIL_ANALYTICAL_SHORT_STRAIGHT_WALKING,        //  7
  PROFIL_CURVED_WALKING_PBW,                       //  8
  PROFIL_STRAIGHT_WALKING_DIMITROV,                //  9
  PROFIL_CURVED_WALKING_DIMITROV,                  // 10
  PROFIL_TURN_90D,                                 // 11
  PROFIL_TURNING_ON_THE_CIRCLE,                    // 12
  PROFIL_TURNING_ON_THE_CIRCLE_TOWARDS_THE_CENTER, // 13
  PROFIL_ANALYTICAL_ONLINE_WALKING,                // 14
  PROFIL_ONLINE_WALKING,                           // 15
  PROFIL_SIMU_ONLINE_WALKING,                      // 16
  PROFIL_HERDT,                                    // 17
  PROFIL_HERDT_ONLINE                              // 18
};

enum InitialPoses_t {
  HALF_SITTING_2003,
  TELEOPERATION_2008,
  HWPG_v1,
  HALF_SITTING_2008,
  INTERACTION_2008,
  MODEL_BUILDING_1,
  MODEL_BUILDING_2
};

#define NBOFPREDEFONLINEFOOTSTEPS 11

extern std::string ProfilesNames[19];

extern double InitialPoses[7][40];

extern double OnLineFootSteps[NBOFPREDEFONLINEFOOTSTEPS][3];

#endif /* _TEST_FOOT_PRINT_PG_INTERFACE_H_ */
