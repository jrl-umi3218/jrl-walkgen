/*
 * Copyright 2006, 2007, 2008, 2009, 2010, 
 *
 * Florent Lamiraux
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

/** 
\mainpage 
\section sec_intro Introduction

This library implements a series of algorithms generating CoM, ZMP and feet
trajectories when given a set of foot-prints. In some cases, when a robot
model provides a specialized inverse kinematics some algorithms can generate 
articular values.

\section walkGenJrl_sec_approach Approach implemented

This code assumes that the mass distribution of your humanoid robot is 
centered around the waist. This is very important as all the algorithms
implemented uses the single point mass model. If the masses of your robot
legs and arms are too important to be ignored it is very likely that
none of the algorithms will give a feasible pair of CoM-ZMP trajectories.

The first available algorithm is the preview algorithm \ref Kajita2003 and \ref Kajita2005. 
This algorithm  is real-time if you do not change the foot-prints inside the preview 
window. 
If you do want to perform this modification you should see \ref Morisawa2007.
This come at the expense of a possible delay in the step execution, if the modifcations
are too important. On the other hand this is one of the fastest solution to modify
the futur in the stack of foot-steps.

Two variants similar to the preview control but including inequalities have been introduced
by \ref Wieber2006 and \ref Dimitrov2009. The former takes 28 ms to be computed whereas the
second introduce a new solver able to solve the problem in less than 2 ms.
The solver named PLDP is included inside the library.

A new algorithm able to take a reference velocity of the CoM is also provided.
Its description can be found in \ref Herdt2010. This problem is currently solved
using ql written by Pr. Schittkowski (\ref Schittkowski2007 ). We thank him for
letting us distribute his code in our LGPL code.

\section References
\anchor Kajita2003
S. Kajita and F. Kanehiro and K. Kaneko and K. Fujiwara and K. Harada and K. Yokoi and H. Hirukawa,
"Biped Walking Pattern Generation by using Preview Control of Zero-Moment Point",
International Conference on Robotics And Automation,  Taipei Taiwan, 2003

\anchor Kajita2005
S. Kajita,
Omsha, 
"Humanoid Robot",
2005,(In Japanese) ISBN4-274-20058-2, 2005

\anchor Verrelst2006
B. Verrelst and K. Yokoi and O. Stasse and H. Arisumi and B. Vanderborght,
"Mobility of Humanoid Robots: Stepping over Large Obstacles Dynamically",
International Conference on Mechatronics and Automation

\anchor Morisawa2007
M. Morisawa and K. Harada and S. Kajita and S. Nakaoka and K. Fujiwara and F. Kanehiro and K. Kaneko and H. Hirukawa,
"Experimentation of Humanoid Walking Allowing Immediate Modification of Foot Place Based on Analytical Solution",
IEEE Int. Conf. on Robotics and Automation,
3989--3994, 2007

\anchor Wieber2006
P.-B. Wieber,
"Trajectory Free Linear Model Predictive Control for Stable Walking in the Presence of Strong Perturbations"
IEEE/RAS Int. Conf. on Humanoid Robots
137-142, 2006

\anchor Dimitrov2009
D. Dimitrov, P.-B. Wieber, O. Stasse, H.J. Ferreau and H. Diedam,
"An Optimized Linear Model Predictive Control Solver for Online Walking Motion Generation",
IEEE International Conference on Robotics and Automation (ICRA)
pp. 1171--1176, 2009

\anchor Herdt2010
A. Herdt, D. Holger, P.B. Wieber, D. Dimitrov, K. Mombaur and D. Moritz
"Online Walking Motion Generation with Automatic Foot Step Placement",
Advanced Robotics
Vol. 24 Issue 5-6, pp. 719--737

\anchor Schittkowski2007 
K. Schittkowski (2007): QL: A Fortran code for convex
quadratic programming - user's guide, Report, Department
of Computer Science, University of Bayreuth

   \defgroup pgjrl JRL Walking Pattern Generator Library (WalkGenJRL)
   @{

   This library is intended to implement walking pattern generator algorithms for humanoid robots.

*/

/**
   \defgroup walkGenJrl_steppingover Stepping Over

   This group implements the stepping over algorithm as presented
   by Bjorn Verrelst \ref Verrelst2006 .
*/

/**
   \defgroup walkGenJrl_previewcontrol Preview Control

   This group implements the preview control algorithm for the cart-model
   as presented by Kajita in \ref Kajita2003 .

*/

/**
   \defgroup walkGenJrl_geometry Geometry
   
   This group implements some basic geometrical tools for the Pattern Generator.
   @{
*/


/**
   @}
*/

/**
   \defgroup walkGenJrl_Interface Interface for the PatternGeneratorJRL

   This group reinforces the independance between the internal
   structure of the Walking Pattern Generator and external
   algorithms. The main class is \a PatternGeneratorInterface
   which allow to handle very simply the WPG.
*/

/**
   @}
*/

/**
   \defgroup pginterfaces  Interfaces to WalkGenJRL
   This group shows how to interface the WalkGenJRL library to three kinds of applications:
   \li An OpenHRP plugin, to run in real-time inside an HRP-2 humanoid robot,
   \li A console program, for quick but yet complete testing of the pattern generator,
   \li An OpenGL interface.

*/
