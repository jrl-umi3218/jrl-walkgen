/*
 * Copyright 2007, 2008, 2009, 2010,
 *
 * Mehdi    Benallegue
 * Andrei   Herdt
 * Francois Keith
 * Olivier  Stasse
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
/*! \file PGTypes.h
  \brief Defines basic types for the Humanoid Walking Pattern Generator.
*/
#ifndef _PATTERN_GENERATOR_TYPES_H_
#define _PATTERN_GENERATOR_TYPES_H_

// For Windows compatibility.
#if defined(WIN32)
#ifdef jrl_walkgen_EXPORTS
#define WALK_GEN_JRL_EXPORT __declspec(dllexport)
#else
#define WALK_GEN_JRL_EXPORT __declspec(dllimport)
#endif
#else
#define WALK_GEN_JRL_EXPORT
#endif
#include <fstream>
#include <iostream>

#include <Eigen/Dense>
#include <vector>

namespace PatternGeneratorJRL {
struct COMState_s;

/// Structure to store the COM position computed by the preview control.
struct WALK_GEN_JRL_EXPORT COMPosition_s {
  double x[3], y[3];
  double z[3];
  double yaw;   // aka theta
  double pitch; // aka omega
  double roll;  // aka hip

  struct COMPosition_s &operator=(const COMState_s &aCS);
};


inline std::ostream &operator<<(std::ostream &os, const COMPosition_s &aCp) {
  for (size_t i = 0; i < 3; ++i) {
    os << "x[" << i << "] " << aCp.x[i] << " y[" << i << "] " << aCp.y[i]
       << " z[" << i << "] " << aCp.z[i] << std::endl;
  }
  os << "yaw " << aCp.yaw << " pitch " << aCp.pitch << " roll " << aCp.roll;
  return os;
}

typedef struct COMPosition_s COMPosition;
typedef struct COMPosition_s WaistState;

/// Structure to store the COM state computed by the preview control.
struct WALK_GEN_JRL_EXPORT COMState_s {
  double x[3], y[3], z[3];
  double yaw[3];   // aka theta
  double pitch[3]; // aka omega
  double roll[3];  // aka hip

  struct COMState_s &operator=(const COMPosition_s &aCS);

  void reset();

  COMState_s();

  friend std::ostream &operator<<(std::ostream &os,
                                  const struct COMState_s &acs);
};

typedef struct COMState_s COMState;

/** Structure to store each foot position when the user is specifying
    a sequence of relative positions. */
struct RelativeFootPosition_s {
  double sx, sy, sz, theta;
  double SStime;
  double DStime;
  int stepType; // 1:normal walking 2:one step before obstacle
                // 3:first leg over obstacle 4:second leg over obstacle
  // 5:one step after obstacle 6 :stepping stair
  double DeviationHipHeight;
  RelativeFootPosition_s();
};
typedef struct RelativeFootPosition_s RelativeFootPosition;

inline std::ostream &operator<<(std::ostream &os,
                                const RelativeFootPosition_s &rfp) {
  os << "sx " << rfp.sx << " sy " << rfp.sy << " sz " << rfp.sz << " theta "
     << rfp.theta << std::endl;
  os << "SStime " << rfp.SStime << " DStime " << rfp.DStime << " stepType "
     << rfp.stepType << " DeviationHipHeight " << rfp.DeviationHipHeight;
  return os;
}

/** Structure to store each of the ZMP value, with a given
    direction at a certain time. */
struct ZMPPosition_s {
  double px, py, pz;
  double theta; // For COM
  double time;
  int stepType; // 1:normal walking 2:one step before obstacle
                // 3:first leg over obstacle 4:second leg over
  // obstacle 5:one step after obstacle
  // +10 if double support phase
  // *(-1) if right foot stance else left foot stance
};
typedef struct ZMPPosition_s ZMPPosition;

inline std::ostream &operator<<(std::ostream &os, const ZMPPosition_s &zmp) {
  os << "px " << zmp.px << " py " << zmp.pz << " pz " << zmp.pz << " theta "
     << zmp.theta << std::endl;
  os << "time " << zmp.time << " stepType " << zmp.stepType;
  return os;
}

/// Structure to store the absolute foot position.
struct FootAbsolutePosition_t {
  /*! px, py in meters, theta in DEGREES. */
  double x, y, z, theta, omega, omega2;
  /*! Speed of the foot. */
  double dx, dy, dz, dtheta, domega, domega2;
  /*! Acceleration of the foot. */
  double ddx, ddy, ddz, ddtheta, ddomega, ddomega2;
  /*! Jerk of the foot. */
  double dddx, dddy, dddz, dddtheta, dddomega, dddomega2;
  /*! Time at which this position should be reached. */
  double time;
  /*! 1:normal walking 2:one step before obstacle
    3:first leg over obstacle 4:second leg over obstacle
    5:one step after  obstacle
    +10 if double support phase
    (-1) if support foot  */
  int stepType;
};
typedef struct FootAbsolutePosition_t FootAbsolutePosition;

inline std::ostream &operator<<(std::ostream &os,
                                const FootAbsolutePosition &fap) {
  os << "x " << fap.x << " y " << fap.y << " z " << fap.z << " theta "
     << fap.theta << " omega " << fap.omega << " omega2 " << fap.omega2
     << std::endl;
  os << "dx " << fap.dx << " dy " << fap.dy << " dz " << fap.dz << " dtheta "
     << fap.dtheta << " domega " << fap.domega << " domega2 " << fap.domega2
     << std::endl;
  os << "ddx " << fap.ddx << " ddy " << fap.ddy << " ddz " << fap.ddz
     << " ddtheta " << fap.ddtheta << " ddomega " << fap.ddomega << " ddomega2 "
     << fap.ddomega2 << std::endl;
  os << "time " << fap.time << " stepType " << fap.stepType;
  return os;
}

/// Structure to store the absolute foot position.
struct HandAbsolutePosition_t {
  /*! x, y, z in meters, theta in DEGREES. */
  double x, y, z, theta, omega, omega2;
  /*! Speed of the foot. */
  double dx, dy, dz, dtheta, domega, domega2;
  /*! Acceleration of the foot. */
  double ddx, ddy, ddz, ddtheta, ddomega, ddomega2;
  /*! Jerk of the hand. */
  double dddx, dddy, dddz, dddtheta, dddomega, dddomega2;
  /*! Time at which this position should be reached. */
  double time;
  /*! -1 : contact
   *   1 : no contact
   */
  int stepType;
};
typedef struct HandAbsolutePosition_t HandAbsolutePosition;

inline std::ostream &operator<<(std::ostream &os,
                                const HandAbsolutePosition &hap) {
  os << "x " << hap.x << " y " << hap.y << " z " << hap.z << " theta "
     << hap.theta << " omega " << hap.omega << " omega2 " << hap.omega2
     << std::endl;
  os << "dx " << hap.dx << " dy " << hap.dy << " dz " << hap.dz << " dtheta "
     << hap.dtheta << " domega " << hap.domega << " domega2 " << hap.domega2
     << std::endl;
  os << "ddx " << hap.ddx << " ddy " << hap.ddy << " ddz " << hap.ddz
     << " ddtheta " << hap.ddtheta << " ddomega " << hap.ddomega << " ddomega2 "
     << hap.ddomega2 << std::endl;
  os << "time " << hap.time << " stepType " << hap.stepType;
  return os;
}

// Linear constraint.
struct LinearConstraintInequality_s {
  Eigen::MatrixXd A;
  Eigen::MatrixXd B;
  Eigen::VectorXd Center;
  std::vector<int> SimilarConstraints;
  double StartingTime, EndingTime;
};
typedef struct LinearConstraintInequality_s LinearConstraintInequality_t;

/// Linear constraints with variable feet placement.
struct LinearConstraintInequalityFreeFeet_s {
  Eigen::MatrixXd D;
  Eigen::MatrixXd Dc;
  int StepNumber;
};
typedef struct LinearConstraintInequalityFreeFeet_s
    LinearConstraintInequalityFreeFeet_t;

// State of the feet on the ground
struct SupportFeet_s {
  double x, y, theta, StartTime;
  int SupportFoot;
};
typedef struct SupportFeet_s SupportFeet_t;

inline std::ostream &operator<<(std::ostream &os, const SupportFeet_s &sf) {
  os << "x " << sf.x << " y " << sf.y << " theta " << sf.theta << std::endl;
  os << " StartTime " << sf.StartTime << " SupportFoot " << sf.SupportFoot;
  return os;
}

/// Structure to store the absolute reference.
struct ReferenceAbsoluteVelocity_t {
  /*! m/sec or degrees/sec */
  double x, y, z, dYaw;

  /*! reference values for the whole preview window */
  Eigen::VectorXd RefVectorX;
  Eigen::VectorXd RefVectorY;
  Eigen::VectorXd RefVectorTheta;
};
typedef struct ReferenceAbsoluteVelocity_t ReferenceAbsoluteVelocity;

inline std::ostream &operator<<(std::ostream &os,
                                const ReferenceAbsoluteVelocity_t &rav) {
  os << "x " << rav.x << " y " << rav.y << " z " << rav.z << " dYaw "
     << rav.dYaw;
  return os;
}

/// Structure to model a circle (e.g : a stricly convex obstable)
struct Circle_t {
  double x_0;
  double y_0;
  double r;
  double margin;
};
typedef struct Circle_t Circle;

inline std::ostream &operator<<(std::ostream &os, const Circle_t &circle) {
  os << "x_0 " << circle.x_0 << " y_0 " << circle.y_0 << " R " << circle.r;
  return os;
}

struct ControlLoopOneStepArgs {
  Eigen::VectorXd CurrentConfiguration;
  Eigen::VectorXd CurrentVelocity;
  Eigen::VectorXd CurrentAcceleration;
  Eigen::VectorXd ZMPTarget;
  COMState finalCOMState;
  FootAbsolutePosition LeftFootPosition;
  FootAbsolutePosition RightFootPosition;
  Eigen::VectorXd Momentum;
};

} // namespace PatternGeneratorJRL
#endif
