/*
 * Copyright 2016,
 *
 * Maximilien Naveau
 * Olivier Stasse
 *
 * JRL, CNRS/AIST
 *
 * This file is part of jrl-walkgen.
 * jrl-walkgen is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * jrl-walkgen is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with jrl-walkgen.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the
 *  Joint Japanese-French Robotics Laboratory (JRL)
 */
/*! \file PinocchioRobot.hh
  \brief This object defines a humanoid robot model based on the PinocchioRobot
frame work */

#ifndef PinocchioRobot_HH
#define PinocchioRobot_HH

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/se3.hpp"

namespace PatternGeneratorJRL {
struct PinocchioRobotFoot_t {
  pinocchio::JointIndex associatedAnkle;
  double soleDepth;  // z axis
  double soleWidth;  // y axis
  double soleHeight; // x axis
  Eigen::Vector3d anklePosition;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  inline PinocchioRobotFoot_t()
      : associatedAnkle(0), soleDepth(0.0), soleWidth(0.0), soleHeight(0.0),
        anklePosition(0.0, 0.0, 0.0) {}
};
typedef PinocchioRobotFoot_t PRFoot;

namespace pinocchio_robot {
const int RPY_SIZE = 6;
const int QUATERNION_SIZE = 7;
} // namespace pinocchio_robot

class PinocchioRobot {
public:
  // overload the new[] eigen operator
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Constructor and Destructor
  PinocchioRobot();
  virtual ~PinocchioRobot();

  /// Compute RNEA algorithm
  /// This is a front end for computeInverseDynamics(q,v,a).
  void computeInverseDynamics();

  /// Compute RNEA algorithm from
  /// \param[in] q: \f$q=[r,\theta,\hat{q}]\f$ with \f$r\f$ the position
  /// of the free floating base (usually the waist), \f$theta\f$ the
  /// free floating
  /// orientation in RPY format, $\hat{q}$ the motor angles position.
  void computeInverseDynamics(Eigen::VectorXd &q, Eigen::VectorXd &v,
                              Eigen::VectorXd &a);

  /// Compute the geometry of the robot.
  void computeForwardKinematics();

  void RPYToSpatialFreeFlyer(Eigen::Vector3d &rpy, Eigen::Vector3d &drpy,
                             Eigen::Vector3d &ddrpy, Eigen::Quaterniond &quat,
                             Eigen::Vector3d &omega, Eigen::Vector3d &domega);

  /// \brief ComputeSpecializedInverseKinematics :
  /// compute POSITION (not velocity) of the joints from end effector pose
  /// This is the implementation of the analitycal inverse kinematic extracted
  /// from the book of Kajita
  /// Authors Shuuji Kajita ; Hirohisa Hirukawa ; Kensuke Harada ;
  /// Kazuhito Yokoi
  /// ISBN 9782287877162 ; 9782287877155
  /// It needs a specific distribution of the joint to work.
  /// a test at the initialization of the class is [should be] done to check
  /// if everything is correct
  /// param root joint index, i.e. the waist or the shoulder
  /// param end joint index, i.e, the wrist or ankle indexes
  /// param root joint homogenous matrix position,
  /// param root joint homogenous matrix index,
  /// param 6D vector output, filled with zeros if the robot is not compatible
  ///
  virtual bool
  ComputeSpecializedInverseKinematics(const pinocchio::JointIndex &jointRoot,
                                      const pinocchio::JointIndex &jointEnd,
                                      const Eigen::Matrix4d &jointRootPosition,
                                      const Eigen::Matrix4d &jointEndPosition,
                                      Eigen::VectorXd &q);

  ///
  /// \brief testArmsInverseKinematics :
  /// test if the robot arms has the good joint
  /// configuration to use the analitical inverse geometry
  /// to be overloaded if the user wants another inverse kinematics
  /// RY-RX-RZ-RY-RZ-RY
  /// \return
  ///
  virtual bool testArmsInverseKinematics();
  ///
  /// \brief testLegsInverseKinematics :
  /// test if the robot legs has the good joint
  /// configuration to use the analitical inverse geometry
  /// to be overloaded if the user wants another inverse kinematics
  /// Two modes are available:
  /// FF-RZ-RX-RY-RY-RY-RX
  /// FF-RX-RZ-RY-RY-RY-RX
  /// \return
  ///
  virtual bool testLegsInverseKinematics();
  ///
  /// \brief testOneModefOfLegInverseKinematics :
  /// test if the robot legs has one good joint
  /// configuration to use the analitical inverse geometry
  /// to be overloaded if the user wants another inverse kinematics
  /// \return
  ///
  virtual bool testOneModeOfLegsInverseKinematics(
      std::vector<std::string> &leftLegJointNames,
      std::vector<std::string> &rightLegJointNames);

  ///
  /// \brief initializeInverseKinematics :
  /// initialize the internal data for the inverse kinematic e.g. the femur
  /// length
  /// \return
  ///
  virtual void initializeLegsInverseKinematics();

public:
  /// tools :
  std::vector<pinocchio::JointIndex>
  jointsBetween(pinocchio::JointIndex first, pinocchio::JointIndex second);
  std::vector<pinocchio::JointIndex> fromRootToIt(pinocchio::JointIndex it);

private:
  // needed for the inverse geometry (ComputeSpecializedInverseKinematics)
  void getWaistFootKinematics(const Eigen::Matrix4d &jointRootPosition,
                              const Eigen::Matrix4d &jointEndPosition,
                              Eigen::VectorXd &q, Eigen::Vector3d &Dt) const;
  double ComputeXmax(double &Z);
  void getShoulderWristKinematics(const Eigen::Matrix4d &jointRootPosition,
                                  const Eigen::Matrix4d &jointEndPosition,
                                  Eigen::VectorXd &q, int side);
  void DetectAutomaticallyShoulders();
  void DetectAutomaticallyOneShoulder(pinocchio::JointIndex aWrist,
                                      pinocchio::JointIndex &aShoulder);

  /*! \brief Computes the size of the free flyer/root robot
    loads by the urdf.
    Set the value of m_PinoFreeFlyerSize.
  */
  void ComputeRootSize();

public:
  /// Getters
  /// ///////
  inline pinocchio::Data *Data() { return m_robotData; }
  inline pinocchio::Data *DataInInitialePose() {
    return m_robotDataInInitialePose;
  }
  inline pinocchio::Model *Model() { return m_robotModel; }

  inline PRFoot *leftFoot() { return &m_leftFoot; }
  inline PRFoot *rightFoot() { return &m_rightFoot; }

  inline pinocchio::JointIndex leftWrist() { return m_leftWrist; }
  inline pinocchio::JointIndex rightWrist() { return m_rightWrist; }

  inline pinocchio::JointIndex chest() { return m_chest; }
  inline pinocchio::JointIndex waist() { return m_waist; }

  inline double mass() { return m_mass; }

  inline pinocchio::JointModelVector &getActuatedJoints() {
    return m_robotModel->joints;
  }

  inline Eigen::VectorXd &currentPinoConfiguration() { return m_qpino; }
  inline Eigen::VectorXd &currentRPYConfiguration() { return m_qrpy; }
  inline Eigen::VectorXd &currentPinoVelocity() { return m_vpino; }
  inline Eigen::VectorXd &currentPinoAcceleration() { return m_apino; }
  inline Eigen::VectorXd &currentRPYVelocity() { return m_vrpy; }
  inline Eigen::VectorXd &currentRPYAcceleration() { return m_arpy; }
  inline Eigen::VectorXd &currentTau() { return m_tau; }

  inline unsigned numberDof() { return m_robotModel->nq; }

  inline unsigned numberVelDof() { return m_robotModel->nv; }

  inline void zeroMomentumPoint(Eigen::Vector3d &zmp) {
    m_externalForces = m_robotData->liMi[1].act(m_robotData->f[1]);
    m_f = m_externalForces.linear();
    m_n = m_externalForces.angular();
    zmp(0) = -m_n(1) / m_f(2);
    zmp(1) = m_n(0) / m_f(2);
    zmp(2) = 0.0; // by default
  }

  inline void positionCenterOfMass(Eigen::Vector3d &com) {
    m_com = m_robotData->com[0];
    com(0) = m_com(0);
    com(1) = m_com(1);
    com(2) = m_com(2);
  }
  inline void CenterOfMass(Eigen::Vector3d &com, Eigen::Vector3d &dcom,
                           Eigen::Vector3d &ddcom) {
    m_com = m_robotData->acom[0];
    ddcom(0) = m_com(0);
    ddcom(1) = m_com(1);
    ddcom(2) = m_com(2);

    m_com = m_robotData->vcom[0];
    dcom(0) = m_com(0);
    dcom(1) = m_com(1);
    dcom(2) = m_com(2);

    m_com = m_robotData->com[0];
    com(0) = m_com(0);
    com(1) = m_com(1);
    com(2) = m_com(2);
  }

  /// SETTERS
  /// ///////
  void currentPinoConfiguration(Eigen::VectorXd &conf);
  void currentRPYConfiguration(Eigen::VectorXd &);
  inline void currentPinoVelocity(Eigen::VectorXd &vel) { m_vpino = vel; }
  inline void currentPinoAcceleration(Eigen::VectorXd &acc) { m_apino = acc; }
  inline void currentRPYVelocity(Eigen::VectorXd &vel) { m_vrpy = vel; }
  inline void currentRPYAcceleration(Eigen::VectorXd &acc) { m_arpy = acc; }

  inline pinocchio::JointIndex getFreeFlyerSize() const {
    return m_PinoFreeFlyerSize;
  }

  inline pinocchio::JointIndex getFreeFlyerVelSize() const {
    return m_PinoFreeFlyerVelSize;
  }

  /// Initialization functions
  /// ////////////////////////
  inline bool isInitialized() {
    return m_boolModel && m_boolData && m_boolLeftFoot && m_boolRightFoot;
  }
  bool checkModel(pinocchio::Model *robotModel);
  bool initializeRobotModelAndData(pinocchio::Model *robotModel,
                                   pinocchio::Data *robotData);
  bool initializeLeftFoot(PRFoot leftFoot);
  bool initializeRightFoot(PRFoot rightFoot);

  const std::string &getName() const;
  /// Attributes
  /// //////////
private:
  pinocchio::Model *m_robotModel;
  pinocchio::Data *m_robotDataInInitialePose; // internal variable
  pinocchio::Data *m_robotData;
  PRFoot m_leftFoot, m_rightFoot;
  double m_mass;
  pinocchio::JointIndex m_chest, m_waist, m_leftShoulder, m_rightShoulder;
  pinocchio::JointIndex m_leftWrist, m_rightWrist;

  /// Fields member to store  configurations, velocity and acceleration
  /// @{
  /// Configuration SE(3) position + quaternion + NbDofs
  Eigen::VectorXd m_qpino;
  /// Configuration SE(3) position + RPY + NbDofs
  Eigen::VectorXd m_qrpy;
  /// Velocity se(3) + NbDofs
  Eigen::VectorXd m_vpino;
  /// Velocity se(3) + NbDofs
  Eigen::VectorXd m_vrpy;
  /// Acceleration acc + NbDofs
  Eigen::VectorXd m_apino;
  Eigen::VectorXd m_arpy;
  // @}

  // tmp variables
  Eigen::Quaterniond m_quat;
  Eigen::Matrix3d m_rot;
  pinocchio::Force m_externalForces; // external forces and torques
  Eigen::VectorXd m_tau;             // external forces and torques
  Eigen::Vector3d m_f, m_n;          // external forces and torques
  Eigen::Vector3d m_com;             // multibody CoM
  Eigen::Matrix3d m_S;
  Eigen::Vector3d m_rpy, m_drpy, m_ddrpy, m_omega, m_domega;

  // Variables extracted form the urdf used for the analitycal inverse
  // kinematic
  bool m_isLegInverseKinematic;
  unsigned int m_modeLegInverseKinematic;
  bool m_isArmInverseKinematic;

  // length between the waist and the hip
  Eigen::Vector3d m_leftDt, m_rightDt;
  double m_femurLength;
  double m_tibiaLengthZ;
  double m_tibiaLengthY;

  bool m_boolModel;
  bool m_boolData;
  bool m_boolLeftFoot;
  bool m_boolRightFoot;

  /// \brief Size of the free flyer configuration space.
  pinocchio::JointIndex m_PinoFreeFlyerSize;

  /// \brief Size of the free flyer velocity space.
  pinocchio::JointIndex m_PinoFreeFlyerVelSize;

}; // PinocchioRobot
} // namespace PatternGeneratorJRL
#endif // PinocchioRobot_HH
