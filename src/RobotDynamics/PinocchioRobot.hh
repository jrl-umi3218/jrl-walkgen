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

#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/multibody/model.hpp"
#include <jrl/mal/matrixabstractlayer.hh>
#include <vector>
namespace PatternGeneratorJRL
{
  struct PinocchioRobotFoot_t{
    se3::JointIndex associatedAnkle ;
    double soleDepth ; // z axis
    double soleWidth ; // y axis
    double soleHeight ;// x axis
    vector3d anklePosition ;
    unsigned JointNb ;
    std::vector<unsigned> JointId ;
  };
  typedef PinocchioRobotFoot_t PRFoot ;

  struct PinocchioRobotHand_t{
    se3::JointIndex associatedWrist ;
    vector3d center ;
    vector3d thumbAxis ;
    vector3d foreFingerAxis ;
    vector3d palmNormal ;
    unsigned JointNb ;
    std::vector<unsigned> JointId ;
  };
  typedef PinocchioRobotHand_t PRHand ;



  class PinocchioRobot
  {
  public:
    /// Constructor and Destructor
    PinocchioRobot(se3::Model *aModel, se3::Data *aData);
    void InitializePinocchioRobot(se3::Model * aModel, se3::Data * aData);


    /// Functions computing kinematics and dynamics
    void computeInverseDynamics();
    void computeInverseDynamics(MAL_VECTOR_TYPE(double) & q,
                                MAL_VECTOR_TYPE(double) & v,
                                MAL_VECTOR_TYPE(double) & a);

    void computeForwardKinematics();
    void computeForwardKinematics(MAL_VECTOR_TYPE(double) & q);

    // compute POSITION (not velocity) of the joints from end effector pose
    bool ComputeSpecializedInverseKinematics(
        const se3::JointIndex &jointRoot,
        const se3::JointIndex &jointEnd,
        const MAL_S4x4_MATRIX_TYPE(double) & jointRootPosition,
        const MAL_S4x4_MATRIX_TYPE(double) & jointEndPosition,
        MAL_VECTOR_TYPE(double) &q);

  public :
    /// tools :
    std::vector<se3::JointIndex> jointsBetween
    ( se3::JointIndex first, se3::JointIndex second);
    std::vector<se3::JointIndex> fromRootToIt (se3::JointIndex it);

  private :
    // need for the inverse geometry (ComputeSpecializedInverseKinematics)
    void getWaistFootKinematics(const matrix4d & jointRootPosition,
                                const matrix4d & jointEndPosition,
                                vectorN &q,
                                vector3d Dt);
    double ComputeXmax(double & Z);
    void getShoulderWristKinematics(const matrix4d & jointRootPosition,
                                    const matrix4d & jointEndPosition,
                                    vectorN &q,
                                    int side);
    void DetectAutomaticallyShoulders();
    void DetectAutomaticallyOneShoulder(PRHand aHand,
                                        se3::JointIndex & aShoulder);




  public :
    /// Getters
    /// ///////
    inline se3::Data * Data()
    {return m_robotData;}
    inline se3::Data * DataInInitialePose()
    {return &m_robotDataInInitialePose;}
    inline se3::Model * Model()
    {return m_robotModel;}

    inline PRFoot * leftFoot()
    {return &m_leftFoot;}
    inline PRFoot * rightFoot()
    {return &m_rightFoot;}

    inline PRHand * leftHand()
    {return &m_leftHand;}
    inline PRHand * rightHand()
    {return &m_rightHand;}

    inline se3::JointIndex chest()
    {return m_chest;}
    inline se3::JointIndex waist()
    {return m_waist;}

    inline double mass()
    {return m_mass;}

    inline se3::JointModelVector & getActuatedJoints()
    {return m_robotModel->joints;}

    inline MAL_VECTOR_TYPE(double) currentConfiguration()
    {return m_qmal;}
    inline MAL_VECTOR_TYPE(double) currentVelocity()
    {return m_vmal;}
    inline MAL_VECTOR_TYPE(double) currentAcceleration()
    {return m_amal;}

    inline unsigned numberDof()
    {return m_robotModel->nv;}

    inline void zeroMomentumPoint(MAL_S3_VECTOR_TYPE(double) & zmp)
    {
      se3::Force externalForces = m_robotData->oMi[1].act(m_robotData->f[1]) ;
      m_f = externalForces.linear() ;
      m_n = externalForces.angular() ;
      zmp(0) = -m_n(1)/m_f(2) ;
      zmp(1) =  m_n(0)/m_f(2) ;
      zmp(2) = 0.0 ; // by default
    }

    inline void positionCenterOfMass(MAL_S3_VECTOR_TYPE(double) & com)
    {
      m_com = m_robotData->com[0] ;
      com(0) = m_com(0) ;
      com(1) = m_com(1) ;
      com(2) = m_com(2) ;
    }

    /// SETTERS
    /// ///////
    inline void currentConfiguration(MAL_VECTOR_TYPE(double) conf)
    {m_qmal=conf;}
    inline void currentVelocity(MAL_VECTOR_TYPE(double) vel)
    {m_vmal=vel;}
    inline void currentAcceleration(MAL_VECTOR_TYPE(double) acc)
    {m_amal=acc;}

  private:
    se3::Model * m_robotModel ;
    se3::Data m_robotDataInInitialePose ; // internal variable
    se3::Data * m_robotData ;
    PRFoot m_leftFoot , m_rightFoot ;
    PRHand m_leftHand , m_rightHand ;
    double m_mass ;
    se3::JointIndex m_chest, m_waist, m_leftShoulder, m_rightShoulder ;

    MAL_VECTOR_TYPE(double) m_qmal ;
    MAL_VECTOR_TYPE(double) m_vmal ;
    MAL_VECTOR_TYPE(double) m_amal ;
    Eigen::VectorXd m_q ;
    Eigen::VectorXd m_v ;
    Eigen::VectorXd m_a ;

    // tmp variables
    Eigen::Quaternion<double> m_quat ;
    Eigen::Vector3d m_f,m_n; // external forces and torques
    Eigen::Vector3d m_com; // multibody CoM
  };
}
#endif // PinocchioRobot_HH
