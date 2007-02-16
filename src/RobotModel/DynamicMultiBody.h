/* Computation of the dynamic aspect for a robot.
   
   This class will load the description of a robot from a VRML file
   following the OpenHRP syntax. Using ForwardVelocity it is then
   possible specifying the angular velocity and the angular value 
   to get the absolute position, and absolute velocity of each 
   body separetly. Heavy rewriting from the original source
   of Adrien and Jean-Remy. 

   This implantation is an updated based on a mixture between 
   the code provided by Jean-Remy and Adrien.
  
   Copyright (c) 2005-2006, 
   @author Olivier Stasse, Ramzi Sellouati, Jean-Remy Chardonnet, Adrien Escande, Abderrahmane Kheddar
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the <ORGANIZATION> nor the names of its contributors 
   may be used to endorse or promote products derived from this software without specific prior written permission.
   
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
   AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
   OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _DYNAMIC_MULTI_BODY_H_
#define _DYNAMIC_MULTI_BODY_H_
#include <vector>

#include <Joint.h>
#include <MatrixAbstractLayer.h>
#include <MultiBody.h>
#include <DynamicBody.h>
#include <jrlDynamicRobot.h>

using namespace::std;
namespace PatternGeneratorJRL 
{

  /** @ingroup forwardynamics
      This class simulates the dynamic of a multibody robot.
  */
  class DynamicMultiBody : public  CjrlDynamicRobot, 
    public MultiBody 
    
  {
  private:

    /**  Label of the root. */
    int labelTheRoot;
    
    /**  List of bodies with dynamical properties */
    vector<DynamicBody> listOfBodies;
    
    /** Array to convert Joint Id from VRL file to Body array index. */
    vector<int> ConvertIDINVRMLToBodyID;
    
    /** Update body parameters from the bodyinfo list of 
      joints and the internal list of bodies. */
    void UpdateBodyParametersFromJoint(int cID, int lD, int LiaisonForFatherJoint);
    
    /** The skew matrix related to the CoM position. */
    MAL_S3x3_MATRIX(,double) SkewCoM;

    /** Weighted CoM position. */
    MAL_S3_VECTOR(,double) positionCoMPondere;
    
    /** Splitted inertial matrices. */
    MAL_MATRIX(m_MHStarB,double);
    MAL_MATRIX(m_MHStarLeftFoot,double); 
    MAL_MATRIX(m_MHStarRightFoot,double);
    MAL_MATRIX(m_MHFree,double);
    
    /** Inversed Jacobian for the left and right foot. */
    MAL_MATRIX(m_ILeftJacobian,double);
    MAL_MATRIX(m_IRightJacobian,double);
    MAL_MATRIX(m_ERBFI_Left,double);
    MAL_MATRIX(m_ERBFI_Right,double);

    /*! \name Members related to the momentum 
      @{
    */
    /** \brief Linear momentum vector */
    MAL_S3_VECTOR(,double) m_P;
    
    /** \brief Derivative of the linear momentum */
    MAL_S3_VECTOR(,double) m_dP;

    /** \brief Angular momentum vector. */
    MAL_S3_VECTOR(,double) m_L;

    /** \brief Derivative of the angular momentum */
    MAL_S3_VECTOR(,double) m_dL;
    
    /** \brief Previous Linear momentum vector */
    MAL_S3_VECTOR(,double) m_Prev_P;

    /** \brief Previous Angular momentum vector. */
    MAL_S3_VECTOR(,double) m_Prev_L;
        
    
    /** @} */
    /** \brief Starting the computation. */
    bool m_FirstTime;

    /** \brief Store the root of the Joints tree. */
    Joint * m_RootOfTheJointsTree;

    /** \brief Velocity of the center of Mass */
    MAL_S3_VECTOR(,double) m_VelocityCenterOfMass;

    /** \brief Acceleration of the center of Mass */
    MAL_S3_VECTOR(,double) m_AccelerationCenterOfMass;

    /** \brief Vector to store the global current configuration 

    \f$ q \f$ 
      The current format is :
      \f[ q = \\
      \left( 
      \begin{matrix}
      {\bf c} \\
      {\bf o} \\
      q_0 \\
      ... \\
      q_{n-1}\\ 
      \end{matrix}
      \right)
      \f]
      
      where \f$ {\bf c} \f$ is the position of the free flying body,
      \f$ {\bf  o}$ its orientation using the \f$ xyz \f$ convention.
      \f$q_0, ..., q_{n-1}\f$ is the value for each of the \f$ n\f$ joint.
    */
    MAL_VECTOR(,double) m_Configuration;

    /** \brief Vector to store the global current velocity 

    \f$ q \f$ 
      The current format is :
      \f[ \dot{q} = \\
      \left( 
      \begin{matrix}
      {\bf v} \\
      {\bf w} \\
      \dot{q}_0 \\
      ... \\
      \dot{q}_{n-1}\\ 
      \end{matrix}
      \right)
      \f]
      
      where \f$ {\bf c} \f$ is the position of the free flying body,
      \f$ {\bf  o}$ its orientation using the \f$ xyz \f$ convention.
      \f$q_0, ..., q_{n-1}\f$ is the value for each of the \f$ n \f$ joint.
    */
    MAL_VECTOR(,double) m_Velocity;

    /** \brief Vector to store the global current velocity 

    \f$ q \f$ 
      The current format is :
      \f[ \dot{q} = \\
      \left( 
      \begin{matrix}
      {\bf V} \\
      {\bf W} \\
      \ddot{q}_0 \\
      ... \\
      \ddot{q}_{n-1}\\ 
      \end{matrix}
      \right)
      \f]
      
      where \f$ {\bf c} \f$ is the position of the free flying body,
      \f$ {\bf  o}$ its orientation using the \f$ xyz \f$ convention.
      \f$q_0, ..., q_{n-1}\f$ is the value for each of the \f$ n \f$ joint.
    */
    MAL_VECTOR(,double) m_Acceleration;

    /** Time step used to compute momentum derivative. */
    double m_TimeStep;

    /** Store the current ZMP value . */
    MAL_S3_VECTOR(,double) m_ZMP;

    /** Vector of pointers towards the joints. */
    std::vector<CjrlJoint *> m_JointVector;

    /**! \name Internal computation of Joints @{ */
    /*! \brief Method to compute the number of joints
      and btw the size of the configuration.
    */
    void ComputeNumberOfJoints();

    /*! Member to store the number of Dofs. */
    unsigned int m_NbDofs;
    
    /** @} */

    /*! \name Methods related to the building
      of proxy values. */
    /*! @{ */
    /*! Interface between the state vector and the joints. */
    std::vector<int> m_StateVectorToJoint; 

    /*! Interface between the state vector and the DOFs
      of each joints. */
    std::vector<int> m_StateVectorToDOFs; 

    /*! Interface between the VRML ID and the row of the
      configuration state vector */
    std::vector<int> m_VRMLIDToConfiguration;

    /*! Number of VRML IDs, they are supposed to be 
     positive or null, and continuous. */
    int m_NbOfVRMLIDs;

    /*! The method to build the two previous vectors. */
    void BuildStateVectorToJointAndDOFs();

    /*! @} */

    /*! Iteration number */
    unsigned int m_IterationNumber;

  public:
    
    /** \brief Default constructor. */
    DynamicMultiBody(void);
    
    /** \brief Destructor */
    virtual  ~DynamicMultiBody();
    
  
    //-----------------------------
    // Forward model computation
    //-----------------------------
    
    /** Parse a vrml file which describes the robot. The format
     should be compatible with the one specified by OpenHRP. */
    virtual void parserVRML(string path, string nom, const char *option);

    /** \name Dynamic parameters computation related methods 
     */
    
    /** \brief Computation the velocity following a tree like path.
	It is assume that the value of the joint has been
	correctly set. */
    void ForwardVelocity(MAL_S3_VECTOR(,double) &PosForRoot, 
			 MAL_S3x3_MATRIX(,double) &OrientationForRoot, 
			 MAL_S3_VECTOR(,double) &v0ForRoot);
    
    /** \brief Compute Inertia Matrices for Resolved Mometum Control
	Fist pass for tilde m and tilde c */
    void InertiaMatricesforRMCFirstStep();
    
    /** \brief Second pass for tilde I, and the inertia matrix M and H
	splitted across all the bodies in RMC_m and RMC_h. */
    void InertiaMatricesforRMCSecondStep();
    
    /** \brief Initialisation of the direct model computation */
    void ForwardDynamics(int corpsCourant, int liaisonDeProvenance);
    
    
    /** \brief Calculate ZMP. */
    void CalculateZMP(double &px, 
		      double &py,
		      MAL_S3_VECTOR(,double) dP, 
		      MAL_S3_VECTOR(,double) dL, 
		      double zmpz);

    /** \brief Compute the matrices MH*B, MH*Fi,MHFree  (Kajita IROS 2003 p. 1645) */
    void BuildSplittedInertialMatrices(  vector<int> LeftLeg, 
					 vector<int> RightLeg,
					 int WaistIndex, 
					 vector<int> FreeJoints);
    
    /** \brief Build the linear system for Resolved Momentum Control. */
    void BuildLinearSystemForRMC(MAL_MATRIX( &PLref, double),
				 MAL_MATRIX(&XiLeftFootRef,double),
				 MAL_MATRIX(&XiRightFootRef,double),
				 int NbOfFreeJoints,
				 MAL_MATRIX(&S,double),
				 MAL_MATRIX(&XiBdThetaFreeRef,double),
				 MAL_MATRIX(&XiBdThetaFree,double),
				 MAL_MATRIX(&LeftLegVelocity,double),
				 MAL_MATRIX(&RightLegVelocity,double));

    
    /** \brief Compute the D operator (Kajita IROS 2003 p. 1647) */
    MAL_S3x3_MATRIX(,double) D(MAL_S3_VECTOR(,double) &r);

    /** @}
     */ 
    
    /** \name Jacobian computation related methods 
	@{
     */
    
    /** \brief Computing the Jacobian. */
    int ComputeJacobian(int corps1, int corps2, 
			MAL_S3_VECTOR(,double) coordLocales, 
			double *jacobienne[6]);

    /** \brief Computing the Jacobian with a path (links) */
    void ComputeJacobianWithPath(vector<int> aPath,
				 MAL_MATRIX(&J,double));

    /** \brief Modifying the initial body. */
    void changerCorpsInitial(int nouveauCorps);

    /** \brief Finding a path between two bodies   
      (this is the version of "trouverCheminEntre" and has been set in english) 
    */
    vector<int> FindPathBetween(int body1, int body2);


    /** \brief Finding a path between the current body and the targeted body. */
    void trouverCheminEntreAux(int corpsCourant, int corpsVise, 
			       int liaisonDeProvenance, vector<int> &chemin);

    inline void empilerTransformationsLiaisonDirecte(int liaison);
    inline void empilerTransformationsLiaisonInverse(int liaison);
    
    void calculerMatriceTransformationEntre(int corps1, int corps2, float *matrice);
    void calculerMatriceTransformationEntre(int corps1, int corps2, double *matrice);
    
    vector<int> trouverCheminEntre(int corps1, int corps2);


    /** @} */

    /** Give the position of a body's point in a frame.  */
    MAL_S3_VECTOR(,double) getPositionPointDansRepere(MAL_S3_VECTOR(,double) point, 
						      int corpsDuPoint, int corpsDuRepere);

    /** \name Getter and setter for dynamic bodies  */

    /** @{ */

    /** Get back the joint values for the joint JointID in the VRML numbering system. */
    double Getq(int JointID) const;
    
    /** Specifies the joint values for the joint JointID in the VRML numbering system. */
    void Setq(int JointID, double q);

    /** Get the joint speed values for the joint JointID in the VRML numbering system. */
    double Getdq(int JointID) const;
    
    /** Specifies the joint speed values for the joint JointID in the VRML numbering system. */
    void Setdq(int JointID, double dq);
        
    /** Get the linear velocity for the joint JointID in the VRML numbering system*/
    MAL_S3_VECTOR(,double) Getv(int JointID);
    
    /** Get the linear velocity for the body JointID in the VRML numbering system. */
    MAL_S3_VECTOR(,double) GetvBody(int BodyID);
    
    /**  Set the orientation for the body JointID in the VRML numbering system. */
    void SetRBody(int BodyID, MAL_S3x3_MATRIX(,double) R);
    
    /** Set the linear velocity for the body JointID in the VRML numbering system. */
    void Setv(int JointID, MAL_S3_VECTOR(,double) v0);
    
    /** Set the angular velocity. */
    void Setw(int JointID, MAL_S3_VECTOR(,double) w);
    
    /** Get the angular velocity for the body JointID in the VRML numbering system. */
    MAL_S3_VECTOR(,double) Getw(int JointID);
    
    /** Get the angular velocity for the body JointID in the VRML numbering system . */
    MAL_S3_VECTOR(,double) GetwBody(int BodyID);
    
    /** Get the position for the body JointID in the VRML numbering system */
    MAL_S3_VECTOR(,double) Getp(int JointID);
    
    /** Get the Angular Momentum for the body JointID in the VRML numbering system */
    MAL_S3_VECTOR(,double) GetL(int JointID);
    
    /** Get the Linear Momentum for the body JointID in the VRML numbering system */
    MAL_S3_VECTOR(,double) GetP(int JointID);

    /** Set the time step to compute the Momentum derivative */
    inline void SetTimeStep(double inTimeStep)
      {m_TimeStep = inTimeStep;};
    
    /** Get the time step used to compute the Momentum derivative */
    inline double GetTimeStep() const
      {return m_TimeStep ;};
    
    /** Set the position, to be used with the body JointID in the VRML numbering system. */
    void Setp(int JointID, MAL_S3_VECTOR(,double) apos);

    /** Gives the two momentums vector. */
    void GetPandL(MAL_S3_VECTOR(,double) &aP, 
		  MAL_S3_VECTOR(,double) &aL);

    /** Get the position of the center of Mass. */
    MAL_S3_VECTOR(,double) getPositionCoM(void);
        
    
    /** Returns the name of the body JointID in the VRML numbering system. */
    string GetName(int JointID);

    /** Returns a CjrlJoint corresponding to body JointID in the VRML numbering system . */
    CjrlJoint * GetJointFromVRMLID(int JointID);

    /** Returns a vector to transfer from VRML ID to configuration ID . */
    void GetJointIDInConfigurationFromVRMLID(vector<int> & VectorFromVRMLIDToConfigurationID);

    /** Returns the ZMP value */
    inline const MAL_S3_VECTOR(,double) getZMP() const
      {return m_ZMP;};

    /** @} */

    /** \name Methods related to the construction of a tree,
	by specifying a root among the vertices of the undirected graph.
	@{ 
    */

    /** Specify the root of the tree and recompute it. */
    void SpecifyTheRootLabel(int ID);
    
    /** Relabel the current body \a corpsCourant according to the 
	link (specified by \a liaisonDeProvenance ) by which this body is explored.
     */
    void ReLabelling(int corpsCourant, int liaisonDeProvenance);
    
    /** @} */

    /** Print all the informations. */
    void PrintAll();
    
    /*! \name Methods related to the generic JRL-interface 
      @{
     */
      /**
     \name Kinematic chain
     @{
  */

    /**
       \brief Set the root joint of the robot.
    */
    void rootJoint(CjrlJoint& inJoint);
    
    /**
       \brief Get the root joint of the robot.
    */
    CjrlJoint* rootJoint() const;
    
    /**
       \brief Get a vector containing all the joints.
    */
    std::vector<CjrlJoint *> jointVector();
    
    /**
       \brief Get the number of degrees of freedom of the robot.
    */
    unsigned int numberDof() const;
    
    /**
       @}
    */
    
    /** 
	\name Configuration, velocity and acceleration
    */
    
    /**
       \brief Set the current configuration of the robot.  
       
       It is assumed that this vector includes the free joint
       corresponding to the root composed of the position and 
       the orientation in this case.
       The linear and angular velocity should be specified with
       the method currentVelocity().
       
       Regarding the orientation the convention is the \f$ x y z \f$ convention
       (pitch-roll-yaw), \f$ \theta \f$ is pitch, \f$ \psi \f$ is row,
       \f$ \phi \f$ is yaw.
       The corresponding orientation is then:


       \f$ {\bf D} \equiv \left[ 
       \begin{matrix}
       cos \; \phi & sin \; \phi & 0 \\
       -sin \; \phi & cos \; \phi & 0 \\
       0 & 0 & 1 \\
       \end{matrix}
       \right]
       \f$


       \f$ {\bf C} \equiv \left[ 
       \begin{matrix}
       cos \; \theta & 0  & -sin \; \theta \\
       0 & 1 & 0  \\
       sin \; \theta & 0 & cos \; \theta \\
       \end{matrix}
       \right]
       \f$


       \f$ {\bf B} \equiv \left[ 
       \begin{matrix}
       1 & 0 & 0 \\
       0 & cos \; \psi & sin \; \psi  \\
       0 & -sin \; \psi & cos \; \psi  \\
       \end{matrix}
       \right]
       \f$


       the final matrix is \f$ {\bf A} = {\bf B} {\bf C} {\bf D} \f$
       

       \param inConfig the configuration vector \f${\bf q}\f$.
       
       \return true if success, false if failure (the dimension of the
       input vector does not fit the number of degrees of freedom of the
       robot).
    */
    bool currentConfiguration(const MAL_VECTOR(,double)& inConfig);
    
    /**
       \brief Get the current configuration of the robot.
       
       \return the configuration vector \f${\bf q}\f$.
    */
    const MAL_VECTOR(,double)& currentConfiguration() const;
    
    /**
       \brief Set the current velocity of the robot.  
       
       \param inVelocity the velocity vector \f${\bf \dot{q}}\f$.
       
       \return true if success, false if failure (the dimension of the
       input vector does not fit the number of degrees of freedom of the
       robot).
    */
    bool currentVelocity(const MAL_VECTOR(,double)& inVelocity);
    
    /**
       \brief Get the current velocity of the robot.
       
       \return the velocity vector \f${\bf \dot{q}}\f$.
    */
    const MAL_VECTOR(,double)& currentVelocity() const;
    /**
       \brief Set the current acceleration of the robot.  
       
       \param inAcceleration the acceleration vector \f${\bf \ddot{q}}\f$.
       
       \return true if success, false if failure (the dimension of the
       input vector does not fit the number of degrees of freedom of the
       robot).
    */
    bool currentAcceleration(const MAL_VECTOR(,double)& inAcceleration) ;
    
    /**
       \brief Get the current acceleration of the robot.

       \return the acceleration vector \f${\bf \ddot{q}}\f$.
    */
    const MAL_VECTOR(,double)& currentAcceleration() const ;
    
    /**
       @}
    */
    
    /** 
	\name Forward kinematics and dynamics
    */
    
    /**
       \brief Compute forward kinematics.
       
       Update the position, velocity and accelerations of each
       joint wrt \f${\bf {q}}\f$, \f${\bf \dot{q}}\f$, \f${\bf \ddot{q}}\f$.
       
    */
    bool computeForwardKinematics() ;
    
    /**
       \brief Compute the dynamics of the center of mass.
       
       Compute the linear and  angular momentum and their time derivatives, at the center of mass.
    */
    bool computeCenterOfMassDynamics() ;
    
    /**
       \brief Get the position of the center of mass.
    */
    const MAL_S3_VECTOR(,double)& positionCenterOfMass() ;
    
    /**
       \brief Get the velocity of the center of mass.
    */
    const MAL_S3_VECTOR(,double)& velocityCenterOfMass() ;
    
    /**
       \brief Get the acceleration of the center of mass.
    */
    const MAL_S3_VECTOR(,double)& accelerationCenterOfMass() ;
    
    /**
       \brief Get the linear momentum of the robot.
    */
    const MAL_S3_VECTOR(,double)& linearMomentumRobot() ;
    
    /**
       \brief Get the time-derivative of the linear momentum.
    */
    const MAL_S3_VECTOR(,double)& derivativeLinearMomentum() ;
    
    /**
       \brief Get the angular momentum of the robot at the center of mass.
    */
    const MAL_S3_VECTOR(,double)& angularMomentumRobot() ;
    
    /**
       \brief Get the time-derivative of the angular momentum at the center of mass.
    */
    const MAL_S3_VECTOR(,double)& derivativeAngularMomentum() ;
    
    /**
       @}
    */
    
    /** 
	\name Jacobian fonctions
    */
    
    /**
       \brief Compute the Jacobian matrix of the center of mass wrt \f${\bf q}\f$.
    */
     void computeJacobianCenterOfMass() ;
    
    /**
       \brief Get the Jacobian matrix of the center of mass wrt \f${\bf q}\f$.
    */
     const MAL_MATRIX(,double) & jacobianCenterOfMass() const ;
    
    /**
       @}
    */
     
     
    /*! @}*/

     /*! \name Method related to the iteration number. 
       @{
      */
     /*! Reset the iterationNumber. */
     bool ResetIterationNumber()
     { m_IterationNumber = 0;} ;

     /*! GetIndex */
     const unsigned int & GetIterationNumber() const
       { return m_IterationNumber;} 
   
  };
};
#endif
