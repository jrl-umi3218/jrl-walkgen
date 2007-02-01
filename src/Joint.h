/* Class to implement a Joint object.

   Copyright (c) 2007, 
   @author Olivier Stasse
   
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

#ifndef __PG_JRL_JOINT_H_
#define __PG_JRL_JOINT_H_
#include <MatrixAbstractLayer.h>
#include <jrlJoint.h>


namespace PatternGeneratorJRL
{
  /** @ingroup forwardynamics
       Define a transformation from a body to another
      Supported type:
      - Rotation around an axis with a quantity (type = ROTATION)
      - Translation of a vector : quantite*axe	(type = TRANSLATION)
      - Rotation through a homogeneous matrix : *rotation (type = ROTATION_LIBRE)
  */
  class Joint: public CjrlJoint
  {

  private:
    /*!  Type of the transformation */ 
    int m_type;
    
    /*! Axis of the transformation,
      for the link with one DoF. */
    MAL_S3_VECTOR(,double) m_axe;	
    
    /*! Quantity of the rotation . */
    float m_quantity;

    /*! 4x4 matrix for free rotation */
    MAL_S4x4_MATRIX(,double) m_rotation;

    /*! Father joint */
    Joint * m_FatherJoint;

    /*! Vector of childs */
    std::vector<CjrlJoint *> m_Children;

  public: 
    

    /*! \brief Constructor with full initialization. */
    Joint(int ltype, MAL_S3_VECTOR(,double) laxe, 
	  float lquantite, MAL_S4x4_MATRIX(,double) rotation);
    
    Joint(int ltype, MAL_S3_VECTOR(,double) laxe, 
	  float lquantite);
	
    /*! \brief Constructor by copy. */
    Joint(const Joint &r); 

    /*! \brief default destructor */
    ~Joint();
    
    /*! \brief Affectation operator */
    Joint & operator=(const Joint &r);

    /*! \brief Operator to get one element of the rotation 
      
    \f$
    R= \left[ 
    \begin{array}{cccc}
    r_{0} & r_{1} & r_{2} & r_{3} \\
    r_{4} & r_{5} & r_{6} & r_{7} \\
    r_{8} & r_{9} & r_{10} & r_{11} \\
    r_{12} & r_{13} & r_{14} & r_{15} \\
    \end{array}
    \right]
    \f$
    where \f$rotation(i) = r_{i}\f$.
     */
    float rotation(unsigned r) ;

    /*! Returns the matrix corresponding to the rigid motion */
    inline const MAL_S4x4_MATRIX(,double) & rotation() const
      {return m_rotation;};

    /*! Operator to access the rotation matrix */
    inline double & operator()(unsigned int i) 
      {return m_rotation(i/4,i%4);}

    /*! Operator to access the rotation matrix */
    inline double & operator()(unsigned int i,
			       unsigned int j) 
      {return m_rotation(i,j);}
    
    
    /*! \name Getter and setter for the parameter 
      @{
     */    
    
    /*! Returns the axe of the rotation. */
    inline const MAL_S3_VECTOR(,double ) & axe() const
      { return m_axe; };

    /*! Set the axe of the rotation */
    inline void axe(const MAL_S3_VECTOR(,double) &anaxe)
      { m_axe = anaxe; };

    /*! Quantity of the rotation */
    inline const float & quantity() const
      { return m_quantity; } 

    /*! Set the rotation of the joint */
    inline void quantity(const float & aquantity)
      { m_quantity = aquantity; }

    /*! Returns the type of the joint */
    inline const int & type() const
      { return m_type; } 

    /*! Set the type of the joint */
    inline void type(const int atype) 
      { m_type = atype; } 

    /* @} */

    /*! \name Implements the virtual function inherited from CjrlJoint
      @{
     */
    /*! \name Joint hierarchy 
      @{
     */
    /*! \brief parent Joint */
    CjrlJoint &  parentJoint() const ;

    /*! \brief Add a child Joint */
    bool addChildJoint(const CjrlJoint &);
    
    /*! \brief Count the number of child joints */
    unsigned int countChildJoints() const;
    
    /*! \brief Returns the child joint at the given rank */
    const CjrlJoint & childJoint(unsigned int givenRank) const;

    /*! \brief Joints from root to this joint */
    std::vector<CjrlJoint *> jointsFromRootToThis() const ;
    /*! @} */

    /*! \name Joint Kinematics 
      @{
     */
    
    /**
       \brief Get the current transformation of the joint.
       
       The current transformation of the joint is the transformation
       moving the joint from the position in initial configuration to
       the current position. 
       
       The current transformation is determined by the configuration \f${\bf q}\f$ of the robot.
    */
    const MAL_S4x4_MATRIX(,double) &currentTransformation() const;
    
    /**
       \brief Get the velocity \f$({\bf v}, {\bf \omega})\f$ of the joint.
       
       The velocity is determined by the configuration of the robot and its time derivative: \f$({\bf q},{\bf \dot{q}})\f$.
       
       \return the linear velocity \f${\bf v}\f$ of the origin of the joint frame
       and the angular velocity \f${\bf \omega}\f$ of the joint frame.
    */
    CjrlRigidVelocity jointVelocity();
    
    /**
       \brief Get the acceleration of the joint.
       
       The acceleratoin is determined by the configuration of the robot 
       and its first and second time derivative: \f$({\bf q},{\bf \dot{q}}, {\bf \ddot{q}})\f$.
    */
    CjrlRigidAcceleration jointAcceleration();
    
    /**
       \brief Get the number of degrees of freedom of the joint.
    */
    unsigned int numberDof() const;
    
    
    /*! @} */
    
    /**
       \name Jacobian functions wrt configuration.
       @{
    */

    /**
       \brief Get the Jacobian matrix of the joint position wrt the robot configuration.

       The corresponding computation can be done by the robot for each of its joints or by the joint.
     
       \return a matrix \f$J \in {\bf R}^{6\times n_{dof}}\f$ defined by 
       \f[
       J = \left(\begin{array}{llll}
       {\bf v_1} & {\bf v_2} & \cdots & {\bf v_{n_{dof}}} \\
       {\bf \omega_1} & {\bf \omega_2} & \cdots & {\bf \omega_{n_{dof}}}
       \end{array}\right)
       \f]
       where \f${\bf v_i}\f$ and \f${\bf \omega_i}\f$ are respectively the linear and angular velocities of the joint 
       implied by the variation of degree of freedom \f$q_i\f$. The velocity of the joint returned by 
       CjrlJoint::jointVelocity can thus be obtained through the following formula:
       \f[
       \left(\begin{array}{l} {\bf v} \\ {\bf \omega}\end{array}\right) = J {\bf \dot{q}}
       \f]
    */
    MAL_MATRIX(,double) jacobianPositionJointWrtConfig() const ;

    /**
       @}
    */

    /**
       \name Body linked to the joint
       @{
    */

    /**
       \brief Get a pointer to the linked body (if any).
    */
    CjrlBody* linkedBody() const;
 	
    /**
       \brief Link a body to the joint.
    */
    int setLinkedBody (CjrlBody& inBody);
  
    /**
       @}
    */
    
    /*! @} */

    /*! Specify the joint father. */
    void SetFatherJoint(Joint *aFather);

  };
};
#endif
