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

#include <MatrixAbstractLayer.h>
#include <MultiBody.h>
#include <DynamicBody.h>
using namespace::std;
namespace PatternGeneratorJRL 
{

  /** @ingroup forwardynamics
      This class simulates the dynamic of a multibody robot.
  */
  class DynamicMultiBody : public MultiBody
  {
  protected:
    
    /**  Label of the root. */
    int labelTheRoot;
    
    /**  List of bodies with dynamical properties */
    vector<DynamicBody> listOfBodies;
    
    /** Array to convert Joint Id from VRL file to Body array index. */
    vector<int> ConvertJOINTIDToBodyID;
    
    /** Update body parameters from the bodyinfo list of 
      joints and the internal list of bodies. */
    void UpdateBodyParametersFromJoint(int cID, int lD);
    
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

    /** Linear momentum vector */
    MAL_S3_VECTOR(,double) m_P;
    
    /** Angular momentum vector. */
    MAL_S3_VECTOR(,double) m_L;
    
    /** Starting the computation. */
    bool m_FirstTime;
    
  public:
    
    /** Default constructor. */
    DynamicMultiBody(void);
    
    /** Destructor */
    virtual  ~DynamicMultiBody();
    
  
    //-----------------------------
    // Forward model computation
    //-----------------------------
    
    /** Parse a vrml file which describes the robot. The format
     should be compatible with the one specified by OpenHRP. */
    virtual void parserVRML(string path, string nom, const char *option);

    /** \name Dynamic parameters computation related methods 
	@{
     */
    
    /** Computation the velocity following a tree like path.
	It is assume that the value of the joint has been
	correctly set. */
    void ForwardVelocity(MAL_S3_VECTOR(,double) &PosForRoot, 
			 MAL_S3x3_MATRIX(,double) &OrientationForRoot, 
			 MAL_S3_VECTOR(,double) &v0ForRoot);
    
    /** Compute Inertia Matrices for Resolved Mometum Control
	Fist pass for tilde m and tilde c */
    void InertiaMatricesforRMCFirstStep();
    
    /** Second pass for tilde I, and the inertia matrix M and H
	splitted across all the bodies in RMC_m and RMC_h. */
    void InertiaMatricesforRMCSecondStep();
    
    /** Initialisation of the direct model computation */
    void ForwardDynamics(int corpsCourant, int liaisonDeProvenance);
    
    
    /** Calculate ZMP. */
    void CalculateZMP(double &px, 
		      double &py,
		      MAL_S3_VECTOR(,double) dP, 
		      MAL_S3_VECTOR(,double) dL, 
		      double zmpz);

    /** Compute the matrices MH*B, MH*Fi,MHFree  (Kajita IROS 2003 p. 1645) */
    void BuildSplittedInertialMatrices(  vector<int> LeftLeg, 
					 vector<int> RightLeg,
					 int WaistIndex, 
					 vector<int> FreeJoints);
    
    /** Build the linear system for Resolved Momentum Control. */
    void BuildLinearSystemForRMC(MAL_MATRIX( &PLref, double),
				 MAL_MATRIX(&XiLeftFootRef,double),
				 MAL_MATRIX(&XiRightFootRef,double),
				 int NbOfFreeJoints,
				 MAL_MATRIX(&S,double),
				 MAL_MATRIX(&XiBdThetaFreeRef,double),
				 MAL_MATRIX(&XiBdThetaFree,double),
				 MAL_MATRIX(&LeftLegVelocity,double),
				 MAL_MATRIX(&RightLegVelocity,double));

    /** @}*/ 
    
    /** Jacobian computation related methods 
	@{
     */
    
    /** Computig the Jacobian. */
    int ComputeJacobian(int corps1, int corps2, 
			MAL_S3_VECTOR(,double) coordLocales, 
			double *jacobienne[6]);

    /** Computing the Jacobian with a path (links) */
    void ComputeJacobianWithPath(vector<int> aPath,
				 MAL_MATRIX(&J,double));

    /** Modifying the initial body. */
    void changerCorpsInitial(int nouveauCorps);

    /** Finding a path between two bodies   
      (this is the version of "trouverCheminEntre" and has been set in english) 
    */
    vector<int> FindPathBetween(int body1, int body2);


    /** Finding a path between the current body and the targeted body. */
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

    /** Get back the joint values. */
    double Getq(int JointID);
    
    /** Specifies the joint values. */
    void Setq(int JointID, double q);
    
    /** Specifies the joint speed values. */
    void Setdq(int JointID, double dq);
    
    /** Get the joint speed values. */
    double Getdq(int JointID);
    
    /** Get the linear velocity for the joint */
    MAL_S3_VECTOR(,double) Getv(int JointID);
    
    /** Get the linear velocity for the body. */
    MAL_S3_VECTOR(,double) GetvBody(int BodyID);
    
    /**  Set the orientation for the body. */
    void SetRBody(int BodyID, MAL_S3x3_MATRIX(,double) R);
    
    /** Set the linear velocity. */
    void Setv(int JointID, MAL_S3_VECTOR(,double) v0);
    
    /** Set the angular velocity. */
    void Setw(int JointID, MAL_S3_VECTOR(,double) w);
    
    /** Get the angular velocity. */
    MAL_S3_VECTOR(,double) Getw(int JointID);
    
    /** Get the angular velocity for the body. */
    MAL_S3_VECTOR(,double) GetwBody(int BodyID);
    
    /** Get the position */
    MAL_S3_VECTOR(,double) Getp(int JointID);
    
    /** Get the Angular Momentum */
    MAL_S3_VECTOR(,double) GetL(int JointID);
    
    /** Get the Linear Momentum */
    MAL_S3_VECTOR(,double) GetP(int JointID);
    
    /** Set the position, to be used with the first body. */
    void Setp(int JointID, MAL_S3_VECTOR(,double) apos);

    /** Gives the two momentums vector. */
    void GetPandL(MAL_S3_VECTOR(,double) &aP, 
		  MAL_S3_VECTOR(,double) &aL);

    /** Get the position of the center of Mass. */
    MAL_S3_VECTOR(,double) getPositionCoM(void);
        
    
    /** Returns the name of the body i. */
    string GetName(int JointID);

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
    
    /** Compute the D operator (Kajita IROS 2003 p. 1647) */
    MAL_S3x3_MATRIX(,double) D(MAL_S3_VECTOR(,double) &r);
    
    
  };
};
#endif
