/* @doc Computation of the dynamic aspect for a robot.
   This class will load the description of a robot from a VRML file
   following the OpenHRP syntax. Using ForwardVelocity it is then
   possible specifying the angular velocity and the angular value 
   to get the absolute position, and absolute velocity of each 
   body separetly. Heavy rewriting from the original source
   of Adrien and Jean-Remy. 

   This implantation is an updated based on a mixture between 
   the code provided by Jean-Remy and Adrien.


   CVS Information:
   $Id: DynamicMultiBody.h,v 1.2 2006-01-18 06:34:58 stasse Exp $
   $Author: stasse $
   $Date: 2006-01-18 06:34:58 $
   $Revision: 1.2 $
   $Source: /home/CVSREPOSITORY/PatternGeneratorJRL/src/DynamicMultiBody.h,v $
   $Log: DynamicMultiBody.h,v $
   Revision 1.2  2006-01-18 06:34:58  stasse
   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin

  
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

using namespace::std;

#include <VNL/matrix.h>
#include <MultiBody.h>
#include <DynamicBody.h>

namespace PatternGeneratorJRL {

/// This class simulates the dynamic of a multibody robot.
class DynamicMultiBody : public MultiBody
{
 protected:

  /// Label of the root.
  int labelTheRoot;

  /// Dynamic Bodies
  vector<DynamicBody> listOfBodies;

  /// Array to convert Joint Id from VRL file to Body array index.
  vector<int> ConvertJOINTIDToBodyID;

  /** Update body parameters from the bodyinfo list of 
      joints and the internal list of bodies. */
  void UpdateBodyParametersFromJoint(int cID, int lD);

  /// The skew matrix related to the CoM position.
  matrix3d SkewCoM;

  /// Splitted inertial matrices.
  VNL::Matrix<double> m_MHStarB, m_MHStarLeftFoot, m_MHStarRightFoot, m_MHFree;

  /// Inversed Jacobian for the left and right foot.
  VNL::Matrix<double> m_ILeftJacobian,m_IRightJacobian;
  VNL::Matrix<double> m_ERBFI_Left, m_ERBFI_Right;
  

 public:

  /// Constructor.
  DynamicMultiBody(void);
  
  /// Destructor
  virtual  ~DynamicMultiBody();

  /// Weighted CoM position.
  vector3d positionCoMPondere;

  /// Get the position of the center of Mass.
  vector3d getPositionCoM(void);

  /// Momentum vector
  vector3d m_P;

  /// Angular momentum vector.
  vector3d m_L;

  /// Starting the computation.
  bool m_FirstTime;

  
  //-----------------------------
  // Forward model computation
  //-----------------------------

  virtual void parserVRML(string path, string nom, const char *option);

  /** Computation the velocity following a tree like path.
      It is assume that the value of the joint has been
      correctly set. */
  void ForwardVelocity(vector3d PosForRoot, matrix3d OrientationForRoot, vector3d v0ForRoot);

  /** Compute Inertia Matrices for Resolved Mometum Control
      Fist pass for tilde m and tilde c */
  void InertiaMatricesforRMCFirstStep();

  /** Second pass for tilde I, and the inertia matrix M and H
      splitted across all the bodies in RMC_m and RMC_h. */
  void InertiaMatricesforRMCSecondStep();

  /// Initialisation of the direct model computation
  void ForwardDynamics(int corpsCourant, int liaisonDeProvenance);
  
  /// Computig the Jacobian.
  int ComputeJacobian(int corps1, int corps2, vector3d coordLocales, double *jacobienne[6]);

  /// Computing the Jacobian with a path (links)
  void ComputeJacobianWithPath(vector<int> aPath, VNL::Matrix<double> &J);

  /// Modifying the initial body.
  void changerCorpsInitial(int nouveauCorps);

  /// Finding a path between two bodies   (this is the version of "trouverCheminEntre" and has been set in english) 
  vector<int> FindPathBetween(int body1, int body2);


  /// Finding a path between the current body and the targeted body.
  void trouverCheminEntreAux(int corpsCourant, int corpsVise, int liaisonDeProvenance, vector<int> &chemin);

  /// Give the position of a body's point in a frame. 
  vector3d getPositionPointDansRepere(vector3d point, int corpsDuPoint, int corpsDuRepere);

  /// Get back the joint values.
  double Getq(int JointID);

  /// Specifies the joint values.
  void Setq(int JointID, double q);

  /// Specifies the joint speed values.
  void Setdq(int JointID, double dq);
  
  /// Get the joint speed values.
  double Getdq(int JointID);

  /// Get the linear velocity for the joint
  vector3d Getv(int JointID);

  /// Get the linear velocity for the body.
  vector3d GetvBody(int BodyID);

  /// Set the orientation for the body.
  void SetRBody(int BodyID, matrix3d R);

  /// Set the linear velocity.
  void Setv(int JointID, double v0);

  /// Set the angular velocity.
  void Setw(int JointID, double w);

  /// Get the angular velocity.
  vector3d Getw(int JointID);

  /// Get the angular velocity for the body.
  vector3d GetwBody(int BodyID);
  
  /// Get the position
  vector3d Getp(int JointID);

  /// Get the Angular Momentum
  vector3d GetL(int JointID);

  /// Get the Linear Momentum
  vector3d GetP(int JointID);

  /// Set the position, to be used with the first body.
  void Setp(int JointID, vector3d apos);
  
  /// Specify the root of the tree and recompute it.
  void SpecifyTheRootLabel(int ID);
  
  /// Print all the informations.
  void PrintAll();

  inline void empilerTransformationsLiaisonDirecte(int liaison);
  inline void empilerTransformationsLiaisonInverse(int liaison);

  void calculerMatriceTransformationEntre(int corps1, int corps2, float *matrice);
  void calculerMatriceTransformationEntre(int corps1, int corps2, double *matrice);

  vector<int> trouverCheminEntre(int corps1, int corps2);

  void ReLabelling(int corpsCourant, int liaisonDeProvenance);

  // Gives the two momentums vector.
  void GetPandL(vector3d &aP, vector3d &aL);

  // Calculate ZMP.
  void CalculateZMP(double &px, 
		    double &py,
		    vector3d dP, vector3d dL, double zmpz);

  /// Returns the name of the body i.
  string GetName(int JointID);

  /// Compute the D operator (Kajita IROS 2003 p. 1647)
  matrix3d D(vector3d r);

  /// Compute the matrices MH*B, MH*Fi,MHFree  (Kajita IROS 2003 p. 1645)
  void BuildSplittedInertialMatrices(  vector<int> LeftLeg, vector<int> RightLeg,
				       int WaistIndex, vector<int> FreeJoints);

  /// Build the linear system for Resolved Momentum Control.
  void BuildLinearSystemForRMC(VNL::Matrix<double> &PLref, 
			       VNL::Matrix<double> &XiLeftFootRef,
			       VNL::Matrix<double> &XiRightFootRef,
			       int NbOfFreeJoints,
			       VNL::Matrix<double> &S,
			       VNL::Matrix<double> &XiBdThetaFreeRef,
			       VNL::Matrix<double> &XiBdThetaFree,       
			       VNL::Matrix<double> &LeftLegVelocity,
			       VNL::Matrix<double> &RightLegVelocity);

};
};
#endif
