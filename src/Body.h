/* @doc Fundamental object used to compute :
   - Center Of Mass,
   - Zero Momentum Point.

   $Id: Body.h,v 1.2 2006-01-18 06:34:58 stasse Exp $
   $Author: stasse $
   $Date: 2006-01-18 06:34:58 $
   $Revision: 1.2 $
   $Source: /home/CVSREPOSITORY/PatternGeneratorJRL/src/Body.h,v $
   $Log: Body.h,v $
   Revision 1.2  2006-01-18 06:34:58  stasse
   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin


   Copyright (c) 2005-2006, 
   @author : 
   Adrien Escande, 
   Abderrahmane Kheddar,  
   Olivier Stasse, 
   Ramzi Sellouati
   
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
#ifndef _BODY_H_
#define _BODY_H_

#include<iostream>
#include <string>

#include "linalg.h"

namespace PatternGeneratorJRL
{

/** This object is used to compute the Center Of Masse,
    and store some basic information of each body of the robot. */    
class Body
{
 protected:

  /// Physical parameters of the body.
   //@{
   /// Label
  int label;
  /// Mass
  double masse;
  /// Position of the Center of Mass.
  vector3d posCoM;
  /// Inertia Matrix.
  matrix3d inertie;
  //@}
  
  /// Number of objects.
  int nombreObjets;
  /// Name of the body
  std::string Name;

  /// Label of the mother of the body in a tree structure.
  int labelMother;
  
  // Says if the body has been explored.
  int m_Explored;

public:
  
  /// Basic constructor.
  Body(void);

  /// Constructor while parsing.
  Body(double masse, vector3d positionCoM = 0, matrix3d matriceInertie = matrix3d::identity);

  /// Destructor.
  ~Body(void);
  
  /// Assignment operator.
  Body & operator=( Body const & r);

  /// Get the label of the body.
  int getLabel(void) const;

  /// Set the label of the body.
  void setLabel(int i);

  /// Get the mass of the body.
  double getMasse(void) const;
  
  /// Set the mass of the body.
  void setMasse(double);

  /// Get the inertia matrix of the body.
  matrix3d getInertie(void) const;

  /// Set the inertia matrix of the body.
  void setInertie(double mi[9]);
  
  /// Get the position of the center of Mass.
  vector3d getPositionCoM(void) const;

  /// Set the position of the center of Mass.
  void setPositionCoM(double cm[3]);

  /// Returns the label of the mother.
  int getLabelMother() const;

  /// Set the label of the mother.
  void setLabelMother(int);
  
  /// Get the number of geometric objects.
  int getNbObjets() const;

  /// Set the number of geometric objects.
  void setNombreObjets(int n);

  /// Display the number of geometric objects (stdout).
  void afficherNombreObjets(void);

  /// Returns if the object has been explored or not.
  int getExplored() const;

  /// Set the object as explored.
  void setExplored(int anEx);
  
  /// Returns the name of the object.
  std::string getName() const;

  /// Specify the name of the object.
  void setName(char *);

  /// Display on stdout all the information of the body.
  void Display();
};

};
#endif /* Body_H_*/
