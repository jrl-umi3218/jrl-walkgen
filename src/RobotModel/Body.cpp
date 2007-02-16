/* @doc Fundamental object used to compute :
   - Center Of Mass,
   - Zero Momentum Point.
   
   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin
   OS (21/12/2006): removed any reference to non-homogeneous matrix
   library.


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

#include "Body.h"

using namespace PatternGeneratorJRL;

/**************************************************/
/* Implementation of constructors and destructors */
/**************************************************/

Body::Body(void)
{
  //  label = cptCorps++;
  masse	= 0;
  posCoM[0] = 0;
  posCoM[1] = 0;
  posCoM[2] = 0;
  MAL_S3x3_MATRIX_SET_IDENTITY(inertie);
  nombreObjets = 0;
  label=-1;
  labelMother=-1;
  m_Explored =0 ;
}

Body::Body(double lmasse) 
{
  //  label = cptCorps++;
  this->masse		= lmasse;
  nombreObjets = 0;
  m_Explored =0 ;
}

Body::Body(double lmasse, 
	   MAL_S3_VECTOR(positionCoM,double)) 
{
  //  label = cptCorps++;
  this->masse		= lmasse;
  nombreObjets = 0;
  m_Explored =0 ;  
  this->posCoM	= positionCoM;
}
Body::Body(double lmasse, 
	   MAL_S3_VECTOR(positionCoM,double), 
	   MAL_S3x3_MATRIX(matriceInertie,double)) 
{
  //  label = cptCorps++;
  this->posCoM	= positionCoM;
  this->masse		= lmasse;
  nombreObjets = 0;
  m_Explored =0 ;  

  this->inertie	= matriceInertie;
}


Body::~Body(void)
{
}


/***********************************************/
/* Implementation of setters and getters       */
/***********************************************/

int Body::getLabel() const
{
  return label;
}

void Body::setLabel(int i)
{
  label = i;
}

double Body::getMasse(void) const
{
  return masse;
}

MAL_S3_VECTOR(,double) Body::getPositionCoM(void) const
{
  return posCoM;
}

void Body::setPositionCoM(double cm[3])
{
  posCoM[0] = cm[0];
  posCoM[1] = cm[1];
  posCoM[2] = cm[2];

}

void Body::setNbObjets(int n)
{
  nombreObjets = n;
}

void Body::afficherNombreObjets() {
  cout << "corps " << label << " : " << nombreObjets << endl;
}


int Body::getNbObjets() const
{
  return nombreObjets;
}

string Body::getName() const
{
  return Name;
}

void Body::setName(char *aname)
{
  Name = aname;
}

Body & Body::operator=( Body const & r)
{
  label = r.getLabel();
  masse = r.getMasse();
  posCoM = r.getPositionCoM();
  inertie = r.getInertie();
  nombreObjets = r.getNbObjets();
  Name= r.getName();
  labelMother=r.getLabelMother();
  m_Explored = r.getExplored();
  return *this;
}

void Body::setInertie(double mi[9])
{
  for(unsigned int i=0;i<3;i++)
    for(unsigned int j=0;j<3;j++)
      inertie(i,j) = mi[i*3+j];
}

void Body::setMasse(double lmasse)
{
  masse =lmasse;
}


void Body::setLabelMother(int al)
{
  labelMother = al;
}

int Body::getLabelMother() const
{
  return labelMother;
}

int Body::getExplored() const
{
  return m_Explored;
}

void Body::setExplored(int anEx)
{
  m_Explored = anEx;

}

/***********************************************/
/* Implementation of the methods for display   */
/***********************************************/

void Body::Display()
{
  cout << "Name  :" << Name << endl;
  cout << "Masse :" << masse << endl;
  cout << "Center of Mass    : " << posCoM[0] << " " 
       << posCoM[1] << " " <<posCoM[2]<<endl;
  cout << "Matrix of Inertie : " << endl;
  for(int i=0;i<3;i++)
    {
      for(int j=0;j<3;j++)
	cout << inertie(i,j) << " ";
      cout << endl;
    }
  cout << "Mother: " << labelMother<< endl;
}


/***********************************************/
/* Implementation of the generic JRL interface */
/***********************************************/

const MAL_S3_VECTOR(,double) & Body::localCenterOfMass() const
{
  return posCoM;
}

void Body::localCenterOfMass(const MAL_S3_VECTOR(,double) &inlocalCenterOfMass)
{
  posCoM = inlocalCenterOfMass;
}

/*! Returns inertia matrix in the local reference frame */
const MAL_S3x3_MATRIX(,double) & Body::inertiaMatrix() const
{
  return inertie;
}


void Body::inertiaMatrix(const MAL_S3x3_MATRIX(,double) &inInertiaMatrix) 
{
  inertie = inInertiaMatrix;
}

const CjrlJoint * Body::joint() const
{
  return m_AttachedJoint;
}

void Body::joint(Joint * ajoint)
{
  m_AttachedJoint = ajoint;
}



