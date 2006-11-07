/* @doc Fundamental object used to compute :
   - Center Of Mass,
   - Zero Momentum Point.
   
   $Id: Body.cpp,v 1.2 2006-01-18 06:34:58 stasse Exp $
   $Author: stasse $
   $Date: 2006-01-18 06:34:58 $
   $Revision: 1.2 $
   $Source: /home/CVSREPOSITORY/PatternGeneratorJRL/src/Body.cpp,v $
   $Log: Body.cpp,v $
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

#include <iostream>

using namespace std;
#include "Body.h"

using namespace PatternGeneratorJRL;

//---------------------------------------------------------------
//Constructeurs et destructeur
Body::Body(void)
{
  //  label = cptCorps++;
  masse	= 0;
  posCoM	= 0;
  inertie = matrix3d::identity;
  nombreObjets = 0;
  label=-1;
  labelMother=-1;
  m_Explored =0 ;
}

Body::Body(double masse, vector3d positionCoM, matrix3d matriceInertie) 
{
  //  label = cptCorps++;
  this->masse		= masse;
  this->posCoM	= positionCoM;
  this->inertie	= matriceInertie;
  nombreObjets = 0;
  m_Explored =0 ;
}


Body::~Body(void)
{
}

//Fin constructeurs et destructeur
//---------------------------------------------------------------


//---------------------------------------------------------------
//Accesseurs
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

vector3d Body::getPositionCoM(void) const
{
  return posCoM;
}

void Body::setPositionCoM(double cm[3])
{
  posCoM.x = cm[0];
  posCoM.y = cm[1];
  posCoM.z = cm[2];

}

void Body::setNombreObjets(int n)
{
  nombreObjets = n;
}

void Body::afficherNombreObjets() {
  cout << "corps " << label << " : " << nombreObjets << endl;
}


matrix3d Body::getInertie() const
{
  return inertie;
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
  for(unsigned int i=0;i<9;i++)
      inertie[i] = mi[i];
}

void Body::setMasse(double lmasse)
{
  masse =lmasse;
}

void Body::Display()
{
  cout << "Name  :" << Name << endl;
  cout << "Masse :" << masse << endl;
  cout << "Center of Mass    : " << posCoM[0] << " " 
       << posCoM[1] << " " <<posCoM[2]<<endl;
  cout << "Matrix of Inertie : " << endl;
  cout << "Mother: " << labelMother<< endl;
  inertie.display();
}

void Body::setLabelMother(int al)
{
  labelMother = al;
}

int Body::getLabelMother() const
{
  return labelMother;
}

//Fin accesseurs
//---------------------------------------------------------------
int Body::getExplored() const
{
  return m_Explored;
}

void Body::setExplored(int anEx)
{
  m_Explored = anEx;
}




