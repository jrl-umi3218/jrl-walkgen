/* @doc Functions to parse a VRML file.
   OS: Small modification to read other parameters such as
   joint name, id, and make it a bit more model versatile.

   CVS Information: 
   $Id: fileReader.h,v 1.2 2006-01-18 06:34:58 stasse Exp $
   $Author: stasse $
   $Date: 2006-01-18 06:34:58 $
   $Revision: 1.2 $
   $Source: /home/CVSREPOSITORY/PatternGeneratorJRL/src/fileReader.h,v $
   $Log: fileReader.h,v $
   Revision 1.2  2006-01-18 06:34:58  stasse
   OS: Updated the names of the contributors, the documentation
   and added a sample file for WalkPlugin


   Copyright (c) 2005-2006, 
   @author Adrien Escande, Abderrahmane Kheddar, Olivier Stasse, Ramzi Sellouati
   
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

#ifndef __PG_FILE_HEADER__
#define __PG_FILE_HEADER__


#include <stdio.h>
#include <stdlib.h>
#include <iostream>
using namespace std;

#define CROCHET_OUVRANT		0
#define CROCHET_FERMANT		1
#define CHILDREN		2
#define JOINT			3
#define DEF                     4

#define AXE_X			0
#define AXE_Y			1
#define AXE_Z			2
#define JOINT_TRANSLATION	3
#define JOINT_ROTATION		4
#define JOINT_ID                5

namespace PatternGeneratorJRL
{

  /// Function to look for an object.
  char look_for(FILE* fichier, const char *str);
  
  /// Give back the word currently at the position of the file fichier in str.
  bool immediatlyAppears(FILE* fichier, const char *str) ;

  /// Look for the next keyword inside fich.
  int nextKeyWord(FILE* fich);
  
  /// Look for the next Joint keyword inside fich.
  int nextJointKeyWord(FILE* fichier);
  
  /// Returns the type of the next joint inside the file.
  int typeOfJoint(FILE* fichier);
};


#endif
