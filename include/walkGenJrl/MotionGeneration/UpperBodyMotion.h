/** @doc This object generate all the values for the foot trajectories,
   and the desired ZMP based on a sequence of relative steps.
   If you want to change the reference trajectories, and the planning
   of the foot, thOn pageis is the object to modify.

   Copyright (c) 2005-2006, 
   @author Bjorn Verrelst,
   Olivier Stasse
   
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

#ifndef _UPPER_BODY_MOTION_
#define _UPPER_BODY_MOTION_

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include <vector>
#include <string>
//#define FULL_POLYNOME

using namespace::std;

#include <walkGenJrl/Mathematics/PolynomeFoot.h>



namespace PatternGeneratorJRL
{
   
  class UpperBodyMotion
    {
      public :
    
      /// Constructor
      UpperBodyMotion();

      /// Destructor
      ~UpperBodyMotion();
      
      void GenerateDataFile(string aFileName, int LenghtDataArray);

      void ReadDataFile(string aFileName, MAL_MATRIX(&UpperBodyAngles,double));
      
      void WriteDataFile(string aFileName, MAL_MATRIX(&UpperBodyAngles,double));
	
    
   protected:


     
   };
};
#endif /* _UPPER_BODY_MOTION_*/
