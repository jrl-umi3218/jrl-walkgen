/** @doc This object generate all the values for the foot trajectories,
   and the desired ZMP based on a sequence of relative steps.
   If you want to change the reference trajectories, and the planning
   of the foot, this is the object to modify.

   Copyright (c) 2005-2006, 
   @author Olivier Stasse,Ramzi Sellouati
   
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

#ifndef _WAISTHEIGHT_VARIATION_H_
#define _WAISTHEIGHT_VARIATION_H_

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>

#include <vector>
#include <string>
#include <Mathematics/Polynome.h>
#include <ZMPRefTrajectoryGeneration/ZMPDiscretization.h>
#include <PreviewControl/PreviewControl.h>
#include <deque>



//#include <PolynomeFoot.h>

namespace PatternGeneratorJRL
{

class WaistPolynome : public Polynome
    {
    public:
      /// Constructor:
      /// boundCond: the different boundary conditions begin, intermediate and end of polynomial
      /// timeDistr: vector with time instants for intermediate boundary conditions and end time
    
      WaistPolynome();
  
      /// Set the parameters
	void SetParameters(MAL_VECTOR( boundCond,double), vector<double> timeDistr);

      /// Destructor.
      ~WaistPolynome();
    };





  /// Object to compute new foot trajectories for the height of the waist with waist differnces as input for each step
  class WaistHeightVariation
    {
      public :
     
    	/// Constructor
      	WaistHeightVariation();

       	/// Destructor
      	~WaistHeightVariation();

	///call for polynomial planning of both steps during the obstacle stepover
	void PolyPlanner(deque<COMPosition> &aCOMBuffer,
			 deque<RelativeFootPosition> &aFootHolds,
			 deque<ZMPPosition> aZMPPosition);
	
	protected:

      	deque<RelativeFootPosition> m_FootHolds;

      	MAL_MATRIX(mBoundCond,double);   
      	vector<double> mTimeDistr;

    
      
      	WaistPolynome *m_PolynomeHip;
	
	///extra COMPosition buffer calculated in ZMPMultibody class 
	vector<COMPosition> m_ExtraCOMBuffer;

	

	/// buffers for first preview
	deque<COMPosition> m_COMBuffer;
	unsigned int m_ExtraBufferLength;
	double m_ModulationSupportCoefficient;
	float m_Tsingle,m_TsingleStepOver; 
        float m_Tdble;
	double m_DiffBetweenComAndWaist;
    	/// Sampling Period.
      	double m_SamplingPeriod;
      	/// Starting a new step sequences.
      	bool m_StartingNewSequence;
};

};

#endif /*_WAISTHEIGHT_VARIATION_H_ */
