/** \file WaistHeightVariation.h

    \brief This object generate all the values for the foot trajectories,
   and the desired ZMP based on a sequence of relative steps.
   If you want to change the reference trajectories, and the planning
   of the foot, this is the object to modify.

   Copyright (c) 2005-2009, 
   @author Olivier Stasse,Ramzi Sellouati, Francois Keith, 
   
   JRL-Japan, CNRS/AIST

   All rights reserved.

   Please see License.txt for further information on license.
*/

#ifndef _WAISTHEIGHT_VARIATION_H_
#define _WAISTHEIGHT_VARIATION_H_

#include <vector>
#include <string>
#include <deque>

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>


#include <Mathematics/Polynome.h>
#include <ZMPRefTrajectoryGeneration/ZMPDiscretization.h>
#include <PreviewControl/PreviewControl.h>



//#include <PolynomeFoot.h>

namespace PatternGeneratorJRL
{

class  WaistPolynome : public Polynome
    {
    public:
      /// Constructor:
      /// boundCond: the different boundary conditions begin, intermediate and end of polynomial
      /// timeDistr: vector with time instants for intermediate boundary conditions and end time
    
      WaistPolynome();
  
      /// Set the parameters
	void SetParameters(MAL_VECTOR( boundCond,double), std::vector<double> timeDistr);

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
      	std::vector<double> mTimeDistr;

    
      
      	WaistPolynome *m_PolynomeHip;
	
	///extra COMPosition buffer calculated in ZMPMultibody class 
	std::vector<COMPosition> m_ExtraCOMBuffer;

	

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
