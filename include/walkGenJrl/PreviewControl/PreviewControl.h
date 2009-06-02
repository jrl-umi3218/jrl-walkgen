/* Object to perform preview control on a cart model
    Copyright (c) 2005-2006, 
   @author Olivier Stasse, Ramzi Sellouati, Francois Keith
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the CNRS/AIST nor the names of its contributors 
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
#ifndef _PREVIEW_CONTROL_H_
#define _PREVIEW_CONTROL_H_


#include <iostream>
#include <string>
#include <deque>
#include <vector>

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>
using namespace::std;

#include <walkGenJrl/walkGenJrl_API.h>
#include <walkGenJrl/PGTypes.h>

namespace PatternGeneratorJRL
{


  /** @ingroup previewcontrol
      
      \brief Class to implement the preview control
   */
  class WALK_GEN_JRL_EXPORT PreviewControl
    {
    public:
      /*! Constructor */
      PreviewControl();

      /*! Destructor */
      ~PreviewControl();

      /** Read the file of parameters aFileName
	  and set the sampling period, the preview control time,
	  Ks, Kx, and F. */
      void ReadPrecomputedFile(string aFileName);
  

      /*! \brief One iteration of the preview control. */
      int OneIterationOfPreview(MAL_MATRIX(& x,double), 
				MAL_MATRIX(& y,double),
				double & sxzmp, double & syzmp,
				deque<PatternGeneratorJRL::ZMPPosition> & ZMPPositions,
				unsigned int lindex,
				double & zmpx2, double & zmpy2,
				bool Simulation);


      /*! \brief One iteration of the preview control along one axis (using queues)*/
      int OneIterationOfPreview1D(MAL_MATRIX( &x, double), 
				  double & sxzmp,
				  deque<double> & ZMPPositions,
				  unsigned int lindex,
				  double & zmpx2,
				  bool Simulation);

      /*! \brief One iteration of the preview control along one axis (using vectors) 
	\param [in][out] x: Current state of the CoM along the axis.
	\param [in][out] sxzmp: Summed error.
	\param [in] ZMPPositions: Vector of ZMP reference positions.
	\param [in] lindex: Starting index in the array of ZMP reference positions.
	\param [out] zmpx2: Resulting ZMP value.
	\param [in] Simulation: This should be set to false.
       */
      int OneIterationOfPreview1D(MAL_MATRIX( &x, double), 
				  double & sxzmp,
				  vector<double> & ZMPPositions,
				  unsigned int lindex,
				  double & zmpx2,
				  bool Simulation);

      /*! \brief Getter for the sampling period. */
      inline double SamplingPeriod() const
	{ return m_SamplingPeriod; }

      /*! Getter for the preview control time. */
      inline double  PreviewControlTime() const
	{ return m_PreviewControlTime; }

      /*! Getter for the height position of the CoM. */
      double GetHeightOfCoM();

      /*! Overloading of << operator. */
      void print();
      
    protected:

      /*! Matrices for preview control. */
      MAL_MATRIX(m_A,double);
      MAL_MATRIX(m_B,double);
      MAL_MATRIX(m_C,double);

      /** \name Control parameters. */

      /*! Gain on the current state of the CoM. */
      MAL_MATRIX(m_Kx,double);
      /*! Gain on the current ZMP. */
      double m_Ks;
      /*! Window  */
      MAL_MATRIX(m_F,double);
      //@}

      /* \name Preview parameters. */
      /**@{ */
       /*! Time for the preview control */
      double m_PreviewControlTime;
      
      /*! Sampling period for the preview  */
      double m_SamplingPeriod;

      /*! Size of the preview window. */
      unsigned int m_SizeOfPreviewWindow;
      
      /*! Height of the CoM. */
      double m_Zc;
      //@}
    };
}
#include <walkGenJrl/ZMPRefTrajectoryGeneration/ZMPDiscretization.h>
#endif /* _PREVIEW_CONTROL_H_ */
