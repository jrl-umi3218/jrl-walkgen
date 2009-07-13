/* Object to perform preview control on a cart model
   
   Copyright (c) 2005-2009, 
   @author Olivier Stasse, Ramzi Sellouati, Francois Keith
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   For more information on the license please look at License.txt 
   in the root directory.

   
*/
#ifndef _PREVIEW_CONTROL_H_
#define _PREVIEW_CONTROL_H_


#include <iostream>
#include <string>
#include <deque>
#include <vector>

#include <MatrixAbstractLayer/MatrixAbstractLayer.h>
using namespace::std;

#include <walkGenJrl_API.h>
#include <PGTypes.h>
#include <SimplePlugin.h>
#include <PreviewControl/OptimalControllerSolver.h>

namespace PatternGeneratorJRL
{


  /** @ingroup previewcontrol
      
      \brief Class to implement the preview control
   */
  class WALK_GEN_JRL_EXPORT PreviewControl : public SimplePlugin
    {
    public:
      
      /*! Constructor */
      PreviewControl(SimplePluginManager *lSPM,
		     unsigned int defaultMode = OptimalControllerSolver::MODE_WITH_INITIALPOS,
		     bool computeWeightsAutomatically = false);

      /*! Destructor */
      ~PreviewControl();

      /** \brief Read the file of parameters aFileName
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
      
      /*! \name Methods to access the basic variables of the preview control.
	@{
      */
      /*! \brief Getter for the sampling period. */
      double SamplingPeriod() const;
	
      /*! Getter for the preview control time. */
      double  PreviewControlTime() const;

      /*! Getter for the height position of the CoM. */
      double GetHeightOfCoM() const;

      /*! \brief Setter for the sampling period. */
      void SetSamplingPeriod(double lSamplingPeriod);
	
      /*! \biref Setter for the preview control time. */
      void SetPreviewControlTime(double lPreviewControlTime);

      /*! Getter for the height position of the CoM. */
      void SetHeightOfCoM(double lZc);
      /*! \brief Indicates if the weights are coherent with the parameters. */
      bool IsCoherent();

      /*! @} */

      /*! \brief Compute optimal weights.
	\param [in] mode: with initial pos (OptimalControllerSolver::MODE_WITH_INITIALPOS), 
	without initial position (OptimalControllerSolver::MODE_WITHOUT_INITIALPOS).
       */
      void ComputeOptimalWeights(unsigned int mode);

      /*! \brief Overloading of << operator. */
      void print();

      /*! \brief Overloading method of SimplePlugin */
      virtual void CallMethod(std::string &Method,
			      std::istringstream &astrm); 
    private:

      /*! \brief Matrices for preview control. */
      MAL_MATRIX(m_A,double);
      MAL_MATRIX(m_B,double);
      MAL_MATRIX(m_C,double);


      /** \name Control parameters. 
       @{ */

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

      /*! \brief Keep track of the modification of the preview parameters. */
      bool m_Coherent;

      /*! \brief Computes weight automatically */
      bool m_AutoComputeWeights;

      /*! \brief Default Mode. */
      unsigned int m_DefaultWeightComputationMode;
    };
}
#include <ZMPRefTrajectoryGeneration/ZMPDiscretization.h>
#endif /* _PREVIEW_CONTROL_H_ */
