/*! \file This object filters an analytical ZMP trajectory through preview control .

   Copyright (c) 2009, 
   Olivier Stasse

   JRL-Japan, CNRS/AIST
   Created: 2009/06/01    

   All rights reserved.
   
   For more information on the license please look at License.txt 
   in the root directory.

*/

#ifndef _FILTERING_ANALYTICAL_TRAJECTORY_BY_PREVIEW_CONTROL_H_
#define _FILTERING_ANALYTICAL_TRAJECTORY_BY_PREVIEW_CONTROL_H_

/*! Local framework include */
#include <MatrixAbstractLayer/MatrixAbstractLayer.h>
#include <walkGenJrl/Mathematics/AnalyticalZMPCOGTrajectory.h>
#include <walkGenJrl/PreviewControl/PreviewControl.h>

namespace PatternGeneratorJRL
{
  /*! \object This class intends to filter an analytical 
    trajectory using a preview control model. */

  class WALK_GEN_JRL_EXPORT FilteringAnalyticalTrajectoryByPreviewControl : public SimplePlugin
  {
  public:

    /*! \brief Default constructor */
    FilteringAnalyticalTrajectoryByPreviewControl(SimplePluginManager * lSPM,
						  AnalyticalZMPCOGTrajectory * lAnalyticalZMPCOGTrajectory=0,
						  PreviewControl * lPreviewControl=0);

    /*! \brief Set Analytical trajectory */
    void SetAnalyticalTrajectory(AnalyticalZMPCOGTrajectory *lAZCT);
    
    /*! \brief Set PreviewControl */
    void SetPreviewControl(PreviewControl *lPC);
    
    /*! \brief Fill in the whole buffer with the analytical trajectory. 
      This has to be done if the analytical trajectory has been changed,
      and that the first interval has been changed.
      \param FistValueOfZMPProfil: The first value of the desired ZMP interval.
      \param DeltaTj0: Value of the time interval during which the filter is applied.
      \return false if a problem occured, true otherwise.
     */
    bool FillInWholeBuffer(double FirstValueofZMPProfil,
			   double DeltaTj0 );

    /*! \brief Update the buffer by removing the first value in the queue,
      and adding a new one corresponding to the next control step.
    \param[in] t: The new time to be add. 
    \return false if a problem occured, true otherwise.
    */
    bool UpdateOneStep(double t, double &ZMPValue, double &CoMValue,double &CoMSpeedValue);

    /*! \brief Overloading method of SimplePlugin */
    virtual void CallMethod(std::string &Method,
			    std::istringstream &astrm); 

    /*! \brief Default destructor */
    ~FilteringAnalyticalTrajectoryByPreviewControl();
    
  private:

    /*! \brief The trajectory used for filtering. */
    AnalyticalZMPCOGTrajectory * m_AnalyticalZMPCOGTrajectory;

    /*! \brief Buffer of information for filtering. */
    std::vector<double> m_DataBuffer;

    /*! \brief Local index of the buffer. 
     -1 means that it was not yet correctly initialized.
    */
    int m_LocalBufferIndex;

    /*! \brief Pointer to the preview control object used to 
      filter. */
    PreviewControl *  m_PreviewControl;

    /*! \brief State of the CoM */
    MAL_MATRIX(m_ComState,double);

    /*! \brief Starting time of the filter. */
    double m_StartingTime;

    /*! \brief Duration of the filtering. */
    double m_Duration;

    /*! \brief Preview control time. */
    double m_PreviewControlTime;
    
    /*! \brief Sampling period. */
    double m_SamplingPeriod;

    /*! \brief Resizing the data buffer depending of the sampling period and
      preview control time. */
    void Resize();
  };
};
#endif /* _FILTERING_ANALYTICAL_TRAJECTORY_BY_PREVIEW_CONTROL_H_ */
