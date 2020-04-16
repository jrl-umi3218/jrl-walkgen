/*
 * Copyright 2006, 2007, 2008, 2009, 2010,
 *
 * Florent Lamiraux
 * Mathieu Poirier
 * Olivier Stasse
 *
 * JRL, CNRS/AIST
 *
 * This file is part of walkGenJrl.
 * walkGenJrl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * walkGenJrl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with walkGenJrl.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the
 *  Joint Japanese-French Robotics Laboratory (JRL)
 */

/* \doc Object to perform preview control on a cart model */
#ifndef _PREVIEW_CONTROL_H_
#define _PREVIEW_CONTROL_H_

#include <deque>
#include <iostream>
#include <string>
#include <vector>

using namespace ::std;

#include <PreviewControl/OptimalControllerSolver.hh>
#include <SimplePlugin.hh>
#include <jrl/walkgen/pgtypes.hh>

namespace PatternGeneratorJRL {

/** @ingroup previewcontrol

    \brief Class to implement the preview control
*/
class PreviewControl : public SimplePlugin {
public:
  /*! Constructor */
  PreviewControl(
      SimplePluginManager *lSPM,
      unsigned int defaultMode = OptimalControllerSolver::MODE_WITH_INITIALPOS,
      bool computeWeightsAutomatically = false);

  /*! Destructor */
  ~PreviewControl();

  /** \brief Read the file of parameters aFileName
      and set the sampling period, the preview control time,
      Ks, Kx, and F. */
  void ReadPrecomputedFile(string aFileName);

  /*! \brief One iteration of the preview control. */
  int OneIterationOfPreview(
      Eigen::MatrixXd &x, Eigen::MatrixXd &y, double &sxzmp, double &syzmp,
      deque<PatternGeneratorJRL::ZMPPosition> &ZMPPositions,
      unsigned long int lindex, double &zmpx2, double &zmpy2, bool Simulation);

  /*! \brief One iteration of the preview control
    along one axis (using queues)*/
  int OneIterationOfPreview1D(Eigen::MatrixXd &x, double &sxzmp,
                              deque<double> &ZMPPositions,
                              unsigned long int lindex, double &zmpx2,
                              bool Simulation);

  /*! \brief One iteration of the preview control along one axis
    (using vectors)
    \param [in][out] x: Current state of the CoM along the axis.
    \param [in][out] sxzmp: Summed error.
    \param [in] ZMPPositions: Vector of ZMP reference positions.
    \param [in] lindex: Starting index in the array of ZMP reference
    positions.
    \param [out] zmpx2: Resulting ZMP value.
    \param [in] Simulation: This should be set to false.
  */
  int OneIterationOfPreview1D(Eigen::MatrixXd &x, double &sxzmp,
                              vector<double> &ZMPPositions,
                              unsigned long int lindex, double &zmpx2,
                              bool Simulation);

  /*! \name Methods to access the basic variables of the preview control.
    @{
  */
  /*! \brief Getter for the sampling period. */
  double SamplingPeriod() const;

  /*! Getter for the preview control time. */
  double PreviewControlTime() const;

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
    \param [in] mode: with initial pos
    (OptimalControllerSolver::MODE_WITH_INITIALPOS),
    without initial position (OptimalControllerSolver::
    MODE_WITHOUT_INITIALPOS).
  */
  void ComputeOptimalWeights(unsigned int mode);

  /*! \brief Overloading of << operator. */
  void print();

  /*! \brief Overloading method of SimplePlugin */
  virtual void CallMethod(std::string &Method, std::istringstream &astrm);

private:
  /*! \brief Matrices for preview control. */
  Eigen::MatrixXd m_A;
  Eigen::MatrixXd m_B;
  Eigen::MatrixXd m_C;

  /** \name Control parameters.
      @{ */

  /*! Gain on the current state of the CoM. */
  Eigen::MatrixXd m_Kx;
  /*! Gain on the current ZMP. */
  double m_Ks;
  /*! Window  */
  Eigen::MatrixXd m_F;
  //@}

  /* \name Preview parameters. */
  /**@{ */
  /*! Time for the preview control */
  double m_PreviewControlTime;

  /*! Sampling period for the preview  */
  double m_SamplingPeriod;

  /*! Size of the preview window. */
  long unsigned int m_SizeOfPreviewWindow;

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
} // namespace PatternGeneratorJRL
#include <ZMPRefTrajectoryGeneration/ZMPDiscretization.hh>
#endif /* _PREVIEW_CONTROL_H_ */
