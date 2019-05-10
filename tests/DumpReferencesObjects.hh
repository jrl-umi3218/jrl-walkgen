// (c) Copyright LAAS CNRS 2019
// Olivier Stasse

#ifndef _TESTS_DUMP_REFERENCES_OBJECT_
#define _TESTS_DUMP_REFERENCES_OBJECT_
#include <fstream>


#include "CommonTools.hh"

namespace PatternGeneratorJRL
{
  namespace TestSuite
  {

    class DumpReferencesObjects
    {
    public:

      DumpReferencesObjects();
      virtual ~DumpReferencesObjects() {};

      void setAnklePositions(Eigen::Vector3d &AnklePositionRight,
			     Eigen::Vector3d &AnklePositionLeft);
      virtual void prepareFile(std::ofstream &aof,
			       std::string &prefix,
			       struct OneStep &anOneStep);
      virtual void fillInTests(std::string &aTestName,
			       struct OneStep &anOneStep,
			       Eigen::VectorXd &aCurrentConfiguration);

      virtual void fillInTestsFormat1
      (std::string &aTestName,
       struct OneStep &anOneStep,
       Eigen::VectorXd &aCurrentConfiguration);

      virtual void fillInTestsFormat2
      (std::string &aTestName,
       struct OneStep &anOneStep,
       Eigen::VectorXd &aCurrentConfiguration);

      virtual void fillFileWithSubsamplingAndClose
      (std::ofstream &aof,
       std::vector<double> &prev,
       std::vector<double> &next,
       double dt,
       double nb_subsampling,
       struct OneStep &anOneStep
       );

      std::vector<double> m_prevCoMp;
      std::vector<double> m_prevdCoMp;
      std::vector<double> m_prevddCoMp;
      std::vector<double> m_prevWaistOrien;

      std::vector<double> m_prevLeftAnklePos,
	m_prevLeftAnkledPos,
	m_prevLeftAnkleddPos,
	m_prevLeftAnkleOrientation,
	m_prevLeftAnkledOrientation,
	m_prevLeftAnkleddOrientation;

      std::vector<double> m_prevRightAnklePos,
	m_prevRightAnkledPos,
	m_prevRightAnkleddPos,
	m_prevRightAnkleOrientation,
	m_prevRightAnkledOrientation,
	m_prevRightAnkleddOrientation;

      std::vector<double> m_prevZMPlocal, m_prevZMPRef;

      Eigen::Vector3d m_AnklePositionRight,
	m_AnklePositionLeft;
      /// Add time or not in the dump file
      bool m_TimeOption;

    private:
      unsigned int m_InternalFormat;
    };

  } // TestSuite
} // PatternGeneratorJRL

#endif /*  _TESTS_DUMP_REFERENCES_OBJECT_ */
