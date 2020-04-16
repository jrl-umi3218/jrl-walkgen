// (c) Copyright LAAS CNRS 2019
// Olivier Stasse

#ifndef _TESTS_DUMP_REFERENCES_OBJECT_
#define _TESTS_DUMP_REFERENCES_OBJECT_
#include <fstream>

#include "CommonTools.hh"

namespace PatternGeneratorJRL {
namespace TestSuite {
class FillingFileArgs_t {
public:
  std::ofstream &aof;
  double dt;
  double nb_subsampling;
  struct OneStep &anOneStep;

  FillingFileArgs_t(std::ofstream &laof, double ldt, double lnb_subsampling,
                    struct OneStep &lOneStep)
      : aof(laof), dt(ldt), nb_subsampling(lnb_subsampling),
        anOneStep(lOneStep){};
};

class DumpReferencesObjects {
public:
  DumpReferencesObjects();
  virtual ~DumpReferencesObjects(){};

  void setAnklePositions(Eigen::Vector3d &AnklePositionRight,
                         Eigen::Vector3d &AnklePositionLeft);
  virtual void prepareFile(std::ofstream &aof, std::string &prefix,
                           struct OneStep &anOneStep);
  /// Fill-in tests
  /// \param[in] aTestName: Prefix of the test (typically algorithm name)
  /// \param[in] anOneStep: One iteration of the algorithm
  /// \param[in] aCurrentConfiguration: A configuration vector in
  /// RPY + motor angle format.
  /// This method is a front end for fillInTestFormat1 and fillInTestFormat2
  virtual void fillInTests(std::string &aTestName, struct OneStep &anOneStep,
                           Eigen::VectorXd &aCurrentConfiguration);

  ///
  virtual void fillInTestsFormat1(std::string &aTestName,
                                  struct OneStep &anOneStep,
                                  Eigen::VectorXd &aCurrentConfiguration);

  virtual void fillInTestsFormat2(std::string &aTestName,
                                  struct OneStep &anOneStep,
                                  Eigen::VectorXd &aCurrentConfiguration);

  virtual void
  fillFileWithSubsamplingAndClose(FillingFileArgs_t &aSetOfFillingFileArgs,
                                  std::vector<double> &next,
                                  std::vector<double> &prev);

  std::vector<double> m_prevCoMp, m_prevCoMpF2;
  std::vector<double> m_prevdCoMp;
  std::vector<double> m_prevddCoMp;
  std::vector<double> m_prevWaistOrien, m_prevWaistOrienF2;

  std::vector<double> m_prevLeftAnklePos, m_prevLeftAnklePosF2,
      m_prevLeftAnkledPos, m_prevLeftAnkleddPos, m_prevLeftAnkleOrientation,
      m_prevLeftAnkledOrientation, m_prevLeftAnkleddOrientation;

  std::vector<double> m_prevRightAnklePos, m_prevRightAnklePosF2,
      m_prevRightAnkledPos, m_prevRightAnkleddPos, m_prevRightAnkleOrientation,
      m_prevRightAnkledOrientation, m_prevRightAnkleddOrientation;

  std::vector<double> m_prevZMPlocal, m_prevZMPlocalF2, m_prevZMPRef,
      m_prevTorquesF2;

  Eigen::Vector3d m_AnklePositionRight, m_AnklePositionLeft;
  /// Add time or not in the dump file
  bool m_TimeOption;

private:
  unsigned int m_InternalFormat;
};

} // namespace TestSuite
} // namespace PatternGeneratorJRL

#endif /*  _TESTS_DUMP_REFERENCES_OBJECT_ */
