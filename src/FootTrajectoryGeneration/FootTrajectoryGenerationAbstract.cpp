#include <walkGenJrl/FootTrajectoryGeneration/FootTrajectoryGenerationAbstract.h>

using namespace PatternGeneratorJRL;

FootTrajectoryGenerationAbstract::FootTrajectoryGenerationAbstract(SimplePluginManager *lSPM,
								   CjrlHumanoidDynamicRobot *aHS) 
  : SimplePlugin(lSPM)
{
  m_Omega = 0.0;
  m_HS= aHS;  
  m_SamplingPeriod = 0.005;

  string aMethodName[4] = 
    {":omega",":stepheight", ":singlesupporttime",":doublesupporttime"};

  for (int i=0;i<4;i++)
    {
      if (!RegisterMethod(aMethodName[i]))
	{
	  std::cerr << "Unable to register " << aMethodName << std::endl;
	}
    }

}

void FootTrajectoryGenerationAbstract::CallMethod(std::string &Method,
					     std::istringstream &strm)
{
  if (Method==":omega")
    {
      strm >> m_Omega;
    }
  else if (Method==":singlesupporttime")
    {
      strm >> m_TSingle;
    }
  else if (Method==":doublesupporttime")
    {
      strm >> m_TDouble;
    }

}


