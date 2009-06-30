/*!\file FootTrajectoryGenerationAbstract.h
   \brief This class determinate how it s generate all the values for the foot trajectories.

   @ingroup foottrajectorygeneration

   Copyright (c) 2007-2009, 
   @author Francois Keith, Olivier Stasse,

   $Id$
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Please see License.txt for further information on license.
*/
#include "FootTrajectoryGeneration/FootTrajectoryGenerationAbstract.h"

using namespace PatternGeneratorJRL;

FootTrajectoryGenerationAbstract::FootTrajectoryGenerationAbstract(SimplePluginManager *lSPM,
								   CjrlFoot *aFoot) 
  : SimplePlugin(lSPM)
{
  m_Omega = 0.0;
  m_Foot= aFoot;  
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


