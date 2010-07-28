#include <walkGenJrl/PGTypes.h>

namespace PatternGeneratorJRL
{
  struct COMPosition_s & COMPosition::operator=(const COMState_s &aCS)
  {
    for(unsigned int i=0;i<3;i++)
      {
	x[i] = aCS.x[i];
	y[i] = aCS.y[i];
	z[i] = aCS.z[i];
      };
    yaw     = aCS.yaw[0];  
    pitch   = aCS.pitch[0];
    roll    = aCS.roll[0];  
    return *this;
  }

  struct COMState_s & COMState::operator=(const COMPosition_s &aCS)
  {
    for(unsigned int i=0;i<3;i++)
      {
	x[i] = aCS.x[i];
	y[i] = aCS.y[i];
	z[i] = aCS.z[i];
      };
    yaw[0]   = aCS.yaw;   yaw[1]   = yaw[2]   = 0.0;
    pitch[0] = aCS.pitch; pitch[1] = pitch[2] = 0.0;
    roll[0]  = aCS.roll;  roll[1]  = roll[2]  = 0.0;
    return *this;
  }
      
  void COMState::reset() 
  {
    for(unsigned int i=0;i<3;i++)
      { 
	x[i] = 0.0; y[i] = 0.0;
	yaw[i] = 0.0; pitch[i] = 0.0; roll[i] = 0.0;
      }
  }

  COMState_s::COMState_s()
  {
    reset();
  }

}
