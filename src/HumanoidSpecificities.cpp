#include <HumanoidSpecificities.h>

using namespace PatternGeneratorJRL;

HumanoidSpecificities::HumanoidSpecificities()
{
  m_DMB = 0;

  m_FootHeight[0] = 0.137;
  m_FootHeight[1] = 0.137;
  m_FootWidth[0] = 0.24;
  m_FootWidth[1] = 0.24;
    
}

HumanoidSpecificities::HumanoidSpecificities(DynamicMultiBody *aDMB)
{

  HumanoidSpecificities();
  m_DMB = aDMB;    
}

HumanoidSpecificities::~HumanoidSpecificities()
{
}

int HumanoidSpecificities::GetFootSize(int WhichFoot, double & Width, double &Height)
{
  Width = 0.0;
  Height = 0.0;

  if (WhichFoot==-1)
    {
      Width = m_FootWidth[0];
      Height = m_FootHeight[0];
    }
  else if (WhichFoot==1)
    {
      Width = m_FootWidth[1];
      Height = m_FootHeight[1];
    }
  else 
    return -1;

  return 0;
}
