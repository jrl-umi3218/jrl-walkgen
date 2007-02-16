#include <HumanoidDynamicMultiBody.h>


int main()
{
  string aFileName = "HRP2Specificities.xml";
  DynamicMultiBody * aDMB = new DynamicMultiBody();
  string aPath="../../../etc/HRP2JRL/";
  string aName="HRP2JRLmain.wrl";
  aDMB->parserVRML(aPath,aName,"");
  HumanoidDynamicMultiBody *aHDMB = new HumanoidDynamicMultiBody(aDMB,aFileName);
  
  delete aDMB;
  delete aHDMB;
  
}
