#include <HumanoidDynamicMultiBody.h>

using namespace PatternGeneratorJRL;

HumanoidDynamicMultiBody::HumanoidDynamicMultiBody()
{
}

HumanoidDynamicMultiBody::~HumanoidDynamicMultiBody()
{

}

const MAL_S3_VECTOR(,double) & HumanoidDynamicMultiBody::zeroMomentumPoint() const
{
  return getZMP();
}
