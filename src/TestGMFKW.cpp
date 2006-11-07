//
// C++ Implementation: TestGMFKW
//
// Description: 
//
//
// Author: Olivier STASSE <olivier.stasse@aist.go.jp>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include <fstream>
#include <GenerateMotionFromKineoWorks.h>

int main()
{
  
  double sx=0.2,sy=0.19,Tsingl=0.78, Tdble =0.1;
//  int walkmode = 2;

  PatternGeneratorJRL::RelativeFootPosition TabFoots2[17] = {
    { 0.0, -sy/2, 0.0, Tsingl, Tdble, 1,  0.0},
    {  sx,  sy,   0.0, Tsingl, Tdble, 1,  0.0},  // 0.2
    {  sx, -sy,   0.0, Tsingl, Tdble, 1,  0.0},  // 0.4
    {  sx,  sy,   0.0, Tsingl, Tdble, 1,  0.0},  // 0.6
    {  sx, -sy,   0.0, Tsingl, Tdble, 1,  0.0},  // 0.8
    {  sx,  sy,   0.0, Tsingl, Tdble, 1,  0.0},  // 1.0
    {  sx, -sy,   0.0, Tsingl, Tdble, 1,  0.0},  // 1.2
    {  sx,  sy,   0.0, Tsingl, Tdble, 1,  0.0},  // 1.4
    {  sx, -sy,   0.0, Tsingl, Tdble, 1, -0.2},  // 1.6
    {  sx,  sy,   0.0, Tsingl, Tdble, 1, -0.2},  // 1.8
    {  sx, -sy,   0.0, Tsingl, Tdble, 1, -0.2},  // 2.0
    {  sx,  sy,   0.0, Tsingl, Tdble, 1, -0.2},  // 2.2 
    {  sx, -sy,   0.0, Tsingl, Tdble, 1, -0.15}, // 2.4
    {  sx,  sy,   0.0, Tsingl, Tdble, 1, -0.1},  // 2.6
    {  sx, -sy,   0.0, Tsingl, Tdble, 1, -0.05}, // 2.8
    {  sx,  sy,   0.0, Tsingl, Tdble, 1,  0.0},  // 3.0
    { 0.0, -sy,   0.0, Tsingl, Tdble, 1,  0.0}
    
  };

  
  PatternGeneratorJRL::ZMPDiscretization * aZMPD;
  PatternGeneratorJRL::PreviewControl * aPC;
  PatternGeneratorJRL::GenerateMotionFromKineoWorks * aGMFKW;
  deque<PatternGeneratorJRL::ZMPPosition> ZMPPositions;
  deque<PatternGeneratorJRL::FootAbsolutePosition> FootAbsolutePositions;
  deque<PatternGeneratorJRL::RelativeFootPosition> RelativeFootPositions;
  deque<PatternGeneratorJRL::FootAbsolutePosition> LeftFootPositions,RightFootPositions;
  deque<PatternGeneratorJRL::FootAbsolutePosition> LeftHandPositions,RightHandPositions;
  deque<PatternGeneratorJRL::KWNode > UpperBodyPositionsBuffer;
  vector<int> ConversionFromLocalIndexToRobotDOFs;
  double Xmax=0.0;
  
  // Initialize the foot sequence.
  for(int i=0;i<17;i++)
    RelativeFootPositions.push_back(TabFoots2[i]);
  
  // First create the buffer of ZMP reference value for each 0.005 ms.
  aZMPD = new PatternGeneratorJRL::ZMPDiscretization();
  
  VNL::Vector<double> lSCOM(3,0.0);

  PatternGeneratorJRL::FootAbsolutePosition InitLFPos, InitRFPos;
  bzero(&InitLFPos,sizeof(InitLFPos));
  bzero(&InitRFPos,sizeof(InitRFPos));

  aZMPD->GetZMPDiscretization(ZMPPositions,
                              FootAbsolutePositions,
                              RelativeFootPositions,
                              LeftFootPositions,
                              RightFootPositions,
                              LeftHandPositions,
                              RightHandPositions,
                              Xmax,lSCOM,InitLFPos,
			      InitRFPos);
  
  // Create and initialize the Preview control object.
  aPC = new PatternGeneratorJRL::PreviewControl();
  
  string PCParameters="PreviewControlParameters.ini";
  aPC->ReadPrecomputedFile(PCParameters);

  // Create and initialize the Motion generator from KineoWorks.
  aGMFKW = new PatternGeneratorJRL::GenerateMotionFromKineoWorks();
  aGMFKW->SetPreviewControl(aPC);
  
  string aPartialModel="PartialModel.dat";
  string aKWPath="KWBarPath.pth";

  if (aGMFKW->ReadPartialModel(aPartialModel)<0)
    cerr<< "Error while reading partial model " << endl;

  if (aGMFKW->ReadKineoWorksPath(aKWPath)<0)
    cerr<< "Error while reading the path " << endl;	
  
  aGMFKW->DisplayModelAndPath();
  
  // Map the path found by KineoWorks onto the ZMP buffer.
  UpperBodyPositionsBuffer.clear();
  ConversionFromLocalIndexToRobotDOFs.clear();
  
  aGMFKW->CreateBufferFirstPreview(ZMPPositions);
  aGMFKW->ComputeUpperBodyPosition(UpperBodyPositionsBuffer,
				   ConversionFromLocalIndexToRobotDOFs);
  
  std::ofstream aof;
  aof.open("output.dat",ofstream::out);
  if (aof.is_open())
  {
    for(unsigned int i=0;i<UpperBodyPositionsBuffer.size();i++)
    {
      for(unsigned int j=0;j<UpperBodyPositionsBuffer[i].Joints.size();j++)
	aof << UpperBodyPositionsBuffer[i].Joints[j] << " " ;
      aof << 
      aof<< endl;
    } 
    aof.close();
  }
}
