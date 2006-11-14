/* This object is an abstract layer on the specificities of 
   a robot humanoid

   

   Copyright (c) 2005-2006, 
   Olivier Stasse,
   Ramzi Sellouati
   
   JRL-Japan, CNRS/AIST

   All rights reserved.
   
   Redistribution and use in source and binary forms, with or without modification, 
   are permitted provided that the following conditions are met:
   
   * Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright notice, 
   this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
   * Neither the name of the CNRS/AIST nor the names of its contributors 
   may be used to endorse or promote products derived from this software without specific prior written permission.
   
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
   AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER 
   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
   OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS 
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef _HUMANOID_SPECIFICITIES_H_
#define _HUMANOID_SPECIFICITIES_H_
#include <DynamicMultiBody.h>

namespace PatternGeneratorJRL
{
  /*@! This Object here implements the constants 
    and the specificities of HRP-2.
    */
  class HumanoidSpecificities
  {
  public:

    // Constructors
    HumanoidSpecificities();
    HumanoidSpecificities(DynamicMultiBody * aDMB);

    // Destructor
    ~HumanoidSpecificities();
    
    // Returns the width of the foot given in parameter:
    // @param WhichFoot : -1 Right foot 1 Left foot.
    // @paran Width: width of the foot, 
    // @param Height: height of the foot,
    // @return -1 if an error occured,
    //  0 otherwise.
    int GetFootSize(int WhichFoot, double &Width,double &Height);
    
    // Returns the width of the foot given in parameter:
    // @param aFileName : Name of the file where the humanoid parameters are stored.
    // @paran HumanoidName : Name of the humanoid. 
    int ReadXML(string & aFileName, string & HumanoidName);

    // Display the information stored inside the object.
    // Namely for debugging.
    void Display();

    // Returns the length of the tibia
    // @param WhichSide: -1 Right 1 Left.
    double GetTibiaLength(int WhichSide);

    // Returns the length of the femur
    // @param WhichSide: -1 Right 1 Left.
    double GetFemurLength(int WhichSide);

    // Returns the length of the Upper arm
    // @param WhichSide: -1 Right 1 Left.
    double GetUpperArmLength(int WhichSide);

    // Returns the length of the Fore arm
    // @param WhichSide: -1 Right 1 Left.    
    double GetForeArmLength(int WhichSide);

    // Returns the ankle position
    // @param WhichSide: -1 Right 1 Left.    
    // @return AnklePosition: (X,Y,Z)
    void GetAnklePosition(int WhichSide, double AnklePosition[3]);

    // Returns the position of the Hip regarding the waist's origin.
    // @param WhichSide: -1 Right 1 Left.
    // @ return WaistToHip translation.
    void GetWaistToHip(int WhichSide, double WaistToHip[3]);

  protected:
    
    // Store foot's height, width and depth.
    double m_FootHeight[2];
    double m_FootWidth[2];
    double m_FootDepth[2];

    // Store position of the ankles in the feet.
    double m_AnklePosition[2][3];

    // Pointer towards the dynamic multibody structure.
    DynamicMultiBody *m_DMB;
    
    // Tibia's length
    double m_TibiaLength[2];
    
    // Femur's length
    double m_FemurLength[2];

    // Upper arm's length.
    double m_UpperArmLength[2];

    // Forearm's length.
    double m_ForeArmLength[2];

    // Waist to hip translation
    double m_WaistToHip[2][3];
    
  };

};

#endif
