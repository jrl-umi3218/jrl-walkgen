/*
 * Copyright 2011, 
 *
 * Olivier Stasse
 *
 * JRL, CNRS/AIST
 *
 * This file is part of walkGenJrl.
 * walkGenJrl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * walkGenJrl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with walkGenJrl.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the 
 *  Joint Japanese-French Robotics Laboratory (JRL)
 */
/* \file This file tests A. Herdt's walking algorithm for
 * automatic foot placement giving an instantaneous CoM velocity reference.
 */
#include "CommonTools.h"
#include "TestObject.h"

#include <Debug.h>
#include <hrp2-dynamics/hrp2OptHumanoidDynamicRobot.h>

using namespace::PatternGeneratorJRL;
using namespace::PatternGeneratorJRL::TestSuite;
using namespace std;

enum contactevent { CE_CONTACT, CE_UP};
enum EventGenerationState {EGS_UP,EGS_CONTACT,EGS_STARTING};
enum FootEvent { FOOT_CREATION, FOOT_STARTING, FOOT_NOTHING};

class DataFromDiffFile
{

public:

  double time_;
  unsigned int foot_;
  double x_,y_,theta_;
  contactevent event_;

  DataFromDiffFile(): 
    time_(0.0),
    foot_(0),
    x_(0.0), y_(0.0), theta_(0.0), event_(CE_CONTACT) {}
  
  void ExpressADatumInLocalCoordinates(DataFromDiffFile &src,
				       DataFromDiffFile &dst);
};

ostream & operator<<(ostream & os, const DataFromDiffFile & datum)
{
  os << "x: " << datum.x_ << " "
     << "y: " << datum.y_ << " "
     << "theta: " << datum.theta_ << " "
     << "foot: " << datum.foot_ << " " 
     << "time: " << datum.time_ << " "
     << "event: " << datum.event_ << endl;
  return os;
}

void DataFromDiffFile::ExpressADatumInLocalCoordinates(DataFromDiffFile &src,
						       DataFromDiffFile &dst)
{
  matrix3d wMsrc,localMw, localMsrc;
  
  // Compute matrix from src frame to world frame.
  double ct = cos(src.theta_);
  double st = sin(src.theta_);
  wMsrc(0,0) = ct; wMsrc(0,1) = -st; wMsrc(0,2) = src.x_;
  wMsrc(1,0) = st; wMsrc(1,1) =  ct; wMsrc(1,2) = src.y_;
  wMsrc(2,0) =  0; wMsrc(2,1) =   0; wMsrc(2,2) = 1;
  
  // Compute matrix from world frame to local frame.
  double clt = cos(theta_);
  double slt = sin(theta_);
  localMw(0,0) = clt; localMw(0,1) = slt; localMw(0,2) = -clt*x_-slt*y_;
  localMw(1,0) =-slt; localMw(1,1) = clt; localMw(1,2) = slt*x_-clt*y_;
  localMw(2,0) =   0; localMw(2,1) =   0; localMw(2,2) = 1.0;
  
  // Compute matrix from src frame to local frame
  localMsrc = localMw * wMsrc;
  
  // Extract new coordinates
  dst.x_ = localMsrc(0,2); dst.y_ = localMsrc(1,2);
  dst.theta_ = atan2(localMsrc(1,0),localMsrc(0,0));
}


class StateFromDiffFile
{

public:
  DataFromDiffFile leftFoot_, rightFoot_, prevSupportPoly_;

  EventGenerationState state_;
  /*! Double support phase duration */
  double tds_;
  /*! Single support phase duration */
  double tss_;

  StateFromDiffFile():
    state_(EGS_STARTING), tds_(0.02), tss_(0.0) {}
  
  // Update foot position when a foot is landing.
  void updateFeetWhenNewContact(DataFromDiffFile newDatum)
  {
    if (newDatum.foot_==0)
      {
	rightFoot_ = newDatum;
	prevSupportPoly_ = leftFoot_;
      }
    else 
      {
	leftFoot_ = newDatum;
	prevSupportPoly_ = rightFoot_;
      }
  }

  // Update support polygone center position when in double support
  void updateSupportPolyWhenStarting()
  {
    prevSupportPoly_.x_ = (leftFoot_.x_+ rightFoot_.x_)/2.0;
    prevSupportPoly_.y_ = (leftFoot_.x_+ rightFoot_.y_)/2.0;
    prevSupportPoly_.theta_ = (leftFoot_.theta_+ rightFoot_.theta_)/2.0;
  }

  
};

ostream & operator<<(ostream & os, const StateFromDiffFile & aState)
{
  os << "tds_: " << aState.tds_ << endl;
  os << "tss_: " << aState.tss_ << endl;
  if (aState.state_ == EGS_STARTING)
    os << "aState.state_ : EGS_STARTING"  << std::endl;
  
  if (aState.state_ == EGS_UP)
    os << "aState.state_ : EGS_UP"  << std::endl;
  
  if (aState.state_ == EGS_CONTACT)
    os << "aState.state_ : EGS_CONTACT"  << std::endl;

  return os;
}

class ReadDataForNovela: public TestObject
{

private:
  string m_FileName;

public:
  ReadDataForNovela(int argc, char *argv[], string &aString, string FileName):
    TestObject(argc,argv,aString)
  {
    m_FileName = FileName;
  };


protected:

  virtual void SpecializedRobotConstructor(   CjrlHumanoidDynamicRobot *& aHDR,
					      CjrlHumanoidDynamicRobot *& aDebugHDR)
  {
    dynamicsJRLJapan::ObjectFactory aRobotDynamicsObjectConstructor;
    Chrp2OptHumanoidDynamicRobot *aHRP2HDR= new Chrp2OptHumanoidDynamicRobot(&aRobotDynamicsObjectConstructor);
    aHDR = aHRP2HDR;
    aDebugHDR = new Chrp2OptHumanoidDynamicRobot(&aRobotDynamicsObjectConstructor);
  }

  
  void InitializePG(PatternGeneratorInterface &aPGI)
  {
    CommonInitialization(aPGI);
    
    {
      istringstream strm2(":SetAlgoForZmpTrajectory KajitaOneStage");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":walkmode 5");
      aPGI.ParseCmd(strm2);
    }
  }

  void ReadDataFromFile(PatternGeneratorInterface &aPGI)
  {
    
    InitializePG(aPGI);

    ifstream aif;
    aif.open(m_FileName.c_str(),ifstream::in);
    if (!aif.is_open())
      throw std::string("Not able to open file.");

    ostringstream ostrm;

    ostrm << ":stepseq";
    ostrm << " ";
    while(!aif.eof())
      {
	double ltheta = 0.0;
	double lx,ly, lTss=0.0;
	aif >> lx;
	if (aif.eof())
	  {
	    cout << " break on lx " << endl;
	    break;
	  }

	aif >> ly;
	if (aif.eof())
	  {
	    cout << " break on ly " << endl;
	    break;
	  }
	aif >> ltheta;
	if (aif.eof())
	  {
	    cout << " break on ly " << endl;
	    break;
	  }		
	aif >> lTss;
	if (aif.eof())
	  {
	    cout << " break on lTss " << endl;
	    break;
	  }

	cout << "Reading: " <<lx << " " << ly << " " << lTss <<endl;
	double  lTds = 0.02;
	ostrm << lx ; ostrm << " ";
	ostrm << ly; ostrm << " ";
	ostrm << ltheta; ostrm << " ";
	ostrm << lTss; ostrm << " ";
	ostrm << lTds;
	if (!aif.eof())
	  ostrm << " ";
      }

    cout << "ostrm gives:" << endl
	 << ostrm.str() <<endl;
    
    {
      istringstream strm2(ostrm.str());
      aPGI.ParseCmd(strm2);
    }

  }

  bool ReadDataFromDiffFile (ifstream &afile,
			     DataFromDiffFile & adatum)
  {
    afile >> adatum.time_;
    afile >> adatum.foot_;
    afile >> adatum.x_;
    afile >> adatum.y_;
    afile >> adatum.theta_;
   
    if (adatum.x_==0.0 && adatum.y_==0.0 && adatum.theta_==0.0)
      adatum.event_ = CE_UP;
    else 
      adatum.event_ = CE_CONTACT;

    return !afile.eof();
  }
  
  FootEvent FilterEvent(StateFromDiffFile &aState,
			DataFromDiffFile &newDatum,
			DataFromDiffFile &prevDatum)
  {

    if (newDatum.time_==0.0)
      {
	aState.state_ = EGS_STARTING;
	return FOOT_STARTING;
      }

    // Define current state.
    if (aState.state_==EGS_UP)
      {
	if (newDatum.event_==CE_UP)
	  throw("Pb two consecutives up");
	
	if (newDatum.event_==CE_CONTACT)
	  {
	    aState.state_ = EGS_CONTACT;
	    if (prevDatum.foot_==newDatum.foot_)
	      {
		aState.tss_=newDatum.time_ - prevDatum.time_;
		return FOOT_CREATION;
	      }
	    else 
	      { throw("This foot is already down.");}

	  }
      }
    else 
      {
	if (newDatum.event_==CE_CONTACT)
	  throw("This is mistake both feet are already in contact.");
	if (newDatum.event_==CE_UP)
	  {
	    aState.tds_ = newDatum.time_ - prevDatum.time_;
	    if (aState.tds_<0.02)
	      aState.tds_ = 0.02;
	  }
	
	aState.state_ = EGS_UP;
      }
    return FOOT_NOTHING;
  }

  void FillPGIWithDiff(PatternGeneratorInterface &aPGI)
  {
    
    ofstream aof;
    aof.open("DebugNovelaSteps.dat");

    InitializePG(aPGI);

    ifstream aif;
    aif.open(m_FileName.c_str(),ifstream::in);
    if (!aif.is_open())
      throw std::string("Not able to open file.");

    ostringstream ostrm;
    DataFromDiffFile newDatum, prevDatum;

    ostrm << ":stepseq";
    ostrm << " ";
    unsigned long int nbData=0;
    
    StateFromDiffFile aDiffFileState;

    while(ReadDataFromDiffFile(aif,newDatum))
      {
	DataFromDiffFile prevSupportFootInNewFrame;

	FootEvent afe = FilterEvent(aDiffFileState,
				    newDatum,
				    prevDatum);

	if (afe==FOOT_CREATION)
	  {
	    DataFromDiffFile SupportFoot;

	    // Who is support foot the flying foot which
	    // just landed.
	    if (newDatum.foot_==0)
	      SupportFoot=aDiffFileState.leftFoot_;
	    else 
	      SupportFoot=aDiffFileState.rightFoot_;
		
	    // Compute the coordinates of the previous support foot in the
	    // coordinates of the new one.
	    SupportFoot.ExpressADatumInLocalCoordinates(aDiffFileState.prevSupportPoly_,
							prevSupportFootInNewFrame);
	
	    ostringstream lstrm;
	    lstrm << -prevSupportFootInNewFrame.x_ ; lstrm << " ";
	    lstrm << -prevSupportFootInNewFrame.y_; lstrm << " ";
	    lstrm << (-prevSupportFootInNewFrame.theta_*180/M_PI); lstrm << " ";
	    lstrm << aDiffFileState.tss_; lstrm << " ";
	    lstrm << aDiffFileState.tds_;
	    
	    aof << lstrm.str() << endl;
	    
	    ostrm << lstrm.str();

	    // Update State: TO BE DONE AFTER computing new support foot coordinates.
	    aDiffFileState.updateFeetWhenNewContact(newDatum);
	  }

	if (afe==FOOT_STARTING)
	  {
	    // Update State
	    aDiffFileState.updateFeetWhenNewContact(newDatum);
	    if (nbData==1)
	      aDiffFileState.updateSupportPolyWhenStarting();
	  }

	if (!aif.eof())
	  ostrm << " ";

	prevDatum = newDatum;
	nbData++;
      }
    
    aof.close();

    cout << "ostrm gives:" << endl
	 << ostrm.str() <<endl;
    
    {
      istringstream strm2(ostrm.str());
      aPGI.ParseCmd(strm2);
    }

  }


  void chooseTestProfile()
  {
    // ReadDataFromFile(*m_PGI);
    FillPGIWithDiff(*m_PGI);
  }
  
  void generateEvent()
  {
  }
};

int PerformTests(int argc, char *argv[])
{
  string TestName("TestReadDataForNovela");
  string FileName("ZMPTrajectory.dat");

  ReadDataForNovela aRDFN(argc,argv,
			  TestName,FileName);
  aRDFN.init();
  try
    {
      if (!aRDFN.doTest(std::cout))
	{
	  cout << "Failed test " << endl;
	  return -1;
	}
      else
	cout << "Passed test " <<  endl;
    }
  catch (const char * astr)
    { 
      cerr << "Failed on following error " << astr << std::endl;
      return -1; 
    }
  return 0;
}

int main(int argc, char *argv[])
{
  try
    {
      return PerformTests(argc,argv);
    }
  catch (const std::string& msg)
    {
      std::cerr << msg << std::endl;
    }
  return 1;
}


