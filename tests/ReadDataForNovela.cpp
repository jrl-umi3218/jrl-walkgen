
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
enum FootEvent { FOOT_CREATION, FOOT_STARTING, FOOT_NOTHING, FOOT_SHIFT_ZMP};

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

  EventGenerationState state_,prevState_;
  /*! Double support phase duration */
  double tds_;
  /*! Double support phase duration */
  double prevTds_;
  /*! Single support phase duration */
  double tss_;
  /*! Previous single support phase duration */
  double prevTss_;

  /*! Absolute time of the take off. */
  double time_;

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
    prevSupportPoly_.y_ = (leftFoot_.y_+ rightFoot_.y_)/2.0;
    prevSupportPoly_.theta_ = (leftFoot_.theta_+ rightFoot_.theta_)/2.0;
  }

  
};

ostream & operator<<(ostream & os, const StateFromDiffFile & aState)
{
  os << "tds_: " << aState.tds_ << endl;
  os << "tss_: " << aState.tss_ << endl;
  if (aState.state_ == EGS_STARTING)
    os << "state_ : EGS_STARTING"  << std::endl;
  
  if (aState.state_ == EGS_UP)
    os << "state_ : EGS_UP"  << std::endl;
  
  if (aState.state_ == EGS_CONTACT)
    os << "state_ : EGS_CONTACT"  << std::endl;

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

    double alpha = 0.7;
    adatum.x_ *= alpha;
    adatum.y_ *= alpha;
    return !afile.eof();
  }
  
  FootEvent FilterEvent(StateFromDiffFile &aState,
			DataFromDiffFile &newDatum,
			DataFromDiffFile &prevDatum)
  {
    aState.prevState_ = aState.state_;

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
		aState.tss_ += newDatum.time_ - prevDatum.time_;
		aState.time_ = prevDatum.time_;
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
	      {
		aState.tds_ = 0.02;
		aState.tss_ = -0.02;
	      } else 	aState.tss_ = 0.0;

	  }
	if (aState.state_==EGS_STARTING)
	  {
	    aState.state_ = EGS_UP;
	    return FOOT_SHIFT_ZMP;
	  }
	aState.state_ = EGS_UP;
      }
    return FOOT_NOTHING;
  }

  void CreatesNewFoot(DataFromDiffFile & newDatum,
		      StateFromDiffFile &aDiffFileState,
		      ofstream & aof, ofstream & pyof,
		      unsigned long int &nbSeq,
		      ostringstream &ostrm)
  {
    // Information needed for normal step.
    DataFromDiffFile SupportFoot,NonSupportFoot;
    DataFromDiffFile prevSupportFootInNewFrame;
    ostringstream lstrm;
    
    // Information needed for additional step.
    bool AdditionalStep = false;
    ostringstream addstrm, addstrmforaof;

    // Who is support foot for the flying foot which
    // just landed.
    
    if (newDatum.foot_==0)
      {
	SupportFoot    = aDiffFileState.leftFoot_;
	NonSupportFoot = aDiffFileState.rightFoot_;
      }
    else 
      {
	SupportFoot    = aDiffFileState.rightFoot_;
	NonSupportFoot = aDiffFileState.leftFoot_;
      }

    // Compute the coordinates of the previous support foot in the
    // coordinates of the new one.
    SupportFoot.ExpressADatumInLocalCoordinates(aDiffFileState.prevSupportPoly_,
						prevSupportFootInNewFrame);

    // Check if the new support is right on the current one.
    // This may happen when the flying foot is the same 
    // on two successive steps.
    if ( (fabs(prevSupportFootInNewFrame.x_)<1e-06) &&
	 (fabs(prevSupportFootInNewFrame.y_)<1e-06) &&
	 (fabs(prevSupportFootInNewFrame.theta_)<1e-06))
      {
	std::cout << "prev to 0.0 0.0 0.0" << std::endl;
	
	AdditionalStep = true;
	DataFromDiffFile IntSupportFootInNewFrame;
	// Compute the coordinates of the previous support foot in the
	// coordinates of the flying foot which just landed.
	NonSupportFoot.ExpressADatumInLocalCoordinates(aDiffFileState.prevSupportPoly_,
						       IntSupportFootInNewFrame);
	
	// Creates a new feet

	// Building the sequence to send to PGI.
	addstrm << -IntSupportFootInNewFrame.x_ << " "
	      << -IntSupportFootInNewFrame.y_ << " "
	      << (-IntSupportFootInNewFrame.theta_*180/M_PI) << " "
	  // Here give a single support time of zero.
	      << "0.0 0.01 " ;

	addstrm << IntSupportFootInNewFrame.x_ << " "
	      << IntSupportFootInNewFrame.y_ << " "
	      << (IntSupportFootInNewFrame.theta_*180/M_PI) << " "
	  // Here give a single support time of zero.
	      << "0.0 0.01 " ;
	
	addstrmforaof << "    seqs.append(\"";
	addstrmforaof << -IntSupportFootInNewFrame.x_; addstrmforaof << " ";
	addstrmforaof << -IntSupportFootInNewFrame.y_; addstrmforaof << " ";
	addstrmforaof << (-IntSupportFootInNewFrame.theta_*180/M_PI); addstrmforaof << " ";
	// Here give a single support time of zero.
	addstrmforaof << "0.0 0.01 \")" <<endl;

	addstrmforaof << "    seqs.append(\"";
	addstrmforaof << IntSupportFootInNewFrame.x_; addstrmforaof << " ";
	addstrmforaof << IntSupportFootInNewFrame.y_; addstrmforaof << " ";
	addstrmforaof << (IntSupportFootInNewFrame.theta_*180/M_PI); addstrmforaof << " ";
	// Here give a single support time of zero.
	addstrmforaof << "0.0 0.01 \")" <<endl;

	
	NonSupportFoot.ExpressADatumInLocalCoordinates(aDiffFileState.prevSupportPoly_,
						       IntSupportFootInNewFrame);

	aDiffFileState.prevTss_ -= 0.02;
	aof << addstrmforaof.str();
      }
    else
      {    
    
	// If the foot is right after starting 
	// this means that this is a shift of the ZMP
	// and there was not foot before.
	// So the function waits for the next one.
	if (aDiffFileState.prevState_!=EGS_STARTING)
	  {
	    // Building the sequence to send to PGI.
	    lstrm << -prevSupportFootInNewFrame.x_ ; lstrm << " ";
	    lstrm << -prevSupportFootInNewFrame.y_; lstrm << " ";
	    lstrm << (-prevSupportFootInNewFrame.theta_*180/M_PI); lstrm << " ";
	    lstrm << aDiffFileState.prevTss_; lstrm << " ";
	    lstrm << aDiffFileState.prevTds_;
	    std::cout << -prevSupportFootInNewFrame.x_ << " "
		      << -prevSupportFootInNewFrame.y_ << " "
		      << -prevSupportFootInNewFrame.theta_ 
		      << std::endl;
	    ostrm << lstrm.str();
	    if (AdditionalStep)
	      {
		// ostrm << addstrm;
	      }
	    
	    // Storing this sequence in a file
	    aof << "    seqs.append(\"";
	    aof << -prevSupportFootInNewFrame.x_ ; aof << " ";
	    aof << -prevSupportFootInNewFrame.y_; aof << " ";
	    aof << (-prevSupportFootInNewFrame.theta_*180/M_PI); aof << " ";
	    aof << aDiffFileState.prevTss_; aof << " ";
	    aof << aDiffFileState.prevTds_;
	    aof << " \")" <<endl;
	    
	    /* 
	       if (AdditionalStep)
	       aof << addstrmforaof.str();
	    */
	    // Contact information generation
	    double tabs =  aDiffFileState.time_;
	    if (nbSeq==0) tabs=0;
	    string footname = (newDatum.foot_==0)?"RF":"LF";
	    double tup = tabs+aDiffFileState.tds_,
	      tdown = tabs+aDiffFileState.tss_+aDiffFileState.tds_;
	
	    
	    pyof << "attime(int(round(" <<tup<<"*T)),\t lambda:";
	    pyof << "rmcontact(contact"<<footname<<",task"<<footname ;
	    pyof <<"),'" << footname <<" take off " << nbSeq << " t="<<tup<<"')"<< std::endl;
	    
	    pyof << "attime(int(round(" << tdown <<"*T)),\t lambda:";
	    pyof << "contact(contact"<<footname<<",task"<<footname ;
	    pyof <<"),'" << footname <<" landing " << nbSeq << " t="<<tdown<<"')" << std::endl;
	  }
	// Update intermediate information
	nbSeq++;
	    
	aDiffFileState.prevTss_ = aDiffFileState.tss_;
	aDiffFileState.prevTds_ = aDiffFileState.tds_;
      }
      
  }

  void FillPGIWithDiff(PatternGeneratorInterface &aPGI)
  {
    
    ofstream pyof("novela_contact.py"); pyof << "T = 1/dt" << std::endl;
    ofstream aof;
    aof.open("DebugNovelaSteps.py");

    InitializePG(aPGI);

    aof << "@optionalparentheses" << endl;
    aof << "def seqH():" << endl;
    aof << "    pg.parseCmd(\":SetAlgoForZmpTrajectory KajitaOneState\")" << endl;
    aof << "    seqs = []" <<endl;
	
    ifstream aif;
    aif.open(m_FileName.c_str(),ifstream::in);
    if (!aif.is_open())
      throw std::string("Not able to open file.");

    ostringstream ostrm;
    DataFromDiffFile newDatum, prevDatum;
    
    CommonInitialization(aPGI);
    
    {
      istringstream strm2(":SetAlgoForZmpTrajectory KajitaOneStage");
      aPGI.ParseCmd(strm2);
    }

    ostrm << ":stepseq";
    ostrm << " ";
    unsigned long int nbData=0;
    unsigned long int nbSeq=0;
    StateFromDiffFile aDiffFileState;

    FootEvent prevCurrentFootEvent = FOOT_NOTHING;
    FootEvent prevPrevCurrentFootEvent = FOOT_NOTHING;

    while(ReadDataFromDiffFile(aif,newDatum))
      {
	FootEvent currentFootEvent = FilterEvent(aDiffFileState,
						 newDatum,
						 prevDatum);

	std::cout << "newDatum: " << newDatum << " "
		  << currentFootEvent << std::endl;

	if (currentFootEvent==FOOT_SHIFT_ZMP)
	  {
	    CreatesNewFoot(newDatum, aDiffFileState,aof,pyof,nbSeq,ostrm);
	    //aDiffFileState.updateFeetWhenNewContact(newDatum);
	  }

	if (currentFootEvent==FOOT_CREATION)
	  {

	    CreatesNewFoot(newDatum, aDiffFileState,aof,pyof,nbSeq,ostrm);
	    // Update State: TO BE DONE AFTER computing new support foot coordinates.
	    aDiffFileState.updateFeetWhenNewContact(newDatum);
	  }

	if (currentFootEvent==FOOT_STARTING)
	  {
	    // Update State
	    aDiffFileState.updateFeetWhenNewContact(newDatum);
	    if (nbData==1)
	      aDiffFileState.updateSupportPolyWhenStarting();
	  }

	if (!aif.eof())
	  ostrm << " ";

	prevDatum = newDatum;
	prevPrevCurrentFootEvent = prevCurrentFootEvent;
	prevCurrentFootEvent = currentFootEvent;
	nbData++;
      }
    
    aof << "    seqall = \"\"" << endl;
    aof << "    for aseq in seqs:" << endl;
    aof << "        seqall = seqall + aseq" << endl;
    aof << "    pg.parseCmd(\":walkmode 5\")" << endl;
    aof << "    pg.parseCmd(\":stepseq \" + seqall)" << endl;
    aof.close();

    cout << "ostrm gives:" << endl
	 << ostrm.str() <<endl;

    if (0)
    {
      istringstream strm2(ostrm.str());
      aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":stepseq 0.0 0.095 0.0 0.78 0.02 \
                     0.0 -0.19 -22.5 0.78 0.02  \
                     0.0 0.19 0.0 0.78 0.02 \
                     0.0 -0.19 -22.5 0.78 0.02  \
                     0.0 0.19 0.0 0.78 0.02 \
                     0.0 -0.19 -22.5 0.78 0.02  \
                     0.0 0.19 0.0 0.78 0.02 \
                     0.0 -0.19 -22.5 0.78 0.02  \
                     0.0 0.19 0.0 0.78 0.02 \
                     0.0 -0.19 -22.5 0.78 0.02  \
                     0.0 0.19 0.0 0.78 0.02 ");
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


