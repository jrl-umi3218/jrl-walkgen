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
#include "CommonTools.hh"
#include "TestObject.hh"

#include <Debug.hh>
#include <hrp2-dynamics/hrp2OptHumanoidDynamicRobot.h>

using namespace::PatternGeneratorJRL;
using namespace::PatternGeneratorJRL::TestSuite;
using namespace std;


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


  void ReadDataFromFile(PatternGeneratorInterface &aPGI)
  {
    CommonInitialization(aPGI);

    {
      istringstream strm2(":SetAlgoForZmpTrajectory Kajita");
      aPGI.ParseCmd(strm2);
    }

    {
      istringstream strm2(":walkmode 5");
      aPGI.ParseCmd(strm2);
    }


    ifstream aif;
    aif.open(m_FileName.c_str(),ifstream::in);
    if (!aif.is_open())
      throw std::string("Not able to open file.");

    ostringstream ostrm;

    ostrm << ":stepseq";
    ostrm << " ";
    while(!aif.eof())
      {
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
		
	aif >> lTss;
	if (aif.eof())
	  {
	    cout << " break on lTss " << endl;
	    break;
	  }

	cout << "Reading: " <<lx << " " << ly << " " << lTss <<endl;
	double ltheta = 0.0, lTds = 0.0;
	ostrm << lx ; ostrm << " ";
	ostrm << ly; ostrm << " ";
	ostrm << ltheta; ostrm << " ";
	ostrm << lTss/200.0; ostrm << " ";
	ostrm << lTds/200.0;
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



  void chooseTestProfile()
  {
    ReadDataFromFile(*m_PGI);
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
    { cerr << "Failed on following error " << astr << std::endl;
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


