/*
 * Copyright 2010, 
 *
 * Andrei Herdt
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
/* \file This file tests A. Herdt's walking algorithm  modified by A. Bussy for
 * automatic foot placement giving an instantaneous CoM velocity reference.
 */
#include "CommonTools.h"
#include "TestObject.h"

#include <vector>

using namespace::PatternGeneratorJRL;
using namespace::PatternGeneratorJRL::TestSuite;
using namespace std;

enum Profiles_t {
  PROFIL_HERDT_ONLINE_WALKING                 // 1
};

class TestBussy2011: public TestObject
{

private:
  
  std::vector<double> velBuffer_;
  std::vector<double> posBuffer_;
  
  
public:
  TestBussy2011(int argc, char *argv[], string &aString, int TestProfile):
  //HACK: TestObject needs 2 args less than TestBussy so we change argc
    TestObject(argc-2,argv,aString)
  {
    m_TestProfile = TestProfile;

    double tmp;
    
    std::ifstream is(argv[6]);
    if(!is){ std::cerr << "Unable to open file!" << std::endl; }
    while(!is.eof()) {
	    is >> tmp;
// 	    if(tmp > 0.005)
// 	    std::cout << tmp << std::endl;
	    velBuffer_.push_back(tmp);
	    is >> tmp >> tmp;
    }
    //Matlab insert an empty line at the end of the data file.
    //It is parsed and gives random values, so we pop it back
    velBuffer_.pop_back();
    is.close();
    
    std::ifstream is2(argv[7]);
    if(!is2){ std::cerr << "Unable to open file!" << std::endl; }
    while(!is2.eof()) {
	    is2 >> tmp;
	    posBuffer_.push_back(tmp);
	    is2 >> tmp >> tmp;
    }
    //Matlab insert an empty line at the end of the data file.
    //It is parsed and gives random values, so we pop it back
    posBuffer_.pop_back();
    is2.close();
  };

protected:




  void startOnLineWalking(PatternGeneratorInterface &aPGI)
  {
    CommonInitialization(aPGI);

    {
      istringstream strm2(":SetAlgoForZmpTrajectory Herdt");
      aPGI.ParseCmd(strm2);

    }
    {
      istringstream strm2(":singlesupporttime 0.7");
      aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":doublesupporttime 0.1");
      aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":setPosReference  0.0 0.0 0.0");
      aPGI.ParseCmd(strm2);
    }
    {
//       istringstream strm2(":setMinPos  -0.2 -0.2 -99.0");
      istringstream strm2(":setMinPos  -99. -99. -99.0");
      aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":setMeanPos  0.0 0.0 0.0");
      aPGI.ParseCmd(strm2);
    }
    {
//       istringstream strm2(":setMaxPos  0.2 0.2 99.0");
      istringstream strm2(":setMinPos  99. 99. 99.0");
      aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":setweight position 0.");
      aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":HerdtOnline 0.03 0.0 0.0");
      aPGI.ParseCmd(strm2);
    }
    {
      istringstream strm2(":numberstepsbeforestop 1");
      aPGI.ParseCmd(strm2);
    }
  }

  void stopOnLineWalking(PatternGeneratorInterface &aPGI)
  {
    {
      istringstream strm2(":setVelReference  0.0 0.0 0.0");
      aPGI.ParseCmd(strm2);
      istringstream strm3(":stoppg");
      aPGI.ParseCmd(strm3);
    }
  }


  void chooseTestProfile()
  {

    switch(m_TestProfile)
      {

      case PROFIL_HERDT_ONLINE_WALKING:
	startOnLineWalking(*m_PGI);
	break;
      default:
	throw("No correct test profile");
	break;
      }
  }

  void generateEvent()
  {

    typedef void (TestBussy2011::* localeventHandler_t)(PatternGeneratorInterface &);

    struct localEvent 
    {
      unsigned time;
      localeventHandler_t Handler ;
    };

    #define localNbOfEvents 1
    struct localEvent events [localNbOfEvents] =
      { {25*200,&TestBussy2011::stopOnLineWalking}};
    
    // Test when triggering event.
    for(unsigned int i=0;i<localNbOfEvents;i++)
      { 
	if ( m_OneStep.NbOfIt==events[i].time)
	  {
	    (this->*(events[i].Handler))(*m_PGI);
	  }
      }
//     std::cout << m_OneStep.finalCOMPosition.x[0] << std::endl;
    //Set velReference and PosReference
    if(m_OneStep.NbOfIt < velBuffer_.size()) {
      m_PGI->setVelocityReference( velBuffer_.at(m_OneStep.NbOfIt), 0., 0. );
      
//       std::cout << velBuffer_.at(m_OneStep.NbOfIt) << std::endl;
    }
    else
      m_PGI->setVelocityReference( 0., 0., 0. );
    if(m_OneStep.NbOfIt < posBuffer_.size())
      m_PGI->setPositionReference( posBuffer_.at(m_OneStep.NbOfIt), 0., 0. );
    else
      m_PGI->setPositionReference( posBuffer_.back(), 0., 0. );
      
  }
};

int PerformTests(int argc, char *argv[])
{

  const unsigned NbTests = 1;
  std::string TestNames[NbTests] = { "TestBussy2011OnLine" };
  int TestProfiles[NbTests] = { PROFIL_HERDT_ONLINE_WALKING };

  for (unsigned i=0;i<NbTests;i++)
    {
      TestBussy2011 testProg(argc,argv,
			    TestNames[i],
			    TestProfiles[i]);
      try
	{
	  if (!testProg.doTest(std::cout))
	    {
	      cout << "Test nb. " << i+1 <<" (of " << NbTests <<" test(s)) failed!!!"<< endl;
	      return -1;
	    }
	  else
	    cout << "Test nb. " << i+1 <<" (of " << NbTests <<" test(s)) passed."<< endl;
	}
      catch (const char * astr)
	{ cerr << "Failed on following error " << astr << std::endl;
	  return -1; }
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


