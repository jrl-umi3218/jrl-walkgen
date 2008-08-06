/** @doc This object generate all the values for the foot trajectories,
    and the desired ZMP based on a sequence of steps.
   
    Copyright (c) 2005-2006, 
    @author Olivier Stasse, Ramzi Sellouati, Bjorn Verrelst
   
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

#if 0 /* MAL 2.0 */
#include <MatrixAbstractLayer/boost.h>
namespace ml = maal::boost;
#else
#include <MatrixAbstractLayer/MatrixAbstractLayer.h>
#endif

#include "plugin.h"
#include "bodyinfo.h"
#include "seqplugin.h" 
#ifdef ORBIX
#include <walkpluginJRL_skel.h>
#endif
#ifdef OMNIORB4
#include <walkpluginJRL.h>
#endif
#include <fstream>
#include <walkGenJrl/PatternGeneratorInterface.h>


#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "WalGenJRLIntegrate :" << x << endl

#if 0
#define ODEBUG(x) cerr << "WalkGenJRLIntegrate :" <<  x << endl
#else
#define ODEBUG(x) 
#endif

#if 1
#define RESETDEBUG4(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG4(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "WalkGenJRL: " << x << endl; DebugFile.close();}
#define _DEBUG_4_ACTIVATED_ 1 
#else
#define ODEBUG4(x,y)
#define RESETDEBUG4(y)
#endif

using namespace::PatternGeneratorJRL;
	
class WalkGenJRL : public plugin,
virtual public POA_walkpluginJRL, virtual public PortableServer::RefCountServantBase
 {// inherite plugin
public: 
  WalkGenJRL(istringstream &strm);
  ~WalkGenJRL();
  bool setup(robot_state* rs, motor_command* mc);  // phase:waiting_setup 
  void control(robot_state* rs, motor_command* mc);// phase:active 
  bool cleanup(robot_state* rs, motor_command* mc);// phase:waiting_cleanup  
  void m_echo(istringstream &strm); 
  void m_StepSequence(istringstream &strm);

   void Matrix2Quaternion(MAL_MATRIX(,double) R,double q[4]);

   // Interface for hrpsys.
   void m_ParseCmd(istringstream &strm);

  void m_DumpZMPreel(istringstream &strm);
  void m_ReadFileFromKineoWorks(istringstream &strm);
  void m_Synchronize(istringstream &strm);

  // This method only update the Step Sequence stack.
  void m_PartialStepSequence(istringstream &strm);




  // This method is doing the real job for preparing the
  // support foot.
  void PrepareForSupportFoot(int SupportFoot);

  // Interface to hrpsys to start the realization of 
  // the stacked of step sequences.
  void m_FinishAndRealizeStepSequence(istringstream &strm);

  // The method which is really creating the buffer 
  // related to the stacked step sequences.
  void FinishAndRealizeStepSequence();

  // Prepare to start or stop on a given support foot.
  // (Just an interface)
  void m_PrepareForSupportFoot(istringstream &strm);

  // This method is finishing on the last support foot 
  // after the last motion.
  void m_FinishOnTheLastCorrectSupportFoot(istringstream &strm);

  
 // Send the stack of ZMP information to the control loop.
   void m_SendStackToControl(istringstream &strm);

  //! Interface for CORBA

  /// 
  void setTargetPos(CORBA::Float x,
		    CORBA::Float y,
		    CORBA::Float th)
    throw(CORBA::SystemException);

  void setTargetPosNoWait(CORBA::Float x,
			  CORBA::Float y,
			  CORBA::Float th)
    throw(CORBA::SystemException);

  void setArc(CORBA::Float x,
	      CORBA::Float y,
	      CORBA::Float th)
    throw(CORBA::SystemException) ;

  void setArcNoWait(CORBA::Float x,
		    CORBA::Float y,
		    CORBA::Float th)
    throw(CORBA::SystemException);

  void setRfootPos(CORBA::Float x,
		   CORBA::Float y,
		   CORBA::Float th)
    throw(CORBA::SystemException) ;

  void setRfootPosNoWait(CORBA::Float x,
			 CORBA::Float y,
			 CORBA::Float th)
    throw(CORBA::SystemException);
  
  void setLfootPos(CORBA::Float x,
		   CORBA::Float y,
		   CORBA::Float th)
    throw(CORBA::SystemException);

  void setLfootPosNoWait(CORBA::Float x,
			 CORBA::Float y,
			 CORBA::Float th)
    throw(CORBA::SystemException);

  void stopWalking()
    throw(CORBA::SystemException);

  void waitArrival()
    throw(CORBA::SystemException);
  
  void startStepping()
    throw(CORBA::SystemException);
  
  void stopStepping()
    throw(CORBA::SystemException);
  
  void setWalkingVelocity(CORBA::Float dx,
			  CORBA::Float dy,
			  CORBA::Float dth)
    throw(CORBA::SystemException);
  
  void getWaistVelocity(CORBA::Float_out vx,
			CORBA::Float_out vy,
			CORBA::Float_out omega)
    throw(CORBA::SystemException);

  void getWaistAcceleration(TransformQuaternion_out aTQ)
    throw(CORBA::SystemException);

  void getWaistPositionAndOrientation(TransformQuaternion_out aTQ,
				      CORBA::Float_out Orientation)
    throw(CORBA::SystemException);

  void setWaistPositionAndOrientation(const TransformQuaternion& aTQ)
    throw(CORBA::SystemException);

  CORBA::Long getLegJointSpeed(dsequence_out dq)
    throw(CORBA::SystemException);

  CORBA::Boolean isWalking() 
    throw(CORBA::SystemException);

protected:

  // The pattern generator interface.
  PatternGeneratorInterface * m_PGI;
	
  bool m_ObstacleDetected;	
	
  int m_DebugMode;
  bool m_ShouldBeRunning;
  ofstream m_arof, m_alof, m_acomof;  

  double m_zmpreel[3*200*120];
  int m_ZMPIndex, m_ZMPIndexmax;
  double m_generalbuffer[3*200*120];
  int m_generalIndex, m_generalIndexmax;
  double m_zmpreeldebug[3],m_zmptarget[3];
  seqplugin_var m_seq;
  unsigned long int m_count;

  bool m_SetupBool;

  MAL_VECTOR(m_CurrentJointValues,double);

  /*! Current value of the joints from the Humanoid Walking Pattern Generator. */
  MAL_VECTOR(m_CurrentStateFromPG,double);

  /*! Current speed of the joints from the Humanoid Walking Pattern Generator. */
  MAL_VECTOR(m_CurrentVelocityFromPG,double);

  // Position of the waist:
  // Relative:
  MAL_MATRIX(,double) m_WaistRelativePos;

  // Absolute:
  MAL_MATRIX(,double) m_WaistAbsPos;

  // Orientation:
  double m_AbsTheta, m_AbsMotionTheta;

  // Position of the motion:
  MAL_MATRIX(,double) m_MotionAbsPos;
  MAL_MATRIX(,double) m_MotionAbsOrientation;

  // Absolute speed:
  MAL_MATRIX(,double) m_AbsLinearVelocity;
  MAL_MATRIX(,double) m_AbsAngularVelocity;

  // Absolute acceleration
  MAL_MATRIX(,double) m_AbsLinearAcc;
 
  // Current Waist state return by the Pattern Generator Interface.
  COMPosition m_CurrentWaistState;

};

plugin* create_plugin(istringstream &strm) 
{// for dynamic loading
  return new WalkGenJRL(strm);
}

WalkGenJRL::WalkGenJRL(istringstream &strm)
{

  m_ShouldBeRunning = false;

  m_DebugMode = 1;

  m_PGI = new PatternGeneratorInterface(strm);

  // Register method the hrpsys interpreter.
  register_method(":parsecmd",(method)&WalkGenJRL::m_ParseCmd);
  register_method(":echo",(method)&WalkGenJRL::m_echo);
  register_method(":synchronize",(method)&WalkGenJRL::m_Synchronize);
  register_method(":stepseq",(method)&WalkGenJRL::m_StepSequence);
  register_method(":dumpzmpreel",(method)&WalkGenJRL::m_DumpZMPreel);
  register_method(":readfilefromkw",(method)&WalkGenJRL:: m_ReadFileFromKineoWorks);
  register_method(":pstepseq",(method)&WalkGenJRL::m_PartialStepSequence);
  register_method(":SendStackToControl",(method)&WalkGenJRL::m_SendStackToControl);

  m_CurrentJointValues.resize(40);
  m_CurrentStateFromPG.resize(46);
  m_CurrentVelocityFromPG.resize(46);

  m_ZMPIndex = 0;
  m_ZMPIndexmax = 3*200*120;

  m_generalIndex = 0;
  m_generalIndexmax = 3*200*120;

  RESETDEBUG4("WPDebugDataql.dat");
  RESETDEBUG4("Debug5.dat");
  RESETDEBUG4("DebugZMPFinale.dat");
  RESETDEBUG4("DebugData.txt");
}

WalkGenJRL::~WalkGenJRL()
{
  if (m_PGI!=0)
    delete m_PGI;
}

void WalkGenJRL::m_ParseCmd(istringstream &strm)
{
  if (m_PGI!=0)
    {
      ODEBUG4("Before ParseCmd:","DebugData.txt");
      m_PGI->ParseCmd(strm);
      ODEBUG4("After ParseCmd:","DebugData.txt");
    }
}

void WalkGenJRL::m_echo(istringstream &strm)
{
  string msg = "echo";
  strm >> msg;
  cout << "WalkGenJRL(";
  if (isRunning()) 
    cout << " not active ";
  else 
    cout << " active ";
  cout << "):" << msg << endl;
}

void WalkGenJRL::m_Synchronize(istringstream &strm)
{
  while(m_ShouldBeRunning)
    {
      usleep(10000);
    }
  
}

void WalkGenJRL::m_StepSequence(istringstream &strm)
{
  if (m_PGI!=0)
    m_PGI->m_StepSequence(strm);
}

bool WalkGenJRL::setup(robot_state *rs, motor_command *mc) 
{
  
  m_seq = seqplugin::_narrow(manager->find("seq",""));
  // Store the current motor command
  for(unsigned int i=0;i<DOF;i++)
    {	
      ODEBUG4( "setup from robot for first time :" <<  i 
	       << " " << (mc->angle[i]*180/M_PI) 
	       << MAL_VECTOR_SIZE(m_CurrentJointValues) << " "
	       << m_PGI, "DebugData.txt");
      m_CurrentJointValues(i) = rs->angle[i];
    }
  if (m_PGI!=0)
    m_PGI->SetCurrentJointValues(m_CurrentJointValues);
  return true; 				    
}

void WalkGenJRL::control(robot_state *rs, motor_command *mc) 
{
	
  MAL_VECTOR_DIM(ZMPTarget,double,3);
  RobotState_var ref_state;	
  // Store the current motor command
  for(unsigned int i=0;i<DOF;i++)
    {
      ODEBUG4( "EvalueCOM read from robot for first time :" <<  i << " " << (mc->angle[i]*180/M_PI), "DebugData.txt");
      m_CurrentJointValues(i) = 
	m_CurrentStateFromPG(i) =
	mc->angle[i];
      
    }

  if (m_PGI!=0)
    m_PGI->SetCurrentJointValues(m_CurrentJointValues);
  
  ODEBUG4("WalkGenJRL : m_ShouldBeRunning" << m_ShouldBeRunning,"DebugData.txt");
  if (!m_ShouldBeRunning)
    return;

  ODEBUG4("WalkGenJRL still running","DebugData.txt");

  if (m_PGI->RunOneStepOfTheControlLoop(m_CurrentStateFromPG,
					m_CurrentVelocityFromPG,
					ZMPTarget))
    {    
      ODEBUG4("Go inside","DebugData.txt");
      if (rs->zmp.length()>0)
	{
	  for(unsigned int i=0;i<3;i++)
	    m_zmpreel[m_ZMPIndex+i] = rs->zmp[i]+mc->waistPos[i];
	  m_ZMPIndex+=3;
	  if (m_ZMPIndex>=m_ZMPIndexmax)
	    m_ZMPIndex = 0;
	}

      if (m_seq) 
	{ 
	  m_seq->getReferenceState(ref_state);
	}
				
      if (m_seq) 
	{            

	  // if seq exists use the ReferenceState
	  for(unsigned int i=0;i<40;i++)
	    {
	      ref_state->angle[i]=m_CurrentStateFromPG(i+6);
	    }
		
	  ODEBUG4( m_CurrentStateFromPG(0+6)*180.0/M_PI 
		   << " " << m_CurrentStateFromPG(1+6) *180.0/M_PI  
		   << " " << m_CurrentStateFromPG(2+6) *180.0/M_PI 
		   << " " << m_CurrentStateFromPG(3+6) *180.0/M_PI 
		   << " " << m_CurrentStateFromPG(4) *180.0/M_PI,
		   "WPDebugDataql.dat");
	    
	  //ZMP ref. in the waist reference frame.
	

	  m_zmptarget[0] = ZMPTarget(0);
	  m_zmptarget[1] = ZMPTarget(1);
	  m_zmptarget[2] = ZMPTarget(2);
		
	  ODEBUG4( ZMPTarget(0) << " " << ZMPTarget(1) << " " << ZMPTarget(2),"Debug5.dat");
	  memcpy(ref_state->zmp.get_buffer(),m_zmptarget,sizeof(double)*3);     // just change zmp

	  //cout << mc->waistPos[2] << " "
	  //<< m_Zc << " " << aCOMPosition.z[0] << endl;
	  
	  double aTQ[7],WaistAbsOrientation;
	  m_PGI->getWaistPositionAndOrientation(aTQ,WaistAbsOrientation);
	  //ref_state->waistPos[0] = m_CurrentWaistState.x[0];
	  //	  ref_state->waistPos[1] = m_CurrentWaistState.y[0];
	  //	  ref_state->waistPos[2] = m_CurrentWaistState.z[0];//m_Zc -m_DiffBetweenComAndWaist;

	  ref_state->waistPos[0] = aTQ[0];
	  ref_state->waistPos[1] = aTQ[1];
	  ref_state->waistPos[2] = aTQ[2];
	  ref_state->waistRpy[2] = WaistAbsOrientation;

	  //cout << ref_state->waistPos[2] << endl;

	  if (m_DebugMode>0)
	    {
	      double temp1;
	      double temp2;
	      double temp3;
	     
	      m_CurrentWaistState.x[0] = aTQ[0];
	      m_CurrentWaistState.y[0] = aTQ[1];
	      m_CurrentWaistState.z[0] = aTQ[2];
	      m_CurrentWaistState.yaw = WaistAbsOrientation;

	      temp3 = m_CurrentWaistState.yaw*M_PI/180.0;
	      // This does the inverse of the transformation perform inside PGI. (use the transpose of the matrix).
	      temp1 = m_zmptarget[0] * cos(temp3) + m_zmptarget[1] * -sin(temp3) ;
	      temp2 = m_zmptarget[0] * sin(temp3) + m_zmptarget[1] * cos(temp3) ;

	      ODEBUG4( m_zmptarget[0] << " " << 
		       m_zmptarget[1] << " " <<
		       m_zmptarget[2] << " " <<
		       temp1 + ref_state->waistPos[0] << " " << 
		       temp2 + ref_state->waistPos[1] << " " << 
		       m_zmptarget[2] + ref_state->waistPos[2] << " " << 
		       ref_state->waistPos[0] << " " << 
		       ref_state->waistPos[1] << " " << 
		       ref_state->waistPos[2]  
		       ,"DebugZMPFinale.dat");
	    }

	  ref_state->waistRpy[0] =0.0;
	  ref_state->waistRpy[1] =0.0;
	  //	  ref_state->waistRpy[2] =m_CurrentWaistState.theta*M_PI/180;
		
	  m_seq->setReferenceState(ref_state,dt);                              // send seq the next ref_state              
	}

      if (m_DebugMode>0)
	{
	  m_PGI->DebugControlLoop(m_CurrentStateFromPG,
				  m_CurrentVelocityFromPG,
				  m_count);
	}
	    
      m_count++;
    }	
  else 
    {
      ODEBUG4("But send nothing","DebugData.txt");
      m_ShouldBeRunning = false;
    }
}

bool WalkGenJRL::cleanup(robot_state *rs,motor_command *mc) 
{
  return true;
}


void WalkGenJRL::m_DumpZMPreel(istringstream &strm)
{
#if 1
  ofstream OutputFile;
  string aFileName;
  strm >> aFileName;
  OutputFile.open((char *)aFileName.c_str(),ofstream::out);
  if (OutputFile.is_open())
    {
      for(int i=0;i<m_ZMPIndexmax;i+=3)
	{
	  OutputFile << m_zmpreel[i] << " " <<
	    m_zmpreel[i+1] << " " <<
	    m_zmpreel[i+2] << endl;
	}
      OutputFile.close();
    }
#endif
}

void WalkGenJRL::m_ReadFileFromKineoWorks(istringstream &strm)
{

  if(m_PGI!=0)
    m_PGI->m_ReadFileFromKineoWorks(strm);
}

  
   
//// Implementation of CORBA Interface

void WalkGenJRL::setTargetPos(CORBA::Float x, CORBA::Float y, CORBA::Float z)
  throw(CORBA::SystemException)
{
  // TO DO
}

void WalkGenJRL::setTargetPosNoWait(CORBA::Float x, CORBA::Float y, CORBA::Float z)
  throw(CORBA::SystemException)
{
  // TO DO
}

void WalkGenJRL::setArc(CORBA::Float x, CORBA::Float y, CORBA::Float th)
  throw(CORBA::SystemException)
{
  // TO DO
}

void WalkGenJRL::setArcNoWait(CORBA::Float x, CORBA::Float y, CORBA::Float th)
  throw(CORBA::SystemException)
{
  // TO DO
}

void WalkGenJRL::setRfootPos(CORBA::Float x, CORBA::Float y, CORBA::Float th)
  throw(CORBA::SystemException)
{
  // TO DO
}

void WalkGenJRL::setRfootPosNoWait(CORBA::Float x, CORBA::Float y, CORBA::Float th)
  throw(CORBA::SystemException)
{
  // TO DO
}

void WalkGenJRL::setLfootPos(CORBA::Float x, CORBA::Float y, CORBA::Float th)
  throw(CORBA::SystemException)
{
  // TO DO
}

void WalkGenJRL::setLfootPosNoWait(CORBA::Float x, CORBA::Float y, CORBA::Float th)
  throw(CORBA::SystemException)
{
  // TO DO
}

void WalkGenJRL::stopWalking()
  throw(CORBA::SystemException)
{
  // TO DO
}

void WalkGenJRL::waitArrival()
  throw(CORBA::SystemException)
{
  // TO DO 
}

void WalkGenJRL::startStepping()
  throw(CORBA::SystemException)
{
  // TO DO
}

void WalkGenJRL::setWalkingVelocity(CORBA::Float dx,
					     CORBA::Float dy,
					     CORBA::Float dth)
  throw(CORBA::SystemException)
{
  // TO DO
}


void WalkGenJRL::getWaistVelocity(CORBA::Float_out dx,
				  CORBA::Float_out dy,
				  CORBA::Float_out omega)
  throw(CORBA::SystemException)
{
  double ldx=0.0,ldy=0.0,lomega=0.0;
  if (m_PGI!=0)
    {
      m_PGI->getWaistVelocity(ldx,ldy,lomega);
      dx = (float)ldx;
      dy = (float)ldy;
      omega = (float)lomega;
    }

}


#define max(a,b) (a>b?a:b)


void WalkGenJRL::stopStepping()
  throw(CORBA::SystemException)
{
  // TO DO
}

CORBA::Boolean WalkGenJRL::isWalking()
  throw(CORBA::SystemException)
{
  // TO DO 
  return true;
}


void WalkGenJRL::getWaistAcceleration(TransformQuaternion_out aTQ)
  throw(CORBA::SystemException)
{
  return;

}
void WalkGenJRL::getWaistPositionAndOrientation(TransformQuaternion_out aTQ,
							 CORBA::Float_out aOrientation)
  throw(CORBA::SystemException)
{
  

  double TQ[7]={0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0};
  double Orientation=0.0;

  if (m_PGI!=0)
    m_PGI->getWaistPositionAndOrientation(TQ,Orientation);

  // Carefull : Extremly specific to the pattern generator.
   // Position
  aTQ.px = TQ[0];
  aTQ.py = TQ[1];
  aTQ.pz = TQ[2];

  aTQ.qx = TQ[3];
  aTQ.qy = TQ[4];
  aTQ.qz = TQ[5];
  aTQ.qw = TQ[6];
  aOrientation = Orientation;
}

void WalkGenJRL::setWaistPositionAndOrientation(const TransformQuaternion & aTQ)
  throw(CORBA::SystemException)
{
  double TQ[7];

  TQ[0] = aTQ.px;
  TQ[1] = aTQ.py;
  TQ[2] = aTQ.pz;
  TQ[3] = aTQ.qx;
  TQ[4] = aTQ.qy;
  TQ[5] = aTQ.qz;
  TQ[6] = aTQ.qw;

  if (m_PGI!=0)
    m_PGI->setWaistPositionAndOrientation(TQ);
}

void WalkGenJRL::m_SendStackToControl(istringstream &strm)
{
  m_ShouldBeRunning=true;
}

void WalkGenJRL::m_FinishAndRealizeStepSequence(istringstream &strm)
{
  if (m_PGI!=0)
    m_PGI->m_FinishAndRealizeStepSequence(strm);

  m_count = 0;
  while(m_ShouldBeRunning)
    {
      usleep(10000);
    }

}


void WalkGenJRL::m_PartialStepSequence(istringstream &strm)
{
  if (m_PGI!=0)
    m_PGI->m_PartialStepSequence(strm);
}

CORBA::Long WalkGenJRL::getLegJointSpeed( dsequence_out dq)
    throw(CORBA::SystemException)
{
  // Joint velocity.
  MAL_VECTOR(dql,double);
  MAL_VECTOR(dqr,double);
  
  // Initialization of the  computed
  // leg's articular speed.
  MAL_VECTOR_RESIZE(dqr,6);
  MAL_VECTOR_RESIZE(dql,6);

  
  dsequence_var adq = new dsequence;
  adq->length(12);
  m_PGI->GetLegJointVelocity(dqr,dql);
  for(int i=0;i<6;i++)
    {
      adq[i] = dqr(i);
      adq[6+i] =dql(i);
    }
  dq = adq._retn();
  return 12;
}
