#include <HumanoidSpecificities.h>

#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "HumanoidSpecificities :" << x << endl
#define RESETDEBUG5(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG5(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "PGI: " << x << endl; DebugFile.close();}
#if 0
#define ODEBUG(x) cerr << "HumanoidsSpecificities :" <<  x << endl
#else
#define ODEBUG(x) 
#endif

#if 0
#define RESETDEBUG4(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG4(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "PGI: " << x << endl; DebugFile.close();}
#define _DEBUG_4_ACTIVATED_ 1 
#else
#define RESETDEBUG4(y) 
#define ODEBUG4(x,y)
#endif

#define ODEBUG6(x,y)

using namespace PatternGeneratorJRL;

HumanoidSpecificities::HumanoidSpecificities()
{

  m_FootHeight[0] = 0.137;
  m_FootHeight[1] = 0.137;
  m_FootWidth[0] = 0.24;
  m_FootWidth[1] = 0.24;
  m_UpperBodyJointNb = 0;
}


HumanoidSpecificities::~HumanoidSpecificities()
{
}

int HumanoidSpecificities::ReadXML(string &aFileName,
				    string &HumanoidName)
{
  FILE *fp;
  fp = fopen((char *)aFileName.c_str(),"r");
  
  if (fp==0)
    {
      cerr << "Unable to read " << aFileName << endl;
      return -1;
    }
 
  char Side[2][80] = {"Right","Left"};
  if (look_for(fp,"Humanoid"))
    {
      ODEBUG("Found Humanoid");
      if (look_for(fp,"Feet"))
	{
	  ODEBUG("Found Feet");
	  for(int i=0;i<2;i++)
	    {
	      if (look_for(fp,Side[i]))
		{
		  ODEBUG("Found Feet Side: " << Side[i]);
		  if (look_for(fp,"SizeX"))
		    {
		      fscanf(fp,"%lf",&m_FootWidth[i]);
		      ODEBUG("Found SizeX: " << m_FootWidth[i]);
		    }
		  
		  if (look_for(fp,"SizeY"))
		    {
		      fscanf(fp,"%lf",&m_FootHeight[i]);
		      ODEBUG("Found SizeY: " << m_FootHeight[i]);
		    }
		  
		  if (look_for(fp,"SizeZ"))
		    {
		      fscanf(fp,"%lf",&m_FootDepth[i]);
		      ODEBUG("Found SizeZ: " << m_FootDepth[i]);
		    }
		  
		  if (look_for(fp,"AnklePosition"))
		    {
		      fscanf(fp,"%lf", &m_AnklePosition[i][0]);
		      fscanf(fp,"%lf", &m_AnklePosition[i][1]);
		      fscanf(fp,"%lf", &m_AnklePosition[i][2]);
		      ODEBUG("AnklePos: " 
			      << m_AnklePosition[i][0] << " "
			      << m_AnklePosition[i][1] << " " 
			      << m_AnklePosition[i][2]);
		    }
		  if (look_for(fp,"JointNb"))
		    {
		      fscanf(fp,"%d", &m_FeetJointNb[i]);
		      ODEBUG("JointNb: " << m_FeetJointNb[i]);
		    }
		  if (look_for(fp,"JointsID"))
		    {
		      int aJoint;
		      for(int j=0;j<m_FeetJointNb[i];j++)
			{
			  fscanf(fp,"%d",&aJoint);
			  m_FeetJoints[i].insert( m_FeetJoints[i].end(),aJoint);
			}
		      ODEBUG("Joints :");
		      for(int j=0;j<m_FeetJoints[i].size();j++)
			{
			  ODEBUG(m_FeetJoints[i][j]);
			}
		      
		    }
		  
		}
	    }
	}

      if (look_for(fp,"Waist"))
	{
	  ODEBUG("Waist");
	  for(int i=0;i<2;i++)
	    {
	      if (look_for(fp,Side[i]))
		{
		  ODEBUG(Side[i]);
		  if (look_for(fp,"WaistToHip"))
		    {
		      fscanf(fp,"%lf",&m_WaistToHip[i][0]);
		      fscanf(fp,"%lf",&m_WaistToHip[i][1]);
		      fscanf(fp,"%lf",&m_WaistToHip[i][2]);
		    }
		}
	    }	  
	}
      if (look_for(fp,"Legs"))
	{
	  ODEBUG("Legs");
	  for(int i=0;i<2;i++)
	    {
	      if (look_for(fp,Side[i]))
		{
		  ODEBUG(Side[i]);

		  if (look_for(fp,"HipLength"))
		    {
		      fscanf(fp,"%lf",&m_HipLength[i][0]);
		      fscanf(fp,"%lf",&m_HipLength[i][1]);
		      fscanf(fp,"%lf",&m_HipLength[i][2]);
		      ODEBUG("Found FemurLength: " << m_HipLength[i]);
		    }

		  if (look_for(fp,"FemurLength"))
		    {
		      fscanf(fp,"%lf",&m_FemurLength[i]);
		      ODEBUG("Found FemurLength: " << m_FemurLength[i]);
		    }
		  
		  if (look_for(fp,"TibiaLength"))
		    {
		      fscanf(fp,"%lf",&m_TibiaLength[i]);
		      ODEBUG("Found TibiaLength: " << m_TibiaLength[i]);
		    }
		
		  if (look_for(fp,"JointNb"))
		    {
		      fscanf(fp,"%d", &m_LegsJointNb[i]);
		      ODEBUG("JointNb: " << m_LegsJointNb[i]);
		    }
		  if (look_for(fp,"JointsID"))
		    {
		      int aJoint;
		      for(int j=0;j<m_LegsJointNb[i];j++)
			{
			  fscanf(fp,"%d",&aJoint);
			  m_LegsJoints[i].insert( m_LegsJoints[i].end(),aJoint);
			}
		      ODEBUG("Joints :");
		      for(int j=0;j<m_LegsJoints[i].size();j++)
			{
			  ODEBUG(m_LegsJoints[i][j]);
			}
		      
		    }
		}

	    }
	}
      if (look_for(fp,"Arms"))
	{
	  ODEBUG("Arms");
	  for(int i=0;i<2;i++)
	    {
	      if (look_for(fp,Side[i]))
		{
		  if (look_for(fp,"UpperArmLength"))
		    {
		      fscanf(fp,"%lf",&m_UpperArmLength[i]);
		      ODEBUG("Found UpperArmLength: " << m_UpperArmLength[i]);
		    }
		  
		  if (look_for(fp,"ForeArmLength"))
		    {
		      fscanf(fp,"%lf",&m_ForeArmLength[i]);
		      ODEBUG("Found ForeArmLength: " << m_ForeArmLength[i]);
		    }
		
		  if (look_for(fp,"JointNb"))
		    {
		      fscanf(fp,"%d", &m_ArmsJointNb[i]);
		      ODEBUG("JointNb: " << m_ArmsJointNb[i]);
		    }
		  if (look_for(fp,"JointsID"))
		    {
		      int aJoint;
		      for(int j=0;j<m_ArmsJointNb[i];j++)
			{
			  fscanf(fp,"%d",&aJoint);
			  m_ArmsJoints[i].insert( m_ArmsJoints[i].end(),aJoint);
			}
		      ODEBUG("Joints :");
		      for(int j=0;j<m_ArmsJoints[i].size();j++)
			{
			  ODEBUG(m_ArmsJoints[i][j]);
			}
		      
		    }
		}
	    }
	}

      // Look for the head information.
      if (look_for(fp,"Head"))
	{
	  
	  if (look_for(fp,"JointNb"))
	    {
	      fscanf(fp,"%d", &m_HeadJointNb);
	      ODEBUG("JointNb: " << m_HeadJointNb);
	    }
	  if (look_for(fp,"JointsID"))
	    {
	      int aJoint;
	      for(int j=0;j<m_HeadJointNb;j++)
		{
		  fscanf(fp,"%d",&aJoint);
		  m_HeadJoints.insert( m_HeadJoints.end(),aJoint);
		}
	      ODEBUG("Joints :");
	      for(int j=0;j<m_HeadJoints.size();j++)
		{
		  ODEBUG(m_HeadJoints[j]);
		}
	      
	    }
	}

      // Look for the head information.
      if (look_for(fp,"Chest"))
	{
	  
	  if (look_for(fp,"JointNb"))
	    {
	      fscanf(fp,"%d", &m_ChestJointNb);
	      ODEBUG("JointNb: " << m_ChestJointNb);
	    }
	  if (look_for(fp,"JointsID"))
	    {
	      int aJoint;
	      for(int j=0;j<m_ChestJointNb;j++)
		{
		  fscanf(fp,"%d",&aJoint);
		  m_ChestJoints.insert( m_ChestJoints.end(),aJoint);
		}
	      ODEBUG("Joints :");
	      for(int j=0;j<m_ChestJoints.size();j++)
		{
		  ODEBUG(m_ChestJoints[j]);
		}
	      
	    }
	}

    }
  fclose(fp);
  return 0;
}

void HumanoidSpecificities::Display()
{

  string Side[2] = {"Right" , "Left"};

  for(int i=0;i<2;i++)
    {
      cout << "Size of the " << Side[i] << 
	" foot:" << endl;
      
      cout << "Width: " <<m_FootWidth[i] << " Height: " << m_FootHeight[i] << endl;

      cout << "Ankle position of foot " << Side[i]  << endl;
      
      for(int j=0;j<3;j++)
	cout << m_AnklePosition[i][j] << " ";
      cout << endl;

      cout << "Number of joints :" << m_FeetJointNb[i] << endl;
      for(int j=0;j<m_FeetJointNb[i];j++)
	cout << m_FeetJoints[i][j] << " ";
      cout << endl;


      cout << "Size of the " << Side[i]
	   << " Tibia: " << m_TibiaLength[i] << endl;
      cout << "Size of the " << Side[i]
	   << " Femur: " << m_FemurLength[i] << endl;

      cout << "Number of joints :" << m_LegsJointNb[i] << endl;
      for(int j=0;j<m_LegsJointNb[i];j++)
	cout << m_LegsJoints[i][j] << " ";
      cout << endl;

      cout << "Size of the " << Side[i]
	   << " UpperArm: " << m_UpperArmLength[i] << endl;
      cout << "Size of the " << Side[i]
	   << " ForeArm: " << m_ForeArmLength[i] << endl;

      cout << "Number of joints :" << m_ArmsJointNb[i] << endl;
      for(int j=0;j<m_ArmsJointNb[i];j++)
	cout << m_ArmsJoints[i][j] << " ";
      cout << endl;

      cout << "Size of the foot " << Side[i] 
	   << " Width: " << m_FootWidth[i] 
	   << " Height: " << m_FootHeight[i] << endl;

      cout << "Position of the hip according to the waist  " << Side[i] << " " ;
      for(int j=0;j<3;j++)
	cout << m_WaistToHip[i][j] << " ";
      cout << endl;

    }
      
  
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


double HumanoidSpecificities::GetTibiaLength(int WhichSide)
{
  if (WhichSide==-1)
    return m_TibiaLength[0];
  return m_TibiaLength[1];
}

double HumanoidSpecificities::GetFemurLength(int WhichSide)
{
  if (WhichSide==-1)
    return m_FemurLength[0];
  return m_FemurLength[1];
}

double HumanoidSpecificities::GetUpperArmLength(int WhichSide)
{
  if (WhichSide==-1)
    return m_UpperArmLength[0];
  return m_UpperArmLength[1];
}
  
double HumanoidSpecificities::GetForeArmLength(int WhichSide)
{
  if (WhichSide==-1)
    return m_ForeArmLength[0];
  return m_ForeArmLength[1];
}
  
void HumanoidSpecificities::GetAnklePosition(int WhichSide,double AnklePosition[3])
{
  int r=1;
  if (WhichSide==-1)
    r=0;
  for (int i=0;i<3;i++)
    AnklePosition[i] = m_AnklePosition[r][i];
}

void HumanoidSpecificities::GetWaistToHip(int WhichSide,double WaistToHip[3])
{
  int r=1;
  if (WhichSide==-1)
    r=0;

  for(int i=0;i<3;i++)
    WaistToHip[i] = m_WaistToHip[r][i];
}

void HumanoidSpecificities::GetHipLength(int WhichSide,double HipLength[3] )
{
  int r=1;
  if (WhichSide==-1)
    r=0;

  for(int i=0;i<3;i++)
    HipLength[i] = m_HipLength[r][i];

}

int HumanoidSpecificities::GetArmJointNb(int WhichSide)
{
  int r=1;
  if (WhichSide==-1)
    r=0;

  return m_ArmsJointNb[r];
}

const std::vector<int> & HumanoidSpecificities::GetArmJoints(int WhichSide)
{
  int r=1;
  if (WhichSide==-1)
    r=0;
  
  return m_ArmsJoints[r];
}



int HumanoidSpecificities::GetLegJointNb(int WhichSide)
{
  int r=1;
  if (WhichSide==-1)
    r=0;

  return m_LegsJointNb[r];
}

const std::vector<int> & HumanoidSpecificities::GetLegJoints(int WhichSide)
{
  int r=1;
  if (WhichSide==-1)
    r=0;
  
  return m_LegsJoints[r];
}

int HumanoidSpecificities::GetFootJointNb(int WhichSide)
{
  int r=1;
  if (WhichSide==-1)
    r=0;

  return m_FeetJointNb[r];
}

const std::vector<int> & HumanoidSpecificities::GetFootJoints(int WhichSide)
{
  int r=1;
  if (WhichSide==-1)
    r=0;
  
  return m_FeetJoints[r];
}

int HumanoidSpecificities::GetHeadJointNb()
{
  return m_HeadJointNb;
}

const std::vector<int> & HumanoidSpecificities::GetHeadJoints()
{
  return m_HeadJoints;
}

int HumanoidSpecificities::GetChestJointNb()
{
  return m_ChestJointNb;
}

const std::vector<int> & HumanoidSpecificities::GetChestJoints()
{
  return m_ChestJoints;
}

int HumanoidSpecificities::InitUpperBodyJoints()
{
  m_UpperBodyJointNb = m_HeadJointNb + m_ChestJointNb + 
    m_ArmsJointNb[0] + m_ArmsJointNb[1];
  int lindex =0;

  m_UpperBodyJoints.resize(m_UpperBodyJointNb);

  for(int i=0;i<m_HeadJointNb;i++)
    m_UpperBodyJoints[lindex++] = m_HeadJoints[i];
  
  for(int i=0;i<m_ChestJointNb;i++)
    m_UpperBodyJoints[lindex++] = m_ChestJoints[i];
  
  for(int i=0;i<m_ArmsJointNb[0];i++)
    m_UpperBodyJoints[lindex++] = m_ArmsJoints[0][i];
  
  for(int i=0;i<m_ArmsJointNb[1];i++)
    m_UpperBodyJoints[lindex++] = m_ArmsJoints[1][i];

}

int HumanoidSpecificities::GetUpperBodyJointNb()
{
  if (m_UpperBodyJointNb==0)
    {
      InitUpperBodyJoints();
    }
  return m_UpperBodyJointNb;
}


const std::vector<int> & HumanoidSpecificities::GetUpperBodyJoints()
{
  if (m_UpperBodyJointNb==0)
    {
      InitUpperBodyJoints();
    }
  return m_UpperBodyJoints;
}

