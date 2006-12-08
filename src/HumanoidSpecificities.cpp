#include <HumanoidSpecificities.h>

#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "HumanoidSpecificities :" << x << endl
#define RESETDEBUG5(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG5(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "PGI: " << x << endl; DebugFile.close();}
#if 1
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
  m_DMB = 0;

  m_FootHeight[0] = 0.137;
  m_FootHeight[1] = 0.137;
  m_FootWidth[0] = 0.24;
  m_FootWidth[1] = 0.24;
    
}

HumanoidSpecificities::HumanoidSpecificities(DynamicMultiBody *aDMB)
{

  HumanoidSpecificities();
  m_DMB = aDMB;    
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

      cout << "Size of the " << Side[i]
	   << " Tibia: " << m_TibiaLength[i] << endl;
      cout << "Size of the " << Side[i]
	   << " Femur: " << m_FemurLength[i] << endl;
      cout << "Size of the " << Side[i]
	   << " UpperArm: " << m_UpperArmLength[i] << endl;
      cout << "Size of the " << Side[i]
	   << " ForeArm: " << m_ForeArmLength[i] << endl;

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
