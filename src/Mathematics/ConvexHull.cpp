#include <Mathematics/ConvexHull.h>
#include <set>
#include <math.h>

#define ODEBUG2(x)
#define ODEBUG3(x) cerr << "ConvexHull :" << x << endl
#define RESETDEBUG5(y) { ofstream DebugFile; DebugFile.open(y,ofstream::out); DebugFile.close();}
#define ODEBUG5(x,y) { ofstream DebugFile; DebugFile.open(y,ofstream::app); DebugFile << "PGI: " << x << endl; DebugFile.close();}
#if 0
#define ODEBUG(x) cerr << "ConvexHull :" <<  x << endl
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

using namespace std;

namespace PatternGeneratorJRL 
{
  CH_Point HRP2CIO_GlobalP0;
  struct ltCH_Point
  {
    bool operator() (const CH_Point & s1, const CH_Point &s2)
    {
      float x1,x2,y1,y2;
      x1 = s1.col - HRP2CIO_GlobalP0.col;
      x2 = s2.col - HRP2CIO_GlobalP0.col;
      y1 = s1.row - HRP2CIO_GlobalP0.row;
      y2 = s2.row - HRP2CIO_GlobalP0.row;

      return (x1*y2 - x2*y1)>0.0;
    }
  };

  void DistanceCHRep(CH_Point & s1,CH_Point & s2,
		     float & distance1, float &distance2)
  {
    float x1,x2,y1,y2;
    x1 = s1.col - HRP2CIO_GlobalP0.col;
    x2 = s2.col - HRP2CIO_GlobalP0.col;
    y1 = s1.row - HRP2CIO_GlobalP0.row;
    y2 = s2.row - HRP2CIO_GlobalP0.row;
    
    distance1 = sqrt(x1*x1 + y1*y1);
    distance2 = sqrt(x2*x2 + y2*y2);
    
  }

  float CompareCBRep(CH_Point & s1,CH_Point &s2)
  {
    float x1,x2,y1,y2;
    x1 = s1.col - HRP2CIO_GlobalP0.col;
    x2 = s2.col - HRP2CIO_GlobalP0.col;
    y1 = s1.row - HRP2CIO_GlobalP0.row;
    y2 = s2.row - HRP2CIO_GlobalP0.row;
    
    return (x1*y2 - x2*y1);
  }

  ComputeConvexHull::ComputeConvexHull()
  {
    
  }
  
  ComputeConvexHull::~ComputeConvexHull()
  {
    
  }
  
  void ComputeConvexHull::DoComputeConvexHull(vector <CH_Point> aVecOfPoints,
				     vector <CH_Point> & TheConvexHull)
  {
  
    if (aVecOfPoints.size()==0)
      return;

    CH_Point p0 = aVecOfPoints[0];

    // Detects the point with the smallest value for y.
    for(unsigned int i=0;i<aVecOfPoints.size();i++)
      if (aVecOfPoints[i].row < p0.row)
	p0 = aVecOfPoints[i];
  
  
    HRP2CIO_GlobalP0 = p0;
    // Create the list of pixels sortes according to their
    // polar coordinates regarding p0.

    ODEBUG2(" VecOfPoints : " << aVecOfPoints.size());
    set<CH_Point, ltCH_Point> ListOfPointinPolarCoord;
    for(unsigned int i=0;i<aVecOfPoints.size();i++)
      { 
	unsigned char bInsert=1;

	// Check if the current point already exist with the same angle.
	// Cannot use find because of some pb with gcc version 2.95.
	set<CH_Point, ltCH_Point>::iterator it_PtinPolarCoord, toDestroy;
      
	it_PtinPolarCoord = ListOfPointinPolarCoord.begin();
	while (it_PtinPolarCoord!=ListOfPointinPolarCoord.end())
	  {
	    CH_Point Current = *it_PtinPolarCoord;
	    if (CompareCBRep(Current,aVecOfPoints[i])==0.0)
	      {
		float distance1, distance2;
		DistanceCHRep(Current,aVecOfPoints[i],
			      distance1, distance2);
		if (distance1<=distance2)
		  {
		    toDestroy = it_PtinPolarCoord;
		    it_PtinPolarCoord++;
		    ListOfPointinPolarCoord.erase(toDestroy);
		  }
		else 
		  {
		    it_PtinPolarCoord++;
		    bInsert=0;
		  }
	      }
	    else 
	      it_PtinPolarCoord++;

	  }

	if (bInsert)
	  {
	    ListOfPointinPolarCoord.insert(aVecOfPoints[i]);
	  }
      
      }

    // Apply the Graham's scan in O(n log n)

    // Creates the stack.

    set<CH_Point, ltCH_Point>::iterator it_LPPC;
    ODEBUG2( "LPPC: " <<ListOfPointinPolarCoord.size());


    it_LPPC = ListOfPointinPolarCoord.begin();
    TheConvexHull.insert(TheConvexHull.end(),p0);
    TheConvexHull.insert(TheConvexHull.end(),(*it_LPPC));
    it_LPPC++;
    TheConvexHull.insert(TheConvexHull.end(),(*it_LPPC));
    it_LPPC++;


    // Create the convex hull.
    while(it_LPPC!=ListOfPointinPolarCoord.end())
      {
	CH_Point pi = *it_LPPC;
	unsigned char ok=0;
	float x1,x2,y1,y2;
	CH_Point s1;
	CH_Point s2;

	do
	  {
	    if (TheConvexHull.size()>=2)
	      {
		s1 = TheConvexHull[TheConvexHull.size()-1];
		s2 = TheConvexHull[TheConvexHull.size()-2];
	      
		x1 = s1.col - s2.col;
		x2 = pi.col - s2.col;
		y1 = s1.row - s2.row;
		y2 = pi.row - s2.row;
	  
		if ((x1*y2 - x2*y1)>0.0)
		  ok = 1;
		else 
		  ok = 0;
		
	      }
	    else 
	      {
		ok=1;
	      }

	    if (!ok)
	      TheConvexHull.pop_back();
	  }
	while(!ok);
      
      
	TheConvexHull.push_back(*it_LPPC);
	it_LPPC++;
      }
  
  }
}
