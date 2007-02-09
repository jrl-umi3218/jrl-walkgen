#ifndef __PG_PATTERN_GENERATOR_TYPES_H_
#define __PG_PATTERN_GENERATOR_TYPES_H_

#include <MatrixAbstractLayer.h>

namespace PatternGeneratorJRL
{
  /**
     \brief : Structure to store the COM position computed by the preview control. 
     \param x : x position, x velocity, x accelaration
     \param y : x position, x velocity, x accelaration
     \param x : x position, x velocity, x accelaration
     \param theta : orientation of the waist (euler Angle)
     \param omega : orientation of the waist (euler Angle)
     \param phi : orientation of the waist (euler Angle)
  */
  struct COMPosition_s
  {
    double x[3],y[3]; 
    double z[3];    	
    double theta;
    double omega;
    double phi;
     	
  };
  typedef struct COMPosition_s COMPosition;

  /** Structure to store each foot position when the user is specifying 
      a sequence of relative positions. */
  struct RelativeFootPosition_s
  { 
    double sx,sy,theta;
    float SStime;
    float DStime;
    int stepType;     //1:normal walking 2:one step before opbstacle
                      //3:first leg over obstacle 4:second leg over obstacle 5:one step after obstacle
    double DeviationHipHeight;      
   };
  typedef struct RelativeFootPosition_s RelativeFootPosition;

  /** Structure to store each of the ZMP value, with a given
      direction at a certain time. */
  struct ZMPPosition_s
  { 
    double px,py;
    double theta;//For COM
    double time;
    int stepType;     //1:normal walking 2:one step before opbstacle
                      //3:first leg over obstacle 4:second leg over obstacle 5:one step after obstacle
			//+10 if duoble support phase
			//*(-1) if right foot stance else left foot stance 
  };
  typedef struct ZMPPosition_s ZMPPosition;

  /// Structure to store the absolute foot position.
  struct FootAbsolutePosition_t
  { 
    double x,y,z, theta, omega; // px, py in meters, theta in DEGREES.
    double time;
    int stepType;     //1:normal walking 2:one step before opbstacle
                      //3:first leg over obstacle 4:second leg over obstacle 5:one step after obstacle
		      //+10 if double support phase
			//*(-1) if stance foot else swing foot or double support 
  };
  typedef struct FootAbsolutePosition_t FootAbsolutePosition;


  /// Linear constraint.
  struct LinearConstraintInequality_s
  {
    MAL_MATRIX(A,double);
    MAL_MATRIX(B,double);
    double StartingTime, EndingTime;
  };
  typedef struct LinearConstraintInequality_s 
    LinearConstraintInequality_t;

};
#endif
