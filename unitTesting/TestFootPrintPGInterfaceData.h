#ifndef _TEST_FOOT_PRINT_PG_INTERFACE_H_
#define _TEST_FOOT_PRINT_PG_INTERFACE_H_

#include <string>

enum Profiles_t  {
  PROFIL_PB_FLORENT,                                 //  0
  PROFIL_STEPPING_OVER,                              //  1 
  PROFIL_SHORT_STRAIGHT_WALKING,                     //  2
  PROFIL_SHORT_STRAIGHT_WALKING_ONE_STAGE,           //  3
  PROFIL_CURVED_WALKING_PBW2,                        //  4
  PROFIL_KINEOWORKS,                                 //  5
  PROFIL_STRAIGHT_WALKING,                           //  6
  PROFIL_ANALYTICAL_SHORT_STRAIGHT_WALKING,          //  7
  PROFIL_CURVED_WALKING_PBW,                         //  8
  PROFIL_STRAIGHT_WALKING_DIMITROV,                  //  9
  PROFIL_CURVED_WALKING_DIMITROV,                    // 10
  PROFIL_TURN_90D,                                   // 11
  PROFIL_TURNING_ON_THE_CIRCLE,                      // 12
  PROFIL_TURNING_ON_THE_CIRCLE_TOWARDS_THE_CENTER,   // 13
  PROFIL_ANALYTICAL_ONLINE_WALKING,                  // 14
  PROFIL_ONLINE_WALKING,                             // 15
  PROFIL_SIMU_ONLINE_WALKING                         // 16
};

std::string ProfilesNames[17] = {
  "PROFIL_PB_FLORENT",                                 
  "PROFIL_STEPPING_OVER",                              
  "PROFIL_SHORT_STRAIGHT_WALKING",                     
  "PROFIL_SHORT_STRAIGHT_WALKING_ONE_STAGE",           
  "PROFIL_CURVED_WALKING_PBW2",                        
  "PROFIL_KINEOWORKS",                                 
  "PROFIL_STRAIGHT_WALKING",                           
  "PROFIL_ANALYTICAL_SHORT_STRAIGHT_WALKING",          
  "PROFIL_CURVED_WALKING_PBW",                         
  "PROFIL_STRAIGHT_WALKING_DIMITROV",                  
  "PROFIL_CURVED_WALKING_DIMITROV",                    
  "PROFIL_TURN_90D",                                   
  "PROFIL_TURNING_ON_THE_CIRCLE",                      
  "PROFIL_TURNING_ON_THE_CIRCLE_TOWARDS_THE_CENTER",   
  "PROFIL_ANALYTICAL_ONLINE_WALKING",                  
  "PROFIL_ONLINE_WALKING",                             
  "PROFIL_SIMU_ONLINE_WALKING"
};

enum InitialPoses_t {
  HALF_SITTING_2003,
  TELEOPERATION_2008,
  HWPG_v1,
  HALF_SITTING_2008,
  INTERACTION_2008,
  MODEL_BUILDING_1,
  MODEL_BUILDING_2
};

double InitialPoses[7][40] = {

  // 1- With previous half-sitting value
  { 
    0.0, 0.0, -20.0, 40.0, -20.0, 0.0, 0.0, 0.0, -20.0, 40.0, -20.0, 0.0,  // legs

    0.0, 0.0, -23.7, 6.6, // chest and head

    27.0, -5.0, -4.0, -87.0, -4.0, -16.0, 20.0, // right arm
    15.0,  10.0, 0.0, -20.0, 0.0, 0.0, 10.0, // left arm 

    -20.0, 20.0, -20.0, 20.0, -20.0, // right hand
    -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
  },
  // 2- Nicolas position + New half sitting for the legs
  { 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 0.0, 0.0, -26.0, 50.0, -24.0, 0.0,  // legs

    0.0, 0.0, -23.7, 6.6, // chest and head

    27.0, -5.0, -4.0, -87.0, -4.0, -16.0, 20.0, // right arm
    15.0,  10.0, 0.0, -20.0, 0.0, 0.0, 10.0, // left arm 

    -20.0, 20.0, -20.0, 20.0, -20.0, // right hand
    -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
  },
  // 3- Test for comparison with PG v1.x
  { 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 0.0, 0.0, -26.0, 50.0, -24.0, 0.0,  // legs

    0.0, 0.0, 0.0, 0.0, // chest and head

    0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // right arm
    0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // left arm 

    0.0, 0.0, 0.0, 0.0, 0.0, // right hand
    0.0, 0.0, 0.0, 0.0, 0.0  // left hand
  },
  // 4- New Half sitting
  { 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0, 
    0.0, 0.0, -26.0, 50.0, -24.0, 0.0,  // legs

    0.0, 0.0, 0.0, 0.0, // chest and head

    15.0, -10.0, 0.0, -30.0, 0.0, 0.0, 10.0, // right arm
    15.0,  10.0, 0.0, -30.0, 0.0, 0.0, 10.0, // left arm 

    -10.0, 10.0, -10.0, 10.0, -10.0  // right hand
    -10.0, 10.0, -10.0, 10.0, -10.0  // left hand
  },
  // 5- Position for interaction
  { 

    -8.41612e-05, -0.000321953, -0.436642, 0.872997, -0.434218, 0.00189464,
    -8.47913e-05, -2.10591e-05, -0.437037, 0.873512, -0.433726, 3.73681e-05, //legs
    0.000169362, 2.98267e-18,  // chest
    -1.34701e-10, 5.62876e-10, // head 

    0.427868, -0.321049, -0.0696696, -1.90803, -0.323627, -0.0845895, 0.199999, // right arm
    0.458665,  0.322058,  0.0772106, -1.93087,  0.324485, -0.0914427, 0.199999, // left arm
    
    -0.174533, 0.174533, -0.174533, 0.174533, -0.174533, // right hand
    -0.174533, 0.174533, -0.174533, 0.174533, -0.174533  // left hand
  },
  // 6- Initial position for model building 1,
  {
    14.323945,  -6.0363396,  -13.459409,    44.02602,  -30.566611,    6.0363396,
    0.0000001,   7.4859801,  -27.663319,    44.65489,  -16.991579,   -7.4859801,
    0.,    0.,    0.,    0.,    
    12.397718,  -10.000004,    0.,  -29.618538,    0.,    0.,    10.0,
    16.536364,   10.000004,    0.,  -29.828011,    0.,    0.,    10.0,
    
    -10.0,  10.0, -10.0,  10,   -10.0, 
    -10.0,  10.0, -10.0,  10.0, -10.0 
  },
  // 7- Initial position for model buiding 2
  {
    -7.16197, -7.69299, -16.1787, 44.5201, -28.3415,  7.69299, 
    7.16197,   5.74946, -31.3668, 44.1057, -12.7389, -5.74946,
    
    0., 0., 0., 0., 
    
    12.622 , -10, 0, -29.678 , 0, 0, 10, 
    16.7091,  10, 0, -29.7841, 0, 0, 10, 
    
    -10.0,  10.0, -10.0,  10,   -10.0, 
    -10.0,  10.0, -10.0,  10.0, -10.0 
  }

};




#define NBOFPREDEFONLINEFOOTSTEPS 11
#if 1
double OnLineFootSteps[NBOFPREDEFONLINEFOOTSTEPS][3]={
  { -0.005221439, -0.00123569, -2.23518e-182},
  { -0.00699088, -0.00170217, -4.21367e-182},
  { -0.00208854, 0.00162538, -6.78877e-183},
  { -0.0058683, -0.0020374, -1.2328e-182},
  { -0.004536, 0.00119127, -1.59333e-183},
  { -0.00696306, -0.00252192, -2.88263e-183},
  { -0.00278527, 0.000492459, -4.2968e-184},
  { -0.00536233, -0.0021008, -7.82941e-184},
  { -0.00191246, 0.00125745, -1.16966e-184},
  { -0.0053683, -0.00232864, -2.08273e-184},
  { -0.00168054, 0.00108031, -3.26998e-185}
};
#else
double OnLineFootSteps[NBOFPREDEFONLINEFOOTSTEPS][3]={
  { 0.05, 0.0, 0.0},
  { 0.05, 0.0, 0.0},
  { 0.05, 0.0, 0.0},
  { 0.05, 0.0, 0.0},
  { 0.05, 0.0, 0.0},
  { 0.05, 0.0, 0.0},
  { 0.05, 0.0, 0.0},
  { 0.05, 0.0, 0.0},
  { 0.05, 0.0, 0.0},
  { 0.05, 0.0, 0.0},
  { 0.05, 0.0, 0.0}
};
#endif

#endif /* _TEST_FOOT_PRINT_PG_INTERFACE_H_ */
