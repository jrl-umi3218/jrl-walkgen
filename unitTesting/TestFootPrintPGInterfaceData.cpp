#include "TestFootPrintPGInterfaceData.h"

std::string ProfilesNames[19] = {
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
  "PROFIL_SIMU_ONLINE_WALKING",
  "PROFIL_HERDT",
  "PROFIL_HERDT_ONLINE"
};




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
