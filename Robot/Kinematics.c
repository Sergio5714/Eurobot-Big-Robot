#include "Kinematics.h"
//This document includes the forward and reverse kinematics of a robot with 4 omni wheels
//Full explanation is here: ( https://docs.google.com/document/d/1IShP9O5Ke3e50hDjm0TVveUJd4YA53RzAsRMRkQrHEM/edit )
//For the first picture in the solution part

// Note : matrix of forward kinematics already takes into account 2 cylindrical transmissions (wheels 2 and 4)

float MLineSpeed[4][3] = { 23.57022667,  -23.57022667,  0.0,  // forward linear matrix
                           23.57022667,   23.57022667,  0.0,
                           23.57022667,   23.57022667,  0.0,
                           23.57022667,  -23.57022667,  0.0};


float MRotSpeed[4][3] = { 0.0,  0.0,  -5.5485487,  // forward rotation matrix
                          0.0,  0.0,   5.5485487,
                          0.0,  0.0,  -5.5485487,
                          0.0,  0.0,   5.5485487};


//For the third picture in the solution part
float InverseKinematics[3][4] = { 0.0106066, -0.0106066,   0.0106066,  -0.0106066,  // inverse matrix
                                 -0.0106066, -0.0106066,   0.0106066,   0.0106066,     
                                 -0.04505683, -0.04505683,  -0.04505683,  -0.04505683 };



