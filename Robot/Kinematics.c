//This document includes the forward and reverse kinematics of a robot with 4 omni wheels
//Full explanation is here: ( https://docs.google.com/document/d/1IShP9O5Ke3e50hDjm0TVveUJd4YA53RzAsRMRkQrHEM/edit )
//For the first picture in the solution part

// Note : matrix of forward kinematics already takes into account 2 cylindrical transmissions (wheels 2 and 4)

float MLineSpeed[4][3] = { 24.04868127,   27.07752606,  0.0,  // forward linear matrix
                           30.82350247,  -16.18349293,  0.0,
                           30.82350247,   16.18349293,  0.0,
                           24.04868127,  -27.07752606,  0.0};


float MRotSpeed[4][3] = { 0.0,  0.0, -5.85666637,  // forward rotation matrix
                          0.0,  0.0,  4.53981119,
                          0.0,  0.0,  4.53981119,
                          0.0,  0.0, -5.85666637};


//For the third picture in the solution part
float InverseKinematics[3][4] = { 0.00783538, -0.01010818, -0.01010818,  0.00783538,  // inverse matrix
                                  0.01360546,  0.00813161, -0.00813161, -0.01360546,     
                                 -0.05319911, -0.04150626, -0.04150626, -0.05319911 };



