/**
Trying to implement NAO Control Examples Using HInfinityRobustController.h
Based on Schunk Control Examples Using HInfinityRobustController.h

Start to implement with one arm only

\author 
\started the 2014/05/28

*/
#ifndef NAO_LARM_INV_KIN_H
#define NAO_LARM_INV_KIN_H

#include "../DQ.h"
#include "../DQ_kinematics.h"
#include "../controllers/HInfinityRobustController.h" //Has HInfinityRobustController(...,...)
#include "../robot_dh/NAO.h" //Has NaoKinematics()

#include <cmath> //For fabs and M_PI_2


#ifdef __cplusplus
extern "C" 
{
#endif

using namespace Eigen;
using namespace DQ_robotics;

Matrix<double,5,1> DQ_invKin(   char* space, 
                    Matrix<double,6,1> eff_pose_current_nao, 
                    Matrix<double,5,1> initial_thethas_nao, 
                    Matrix<double,6,1> eff_pose_reference_nao );


#ifdef __cplusplus
}
#endif


#endif
