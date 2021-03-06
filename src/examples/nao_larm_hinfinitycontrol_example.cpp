/**
Trying to implement NAO Control Examples Using HInfinityRobustController.h
Based on Schunk Control Examples Using HInfinityRobustController.h

Start to implement with one arm only

\author 
\since 05/2014

*/

#include "../DQ.h"
#include "../DQ_kinematics.h"
#include "../controllers/HInfinityRobustController.h" //Has HInfinityRobustController(...,...)
#include "../robot_dh/NAO.h" //Has NaoKinematics()

#include <cmath> //For fabs and M_PI_2

using namespace Eigen;
using namespace DQ_robotics;

float* DQ_invKin(   char* space, 
                    Matrix<double,6,1> eff_pose_current_nao, 
                    Matrix<double,5,1> initial_thethas_nao, 
                    Matrix<double,6,1> eff_pose_reference_nao )
{
    const double pi2 = M_PI_2;

    //Gain Matrix
	  Matrix<double,8,1> B;
    B << 1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0;

    //Initial Joint Values
    //Matrix<double,5,1> initial_thetas;
    //initial_thetas << 0.1,0.1,0.1,0.1,0.1;
    Matrix<double,5,1> thetas = initial_thetas;

 	  //Robot DH
	  DQ_kinematics nao_larm = NaoKinematics();

    //End Effector Pose eff_pose_reference
    //Matrix<double,5,1> reference_thetas;
    //reference_thetas << 0.2,0.2,0.2,0.2,0.2;
    DQ eff_pose_reference( nao_larm.fkm( reference_thetas ) );

    // verify if eff_pose_reference ~ eff_pose_reference_nao

    //HInfinity Controller
    double alpha = 2.0;

    //Gamma finder
    double initial_gamma  =  0.01;
    double gamma_step     =  0.001;
    double final_gamma    =  3.00;
    int    maximum_steps  =  10;
    double current_gamma  =  initial_gamma;
    bool   finished       =  false;


    //Control Loop Variables
    DQ eff_pose_current(0);
    DQ eff_pose_difference(20);
    double control_threshold = 1.e-10;
    int control_step_count   = 0;

    while( current_gamma < final_gamma && not finished )
    {
      HInfinityRobustController controller = HInfinityRobustController(nao_larm, B, current_gamma, alpha);

      //Control Loop Variables
      eff_pose_current    = DQ(0);
      eff_pose_difference = DQ(20);
      control_step_count  = 0;
      thetas = initial_thetas;

      finished = true;
      //Control Loop
      while(eff_pose_difference.vec8().norm() > control_threshold)
      {   
          //One controller step
          thetas = controller.getNewJointPositions(eff_pose_reference,thetas);

          //End of control check
          eff_pose_current = nao_larm.fkm(thetas);
          eff_pose_difference = (eff_pose_current - eff_pose_reference);
          //std::cout << std::endl << (eff_pose_difference.vec8()).transpose().norm() << std::endl;

          //Count Steps
          control_step_count++;
          if(control_step_count > maximum_steps)
          {
            std::cout << std::endl << "Gamma " << current_gamma << " not fit." << std::endl;
            break;
          }
      }      
      if(control_step_count > maximum_steps)
        finished = false;

      current_gamma += gamma_step;
    }
    std::cout << std::endl << "Gamma " << current_gamma << " selected." << std::endl; 


    std::cout << std::endl <<"Control Loop Ended In " << control_step_count << " Steps" << std::endl;
    std::cout << std::endl <<"End Effector Pose Reference" << std::endl << eff_pose_reference << std::endl;
    std::cout << std::endl <<"End Effector Final Pose" << std::endl << eff_pose_current << std::endl;
    std::cout << std::endl <<"End Effector Final Pose Difference" << std::endl << eff_pose_difference << std::endl;
    std::cout << std::endl <<"Final Thetas" << std::endl << thetas << std::endl;

    return 0;
}


