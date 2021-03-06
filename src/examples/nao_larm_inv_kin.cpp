/**
Trying to implement NAO Control Examples Using HInfinityRobustController.h
Based on Schunk Control Examples Using HInfinityRobustController.h

Start to implement with one arm only

\author 
\since 05/2014

*/
// Transform into a ROS node
#include "ros/ros.h"
#include "tg_nao_teleoperation/getAnglesDQ.h"

#include "../DQ.h"
#include "../DQ_kinematics.h"
#include "../controllers/HInfinityRobustController.h" //Has HInfinityRobustController(...,...)
#include "../robot_dh/NAO.h" //Has NaoKinematics()

#include <cmath> //For fabs and M_PI_2


//#include "../include/nao_larm_inv_kin.h"

using namespace Eigen;
using namespace DQ_robotics;
using namespace std;


// This function returns all the thetas to move the left nao arm
bool DQ_invKin(tg_nao_teleoperation::getAnglesDQ::Request  &req,
               tg_nao_teleoperation::getAnglesDQ::Response &res )
{

    string space = req.space;
    
    Matrix<double,6,1> eff_pose_current_nao(req.eff_pose_current.data());
    
    Matrix<double,5,1> initial_thetas_nao(req.initial_thetas.data());
    
    Matrix<double,6,1> eff_pose_reference_nao(req.eff_pose_reference.data());
    
    const double pi2 = M_PI_2;

    std::cout << std::endl <<"End Effector Pose Reference" << std::endl << eff_pose_reference_nao << std::endl;

    //Gain Matrix
	  Matrix<double,8,1> B;
    B << 1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0;

    //Initial Joint Values
    Matrix<double,5,1> thetas;
    thetas << initial_thetas_nao;

 	  //Robot DH
	  DQ_kinematics nao_larm = NaoKinematics();

    //eff_pose_current_nao[1] = eff_pose_current_nao[1] - 0.098;
    //eff_pose_current_nao[2] = eff_pose_current_nao[2] - 0.1;
    DQ eff_pose_currentX(1,0,0,0,0,eff_pose_current_nao[0]/2,eff_pose_current_nao[1]/2,eff_pose_current_nao[2]/2);
    std::cout << std::endl <<"End Effector Pose Current from nao" << std::endl << eff_pose_currentX << std::endl;
    
    eff_pose_reference_nao[0] = eff_pose_reference_nao[1] - 0.05775;
    eff_pose_reference_nao[1] = eff_pose_reference_nao[1] - 0.098;
    eff_pose_reference_nao[2] = eff_pose_reference_nao[2] - 0.1;

    // Taking into account translation only
    DQ eff_pose_reference(1,0,0,0,0,eff_pose_reference_nao[0]/2,eff_pose_reference_nao[1]/2,eff_pose_reference_nao[2]/2);
    std::cout << std::endl <<"End Effector Pose Reference" << std::endl << eff_pose_reference << std::endl;

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

      //Control Loop Variables
      eff_pose_current    = DQ(0);
      eff_pose_difference = DQ(20);
      control_step_count  = 0;
      thetas = initial_thetas_nao;

      finished = true;
      
      double epsilon = 0.001;
      double gain = 0.1;
      
      MatrixXd jacob = MatrixXd(8,5) ;
      double xm;
      Matrix<double,8,8> C8_ = C8();
      if (eff_pose_difference.vec8().norm() > epsilon)
      {

        jacob = nao_larm.jacobian(thetas);

        eff_pose_current = nao_larm.fkm(thetas); //o que esta acontecendo aqui?
        
        std::cout << std::endl <<"End Effector Pose Current from Theta" << std::endl << eff_pose_current << std::endl;
        eff_pose_difference = ( eff_pose_reference - eff_pose_current );//(eff_pose_current - eff_pose_reference);

        thetas = thetas + pseudoInverse(jacob)*gain* eff_pose_difference.vec8();
        
        double aux[] = {thetas(0, 0), thetas(1, 0), thetas(2, 0), thetas(3, 0), thetas(4, 0)};
        res.thetas.assign(aux, aux+5);
      }
        
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nao_dual_quaternions");
    ros::NodeHandle n;
    ROS_INFO("nao dual quaternions is running...");
    
    ros::ServiceServer service = n.advertiseService("find_angles_using_dq", DQ_invKin);
    ROS_INFO("Ready to calculate new angles for NAO");
    ros::spin();
    
    return 0;
}


