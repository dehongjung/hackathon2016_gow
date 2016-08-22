/**
Trying to implement NAO Control Examples Using HInfinityRobustController.h
Based on Schunk Control Examples Using HInfinityRobustController.h

Start to implement with one arm only

\author 
\since 05/2014

*/
// Transform into a ROS node
#include "ros/ros.h"
#include "tg_nao_teleoperation/DQservice.h" //The server file

#include "../DQ.h"
#include "../DQ_kinematics.h"
#include "../controllers/TranslationOnlyPoseController.h"
#include "../robot_dh/NAO.h" //Has NaoKinematics()

#include <cmath> //For fabs and M_PI_2


using namespace Eigen;
using namespace DQ_robotics;
using namespace std;

// This function returns all the thetas to move the left nao arm
bool DQ_invKin(tg_nao_teleoperation::DQservice::Request  &req,
               tg_nao_teleoperation::DQservice::Response &res )
{
    ros::NodeHandle node_handle;
    
    // Data from NAO
    string chain = req.chain;
    Matrix<double,6,1> initial_thetas_nao(req.initial_thetas.data());
    Matrix<double,6,1> eff_pose_reference_nao(req.eff_pose_reference.data());
    
    double t_gain = req.gain; //translation gain
    
    double t_damp = req.damp;
    
    const double pi2 = M_PI_2;

//    std::cout << std::endl <<"End Effector Pose Reference" << std::endl << eff_pose_reference_nao << std::endl;


    //Initial Joint Values
    Matrix<double,6,1> thetas;
    thetas << initial_thetas_nao;
    
    //Gain Matrix
	Matrix<double,4,4> kp = Matrix<double,4,4>::Zero(4,4);
	Matrix<double,4,1> kp_diagonal(4,1);
    kp_diagonal << t_gain,t_gain,t_gain,t_gain;
    kp.diagonal() = kp_diagonal;

    // Nao uso
	Matrix<double,4,4> kr = Matrix<double,4,4>::Zero(4,4);
	Matrix<double,4,1> kr_diagonal(4,1);
	kr_diagonal << 0.2,0.2,0.2,0.2;
	kr.diagonal() = kr_diagonal;    

    //Robot DH
    
    DQ_kinematics nao_chain = NaoKinematics_Leg();
    
    if (chain == "LLeg")
        {
        eff_pose_reference_nao[0] = eff_pose_reference_nao[0];
        eff_pose_reference_nao[1] = eff_pose_reference_nao[1] - 0.050;
        eff_pose_reference_nao[2] = eff_pose_reference_nao[2] + 0.085 + 0.04519;         
        }
    else if (chain == "RLeg")
        {
        eff_pose_reference_nao[0] = eff_pose_reference_nao[0];
        eff_pose_reference_nao[1] = eff_pose_reference_nao[1] + 0.050;
        eff_pose_reference_nao[2] = eff_pose_reference_nao[2] + 0.085 + 0.04519;
        }
        
    // Taking into account translation only
    DQ eff_pose_reference(1,0,0,0,0,eff_pose_reference_nao[0]/2,eff_pose_reference_nao[1]/2,eff_pose_reference_nao[2]/2);

    TranslationOnlyPoseController pf_controller(nao_chain, kp, kr, t_damp, 0.1);

    //Control Loop Variables
    DQ eff_pose_current(0);
	DQ eff_pose_older = nao_chain.fkm(thetas);
    DQ eff_pose_difference(0); 

	DQ eff_translation_difference(20); //Big initial values
	DQ eff_rotation_difference(20);    //so they don't break the loop.

    double translation_threshold = 0.001;

    //Control Loop
	//The control loop end criterion is changed in this example. This criterion is used 
	//to search for local convergence. Both sensibilities can be changed using the given thresholds.
	eff_pose_current = nao_chain.fkm(thetas);
	eff_translation_difference = vec4( eff_pose_current.translation() - eff_pose_reference.translation());
//	std::cout << std::endl <<"Error" << std::endl << eff_translation_difference.vec4().norm() << std::endl;
    if( (eff_translation_difference.vec4().norm() > translation_threshold) )
    {   

        //One controller step
        thetas = pf_controller.getNewJointPositions(eff_pose_reference, thetas);

        //End of control check
        eff_pose_current = nao_chain.fkm(thetas);

		// Get translation differenceS
		eff_translation_difference = vec4(eff_pose_current.translation() - eff_pose_reference.translation());
    }
    
//    std::cout << std::endl << "oi" << std::endl;
    
    double aux[] = {thetas(0, 0), thetas(1, 0), thetas(2, 0), thetas(3, 0), thetas(4, 0), thetas(5, 0)};
    res.thetas.assign(aux, aux+6);
    
    
    Vector4d end_eff_position = vec4(eff_pose_current.translation());
    
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nao_dual_quaternions_legs");
    ros::NodeHandle n;
    ROS_INFO("nao dual quaternions legs is running...");
    
    ros::ServiceServer service = n.advertiseService("find_angles_using_dq_legs", DQ_invKin);
    ROS_INFO("Ready to calculate new angles for NAO");
    ros::spin();
    
    return 0;
}


