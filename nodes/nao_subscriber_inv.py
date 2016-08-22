#!/usr/bin/env python


# Standard imports
from naoqi import ALProxy
import roslib
roslib.load_manifest('tg_nao_teleoperation')
import rospy

# Related third part imports
import math
from naoqi import motion
import time

# # Personal imports
import utils.nao_utils as n_utils

# ROS services
from tg_nao_teleoperation.srv import *

#ROS messages
from tg_nao_teleoperation.msg import NaoFrame_inv, NaoChain, NaoSupPol

class ControlNAO(object):
        
    def StiffnessOn(self):
        # We use the "Body" name to signify the collection of all joints
        pNames = "Body"
        pStiffnessLists = 1.0
        pTimeLists = 1.0
        self.motionProxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

    def get_elapsed_time(self):
        """
        """
        if self.first_time == True:
            self.start_time = time.time()
            self.first_time = False
            return 0.0
        else:
            elapsed_time = time.time() - self.start_time   
            return elapsed_time
                      
    def move_chain(self, data, axis_mask):
        """ """
        chain_name = data.effector
        # Get position
        x, y, z = data.x, data.y, data.z
        roll, pitch, yaw = 0, 0, 0
        #roll, pitch, yaw = data.roll, data.pitch, data.yaw
        if self.change_axis_mask == True:
            axis_mask = 7

        space = motion.SPACE_TORSO

        # Set target position
        targetPos  = [x, y, z, roll, pitch, yaw]
        targetTime = 0.002 #works slowly, but ok with .2

        path  = [ targetPos ]
        times = [ targetTime ]

        eff_pose_current = self.motionProxy.getPosition(chain_name, space, True)
        initial_thetas = self.motionProxy.getAngles(chain_name, space, use_sensors = True)
        eff_pose_reference = targetPos
        go_balance = 0
        balance_rotation = [0]
        
        # First test: non-blocking call set Position
        fraction_max_speed = 1.0     
#        self.motionProxy.setPosition(chain_name, space, targetPos, fraction_max_speed, axis_mask)
        
        rospy.wait_for_service("find_angles_using_dq")
        try:
            get_angles_dq = rospy.ServiceProxy('find_angles_using_dq', DQservice)
            data_dq = get_angles_dq(chain_name, initial_thetas[0:5], eff_pose_reference, 200.0, 2.8)
            thetas = list(data_dq.thetas)
            thetas.append(initial_thetas[5])

            self.motionProxy.setAngles(chain_name, thetas, 1.0)
            
            actual_position = self.motionProxy.getPosition(chain_name, motion.SPACE_TORSO, True)
            # Measure time elapsed
            elapsed_time = self.get_elapsed_time()
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e               

    def move_leg(self, data, axis_mask):
        """ """
        chain_name = data.effector
        # Get position
        x, y, z = data.x, data.y, data.z
        roll, pitch, yaw = 0, 0, 0
        #roll, pitch, yaw = data.roll, data.pitch, data.yaw
        if self.change_axis_mask == True:
            axis_mask = 7

        space = motion.SPACE_TORSO

        # Set target position
        targetPos  = [x, y, z, roll, pitch, yaw]

        eff_pose_current = self.motionProxy.getPosition(chain_name, space, True)
        initial_thetas = self.motionProxy.getAngles(chain_name, space, use_sensors = True)
        eff_pose_reference = targetPos
        fraction_max_speed = 1.0     
        self.motionProxy.setPosition(chain_name, space, targetPos, fraction_max_speed, axis_mask)
#       
        actual_position = self.motionProxy.getPosition(chain_name, motion.SPACE_TORSO, True)
        
        # Measure time elapsed
        elapsed_time = self.get_elapsed_time()


    def left_arm_callback(self, data):
        axis_mask = 7 # just control position
        self.move_chain(data, axis_mask)
                    
    def right_arm_callback(self, data):
        axis_mask = 7
        self.move_chain(data, axis_mask)

    def head_callback(self, data):

        # Movimentacao das juntas
        JointNames = ["HeadPitch", "HeadYaw"]
        JointAngles = [data.pitch, data.yaw]
        Time = 0.4
        print "Pitch: ", data.pitch
        print "Yaw: ", data.yaw

        self.motionProxy.setAngles(JointNames, JointAngles, Time)

    def lateral_move_callcabk(self, data):
        leftArmEnable  = False
        rightArmEnable  = False
        self.motionProxy.setWalkArmsEnabled(leftArmEnable, rightArmEnable)

        x  = 0.0
        y  = 0.7
        theta  = 0.0
        frequency  = 0.8
        self.motionProxy.setWalkTargetVelocity(x, y, theta, frequency)

    def listen_to_nao_topics(self):
        """
        """

        print 'NAOOOOOOOOOOOOOOOOOOOOOOOOO'
        # Subscribe to left arm
        rospy.Subscriber("nao_left_arm", NaoChain, self.left_arm_callback, queue_size=1)
        
        # Subscribe to right arm
        rospy.Subscriber("nao_right_arm", NaoChain, self.right_arm_callback, queue_size=1)

        # Subscribe to Head
        rospy.Subscriber("nao_head", NaoChain, self.head_callback, queue_size=1)
        

    def send_to_initial_pose(self, desired_position):
        """
        """
        # Set NAO in Stiffness On
        self.StiffnessOn()

        # Send NAO to Pose Init
        # self.postureProxy.goToPosture(desired_position, 0.5)
        
        time.sleep(1.0)
        
        # self.initial_larm_position = self.motionProxy.getPosition("LArm", motion.SPACE_TORSO, True)
        
    def create_proxies(self):
        # Connect to NAOqi
        try:
            self.motionProxy = ALProxy("ALMotion", self.ip, self.port)
        except RuntimeError, e:
            rospy.logerr("Could not create ALMotion ALProxy: %s", e)
            exit(1)
        
        # Connect to module ALRobotPosture
        try:
            self.postureProxy = ALProxy("ALRobotPosture", self.ip, self.port)
        except Exception, e:
            print "Could not create proxy to ALRobotPosture"
            print "Error was: ", e
            
        # Connect to ALMemory
        try:
            self.memoryProxy = ALProxy("ALMemory", self.ip, self.port)
        except Exception, e:
            print "Could not create proxy to ALMemory"
            print "Error was: ", e
            
    def charge_parameters(self):
        # Change values in launch file, these are the default values
        # For naoqi_ip, we have the options: home, lpci, lara
        self.ip = '169.254.212.204' # rospy.get_param('~naoqi_ip_lara', '127.0.0.1')
        self.port = 9559 #int(rospy.get_param('~nao_port', '9559'))
        
        # Check if parameters are ok
        rospy.loginfo('Parameter %s has value %s',
                       rospy.resolve_name('naoqi_ip'),
                       self.ip)

    def __init__(self):
    
        # ROS inicialization
        rospy.init_node('nao_controller', anonymous=True)
        
        rospy.loginfo("control nao arms is running...")
        
        self.wait_change_support = False
        self.change_axis_mask = False
        self.first_time = True
        
        # Get NAO robot parameters
        self.charge_parameters()
                
        # Connect to NAO modules
        self.create_proxies()

        # Start NAO in a specific position
        self.send_to_initial_pose("StandInit")
        self.postureProxy.goToPosture("StandInit", 1.0)

        # Subscribes to a topic to receive NAO angles
        self.listen_to_nao_topics() #achei esse nome ruim... =p

if __name__ == '__main__':
    control_nao = ControlNAO()
    rospy.spin()
    
    rospy.loginfo("control nao arms stopped running.")

    exit(0)  
