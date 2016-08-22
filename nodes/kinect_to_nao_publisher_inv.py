#!/usr/bin/env python

# 

# Standard imports
import roslib
roslib.load_manifest('tg_nao_teleoperation')
import rospy
import numpy as np

# Related third part imports
import tf
import math
from tf.transformations import euler_from_quaternion
import time

# Personal imports
import utils.kinect_utils as k_utils
import utils.kalman as kalman_filter

# ROS messages
import geometry_msgs.msg
from geometry_msgs.msg import Point, Vector3
from tg_nao_teleoperation.msg import *

PI = 3.14159265359
HALFPI = 1.57079632679

class KinectToNaoPublisher(object):

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
        

    def apply_kalman_filter(self, kalman_object, nao_joint, chain_name):
        """
        """
        kalman_object.state_callback()
        
        joint_measurement = np.array([
                                [nao_joint.x],
                                [nao_joint.y],
                                [nao_joint.z]
                                ])
        kalman_object.measurement_callback(joint_measurement)
        
#        print nao_joint.x, nao_joint.y, nao_joint.z
#        print kalman_object.x
        
        corrected_nao_joint = Vector3()
        corrected_nao_joint.x = kalman_object.x[0]
        corrected_nao_joint.y = kalman_object.x[1]
        corrected_nao_joint.z = kalman_object.x[2]
        
        # Measure time elapsed
        elapsed_time = self.get_elapsed_time()
        
        return corrected_nao_joint

    def fill_message(self, joint, index, effector):
        """ """
        nao_chain_msg = NaoChain()
        
        # Append effector name
        nao_chain_msg.effector = effector
        
        # Append points
        nao_chain_msg.x = joint.x
        nao_chain_msg.y = joint.y
        nao_chain_msg.z = joint.z

        # Append angles
        nao_chain_msg.roll  = self.kinect_frame_RPY.euler_angle[index].roll
        nao_chain_msg.pitch = self.kinect_frame_RPY.euler_angle[index].pitch           
        nao_chain_msg.yaw   = self.kinect_frame_RPY.euler_angle[index].yaw
        
        return nao_chain_msg       

    def get_nao_coordinates(self, joint1_name, joint2_name, nao_joint_offset):
        
        # Get joints indexes
        joint1_index = self.kinect_skeleton_frames.index(joint1_name)
        joint2_index = self.kinect_skeleton_frames.index(joint2_name)
        
        # Now, get their coordinates relativelly to openni_depth_frame
        joint1 = Vector3()
        joint2 = Vector3()
        
        joint1.x = self.kinect_frame_RPY.position[joint1_index].x
        joint2.x = self.kinect_frame_RPY.position[joint2_index].x
        
        joint1.y = self.kinect_frame_RPY.position[joint1_index].y
        joint2.y = self.kinect_frame_RPY.position[joint2_index].y
        
        joint1.z = self.kinect_frame_RPY.position[joint1_index].z
        joint2.z = self.kinect_frame_RPY.position[joint2_index].z
        
        # Calculate the distance between the 2 points
        kin_module_length = k_utils.get_distance_between_points(joint1, joint2)
        
        # Then, we can find the respective point of the second joint using the first
        # joint as the origin point
        joint_target = Vector3()
        
        joint_target.x = (joint2.x - joint1.x)*(nao_joint_offset/kin_module_length)
        joint_target.y = (joint2.y - joint1.y)*(nao_joint_offset/kin_module_length)
        joint_target.z = (joint2.z - joint1.z)*(nao_joint_offset/kin_module_length)        
        
        return joint_target
        
    def manipulate_and_fill_msg_with_frame_information(self):
         
        if "left_hand" in self.kinect_frame_RPY.frame_name:
            index_left_hand = self.kinect_skeleton_frames.index("left_hand")
            # in metters
            nao_upper_arm_length = .105
            
            nao_elbow = Vector3()
            nao_elbow = self.get_nao_coordinates("left_shoulder", "left_elbow", nao_upper_arm_length)
            
            nao_hand_offset = .1117 #LowerArmLength + HandOffsetX
            
            nao_hand = Vector3()
            nao_hand = self.get_nao_coordinates("left_elbow", "left_hand", nao_hand_offset)

            # Translate positions
            nao_joint = Vector3()
            nao_joint.x = -(nao_hand.z + nao_elbow.z)
            nao_joint.y = -(nao_hand.x + nao_elbow.x) + .098
            nao_joint.z = nao_hand.y + nao_elbow.y + .1
            
            # Apply Kalman Filter
            nao_joint = self.apply_kalman_filter(self.kalman_larm, nao_joint, "LArm")
                
            self.nao_larm = self.fill_message(nao_joint, index_left_hand, "LArm")
            
        if "right_hand" in self.kinect_frame_RPY.frame_name:
            index_right_hand = self.kinect_skeleton_frames.index("right_hand")

            # in metters
            nao_upper_arm_length = .105
            
            nao_elbow = Vector3()
            nao_elbow = self.get_nao_coordinates("right_shoulder", "right_elbow", nao_upper_arm_length)
            
            nao_hand_offset = .1117
            
            nao_hand = Vector3()
            nao_hand = self.get_nao_coordinates("right_elbow", "right_hand", nao_hand_offset)

            nao_joint = Vector3()
            nao_joint.x = -(nao_hand.z + nao_elbow.z)
            nao_joint.y = -(nao_hand.x + nao_elbow.x) - .098 +.010
            nao_joint.z = nao_hand.y + nao_elbow.y + 0.1
            
            # Apply Kalman Filter
            nao_joint = self.apply_kalman_filter(self.kalman_rarm, nao_joint, "RArm")
            
            self.nao_rarm = self.fill_message(nao_joint, index_right_hand, "RArm")
        
    def publish_nao_frames(self):
        """
        Publishes frames
        """

        self.left_arm_publisher.publish(self.nao_larm)

        self.right_arm_publisher.publish(self.nao_rarm)
        
        self.pub_rate.sleep()

            
    def listen_to_kinect(self):
        """
        Returns a list of frames
        """
    
        # Initialize tf listener
        self.listener = tf.TransformListener()
        
        # Set time to get tracking information
        self.now = rospy.Time(0)
        
        # 
        first_iteration = True
            
        while not rospy.is_shutdown():
        
            user_detected = False
            
            try:
                # Clear the frames lists to store new positions and orientations
                self.kinect_frame_RPY.frame_name  = list()
                self.kinect_frame_RPY.position    = list()
                self.kinect_frame_RPY.euler_angle = list()
                
                # Access all frames in kinect_skeleton_frames to get data
                for frame in self.kinect_skeleton_frames:
                    # The program i use now publishes tf without the user id. But when I use openni I have to append this.
                    # Only track the first user
                    #user_frame = frame + "_1" 
                    euler_pose = Eulers()
                    pose = Point()
                    frame_name = frame
                    
                    # Get translation and rotation for each frame
                    (trans, rot) = self.listener.lookupTransform(self.base_frame, frame_name, self.now)
                    
                    user_detected = True
                    
                    pose.x = trans[0]
                    pose.y = trans[1]
                    pose.z = trans[2]
                    
                    # Convert quaternion to euler angles
                    euler_pose.roll, euler_pose.pitch, euler_pose.yaw  = k_utils.convert_quaternion_to_euler(rot)
                    
                    # Append frame information to list
                    self.kinect_frame_RPY.frame_name.append(frame_name)
                    self.kinect_frame_RPY.position.append(pose)
                    self.kinect_frame_RPY.euler_angle.append(euler_pose)
                    
                if user_detected == True:
                    # Won't set new references for the pitch
                    first_iteration = False
                    
                    # Scale and translate data to correspond to NAO coordinate system and
                    # set message to send to the robot
                    self.manipulate_and_fill_msg_with_frame_information()
                    
                    self.publish_nao_frames()
                    
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue     
                
    def initialize_publishers(self):
        """
        """
        # Define a publisher for the left arm chain
        self.left_arm_publisher = rospy.Publisher('nao_left_arm', NaoChain)
        
        # Define a publisher for the right arm chain
        self.right_arm_publisher = rospy.Publisher('nao_right_arm', NaoChain)
                
    def initialize_messages(self):
        """
        """
        # Initialize the nao message (left arm)
        self.nao_larm = NaoChain()

        # Initialize the nao message (left arm)
        self.nao_rarm = NaoChain()
        
    def initialize_kalman_filter(self):
        """
        """
        # Initialize an object to use kalman filter for the left arm
        self.kalman_larm = kalman_filter.KalmanFilter()    
        
        # Initialize an object to use kalman filter for the right arm
        self.kalman_rarm = kalman_filter.KalmanFilter()

    def __init__(self):
    
        # ROS inicialization
        rospy.init_node('kinect_to_nao_publisher')
        
        rospy.loginfo("kinect joint tracker is running...")
        
        self.first_time = True
        
        # Rate to publish
        self.pub_rate = rospy.Rate(100)
        
        # Get base frame
        self.base_frame = rospy.get_param('~fixed_frame', 'openni_depth_frame')
        
        # Get list of skeleton frames
        self.kinect_skeleton_frames = rospy.get_param('~kinect_skeleton_frames', '')
        
        # Intialize data smt to store information
        self.kinect_frame_RPY = KinectFrame()
        
        # Initialize messages
        self.initialize_messages()
        
        # Initialize publishers
        self.initialize_publishers() 
        
        # Initialize filters
        self.initialize_kalman_filter()

        # Get informations from /tf topic
        self.listen_to_kinect()            
                                    
if __name__ == '__main__':

    publisher = KinectToNaoPublisher()

    rospy.loginfo("kinect joint tracker stopped running.")
    
    exit(0)

