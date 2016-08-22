#!/usr/bin/env python

import roslib
roslib.load_manifest('tg_nao_teleoperation')
import rospy
import tf
from tf.transformations import euler_from_quaternion
import math

def convert_quaternion_to_euler(self, quaternion):
    #from tf.transformations import euler_from_quaternion
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]
    
    (r, p, y) = tf.transformations.euler_from_quaternion([x, y, z, w])
    
    return (r, p, y)
    
def get_distance(self, joint1, joint2):
    """Get distance between two points in space"""
    
    dist = math.sqrt((joint2.x - joint1.x)**2 + (joint2.y - joint1.y)**2 + (joint2.z - joint1.z)**2)
    
    return dist   
        
def get_nao_coordinates(self, skeleton_frames, actual_frame_RPY, joint1_name, joint2_name, nao_joint_offset):
    
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
    kin_module_length = self.get_distance(joint1, joint2)
    
    # Then, we can find the respective point of the second joint using the first
    # joint as the origin point
    joint_target = Vector3()
    
    joint_target.x = (joint2.x - joint1.x)*(nao_joint_offset/kin_module_length)
    joint_target.y = (joint2.y - joint1.y)*(nao_joint_offset/kin_module_length)
    joint_target.z = (joint2.z - joint1.z)*(nao_joint_offset/kin_module_length)        
    
    return joint_target         
