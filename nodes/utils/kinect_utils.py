#!/usr/bin/env python

# Year: 2014 
# Author: Marcela P. Carvalho, email: marcela_pcarvalho@hotmail.com

# Standard imports
import rospy

# Related third part imports
import tf
import math

# ROS services

def get_distance_between_points(joint1, joint2):
    """
    Get distance between two points in 3D space
    """
    dist = math.sqrt((joint2.x - joint1.x)**2 + 
                (joint2.y - joint1.y)**2 + 
                (joint2.z - joint1.z)**2)

    return dist
    
def verify_foot_pitch(frame, index, reference):
    """
    
    """
    # Considerando que a indicacao eh de 0 a 180 e 0 a -PI
    threshold_pitch = 0.1745 # 10 degrees
    actual_pitch = frame.euler_angle[index].pitch
    
    if (actual_pitch < reference + threshold_pitch
            and actual_pitch > reference - threshold_pitch):
        return 1
    else:
        return 0
        
def check_support_polygon(frame, index_left, index_right,
                        pitch_reference_left_foot, pitch_reference_right_foot,
                        threshold_y):
    """
    Returns the polygon support based on the difference of height between
    user's feet.
    """
    
    left_y  = (-1)*frame.position[index_left].y
    right_y = (-1)*frame.position[index_right].y
    
    # The y has the openni_depth_frame as reference. That's why whe don't
    # call it height
    # left_foot raised
    if left_y < right_y - threshold_y:
    
        # Now verify angle of the anckle joint of the other foot
        no_changes_in_pitch = verify_foot_pitch(
                                    frame,
                                    index_right,
                                    pitch_reference_right_foot
                                    )
        if no_changes_in_pitch:
            return "right_support"
    else:
        if right_y < left_y - threshold_y:
        
            # Now verify angle of the anckle joint of the other foot
            no_changes_in_pitch = verify_foot_pitch(
                                        frame, 
                                        index_left,
                                        pitch_reference_left_foot
                                        )
            if no_changes_in_pitch:
                return "left_support"
    
    return "double_support"     
    
def convert_quaternion_to_euler(quaternion):
    """
    """
    #from tf.transformations import euler_from_quaternion
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]
    
    (r, p, y) = tf.transformations.euler_from_quaternion([x, y, z, w])
    
    return (r, p, y)    
