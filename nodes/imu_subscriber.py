'''
    Hackathon Globo 2016
    Author: De Hong Jung
            Filipe Caixeta
'''

import roslib
roslib.load_manifest('tg_nao_teleoperation')
import rospy
import sys
import numpy as np 
import socket, traceback
from math import radians, degrees

#ROS messages
from tg_nao_teleoperation.msg import NaoChain

class IMU:

  def __init__(self):
    # Publisher
    self.pub = rospy.Publisher("nao_head", NaoChain)

    # Subscriber
    self.sub = rospy.Subscriber("nao_left_arm", NaoChain, self.callback, queue_size=1)

    # Comunicacao
    host = ''
    port = 5555
    self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    self.s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    self.s.bind((host, port))

    self.aux = 1

    self.angX = 0.0
    self.angY = 0.0
    self.angZ = 0.0

    self.angles = NaoChain()

    self.t = 0.0

    i = 0
    while i < 300:
      self.s.recvfrom(8192)
      i += 1


  # ================= LOOP ===================

  def callback(self, data):
    # try:       
    #     message, address = self.s.recvfrom(8192)
    #     data = message.split(',')
    #     sensorID = int(data[1])
    #     if sensorID == 3 and len(data) == 17:
    #         pitch = float(data[16])
    #         yaw = -1.0 * float(data[15])
                    
    #         if self.isFirstIteration:
    #           self.originYaw = yaw     # -180 to 180                
    #           self.originPitch = pitch # -90 to 90
    #           self.isFirstIteration = False
        
    #         pitch = pitch - self.originPitch
    #         yaw = yaw - self.originYaw
        
    #         pitch = self.adjust_pitch(pitch)
    #         yaw = self.adjust_yaw(yaw)
        
    #         self.angles.yaw = radians(yaw)
    #         self.angles.pitch = radians(pitch)

    #         print "Pitch: ", self.angles.pitch
    #         print "Yaw: ", self.angles.yaw

    #         self.pub.publish(self.angles)
    # except (KeyboardInterrupt, SystemExit):
    #     raise
    # except:
    #     traceback.print_exc()

    try:            
      message, address = self.s.recvfrom(8192)

      # Split records using comma as delimiter (data are streamed in CSV format)
      data = message.split( "," )

      if (self.aux):
          self.t = float(data[0])
          self.aux = 0

      # Convert to float for plotting purposes
      

      sensorID = int(data[1])
      if sensorID == 3 and len(data) > 8:     # sensor ID for the accelerometer
          dt = float(data[0]) - self.t
          self.t = float(data[0])

          ax, ay, az = data[2], data[3], data[4]
          wx, wy, wz = float(data[6]), float(data[7]), float(data[8])
          
          self.angX = self.angX + wx*dt
          self.angY = self.angY + wy*dt

          self.angles.pitch = self.adjust_pitch(self.angY)
          self.angles.yaw = self.adjust_yaw(self.angX)

          self.angles.yaw = self.angX
          self.angles.pitch = self.angY

          print "Pitch: ", self.angles.pitch
          print "Yaw: ", self.angles.yaw
          print "dt: ", dt

          self.pub.publish(self.angles)

    except (KeyboardInterrupt, SystemExit):
      raise
    except:
      traceback.print_exc()


  def adjust_yaw(self, originalYaw):
    yaw = originalYaw
    if originalYaw < -radians(118.0):
        yaw = -radians(118.0)
    elif originalYaw > radians(118.0):
        yaw = radians(118.0)
    return yaw
    
  def adjust_pitch(self, originalPitch):
    pitch = originalPitch
    if originalPitch < radians(-37.0):
        pitch = radians(-37.0)
    elif originalPitch > radians(28.0):
        pitch = radians(28.0)
    return pitch


# ============== MAIN ================

def main(args):
  ic = IMU()
  rospy.init_node('IMU', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)