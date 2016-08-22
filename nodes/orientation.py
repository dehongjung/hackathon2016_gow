import socket, traceback, string
from sys import stderr
from math import radians

def adjust_yaw(originalYaw):
    yaw = originalYaw
    if originalYaw < -118:
        yaw = -118
    elif originalYaw > 118:
        yaw = 118
    return yaw
    
def adjust_pitch(originalPitch):
    pitch = originalPitch
    if originalPitch < -37:
        pitch = -37
    elif originalPitch > 28:
        pitch = 28
    return pitch

host = ''
port = 5555
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
s.bind((host, port))

isFirstIteration = True
print 'Press <enter> to finish calibration.'
raw_input()

# Ignoring first elements in order to get the real orientation
i = 0
while i < 300:
    s.recvfrom(8192)
    i += 1
    
while 1:
        try:            
            message, address = s.recvfrom(8192)
            data = message.split(',')
            sensorID = int(data[1])
            if sensorID == 3 and len(data) == 17:
                pitch = float(data[16])
                yaw = -1.0 * float(data[15])
                        
                if isFirstIteration:
                    originYaw = yaw     # -180 to 180                
                    originPitch = pitch # -90 to 90
                    isFirstIteration = False
            
                pitch = pitch - originPitch
                yaw = yaw - originYaw
            
                pitch = adjust_pitch(pitch)
                yaw = adjust_yaw(yaw)
            
                stderr.write("\ryaw: %f, pitch: %f" % (radians(yaw), radians(pitch)))
                stderr.write("\ryaw: %f, pitch: %f" % (yaw, pitch))
                stderr.flush()
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            traceback.print_exc()
        