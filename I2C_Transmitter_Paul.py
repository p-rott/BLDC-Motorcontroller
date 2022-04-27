#!/usr/bin/env python
# -*- coding: utf-8 -*-
import smbus
import sys
import rospy
from std_msgs.msg import Float32
from bad_i2c.msg import steering_data
from threading import Thread

global txSpeed, txSteering
txSpeed = 0                         # speed value send to motorcontroller
txSteering = 0                      # steering value send to motorcontroller
i2cSlaveAddress = 0x08              # esp-32 motorcontroller i2c address
i2cMasterbus = smbus.SMBus(1)       # selects I2C bus nr.1 on Raspberry (older RPis might require bus 0)

maxSteeringAngle = 255              
minSteeringAngle = -255
maxSpeed = 255
minSpeed = -255


def transmit(speed, steering):    
    try: 
        # check given values
        if(speed > maxSpeed): speed = maxSpeed
        elif(speed < minSpeed): speed = minSpeed
        if (steering > maxSteeringAngle): steering = maxSteeringAngle
        elif(steering < minSteeringAngle): steering = minSteeringAngle

        # normalize values 
        speed = 128 + (speed / 2)
        #speed = int(speed/255.0 * 46)
        #steering = int(steering +255)
        rospy.logout("calculated speed: " + str(speed))

        message = [speed, steering]
        #i2cMasterbus.write_byte(i2cSlaveAddress, message)
        i2cMasterbus.write_i2c_block_data(i2cSlaveAddress, 0, message)
        rospy.logout("I2C_Handler: transmitted: " + str(message))
        sys.exit(1)

    except Exception as e: 
        rospy.logerr("I2C_Transmitter: cought exception while transmitting " + str(e))
        sys.exit(1)

## Node Class
#
#  initialises the ROS Node 'T_XBox'
class CT_Node(object):  
    ## contructor
    #
    #  initiates ROS node and sets loop rate
    def __init__(self):
        rospy.init_node('I2C_Transmitter', anonymous=False)
        self.loop_rate = rospy.Rate(100)  
        rospy.logout("I2C_Transmitter: Node started")
        rospy.Subscriber('ap_steeringData', steering_data, callback_steeringData)
        rospy.spin()
    
def callback_steeringData(data):
    txSteering = data.steering
    txSpeed = data.speed
    #rospy.logout("I2C_Transmitter: transmitting data - speed: " + str(txSpeed) + " steering: " + str(txSteering))
    transmit(txSpeed, txSteering)

def main():
    try:
        i2cNode = CT_Node()
    except KeyboardInterrupt:
        print("I2C_Transmitter: Stopped by user")
        sys.exit(0)
    except Exception as e:
        print("I2C_Transmitter: Cought Exception in main - " + str(e))
        sys.exit(0)
if __name__ == '__main__':
    main()