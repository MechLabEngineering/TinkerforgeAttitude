#!/usr/bin/env python
# -*- coding: utf-8 -*-  

HOST = "localhost"
PORT = 4223
UID = "6DdNWN" # Change to your UID

from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_imu import IMU
import numpy as np
from numpy.linalg import inv
import time

def rad2deg(x):
    return x*180.0/np.pi


def calcattitude(ax, ay, az):
    ax = ax*9.806/1000.0
    ay = ay*9.806/1000.0
    az = az*9.806/1000.0


    # Calculate Roll and Pitch from Acceleration
    # is only valid in quasistatic situations
    rollacc = rad2deg(np.arctan2(-ay, -az))
    pitchacc= rad2deg(-np.arctan2(-ax, np.sqrt(ay**2+az**2)))

    print("ax:\t%+6.0f\tay:\t%+6.0f\taz:\t%+6.0f\troll:\t%+6.0f\tpitch:\t%+6.0f" % (ax, ay, az, rollacc, pitchacc))

def calcattitude2(x, y, z, w):

    # Calculate Attitude
    # Buchholz, J. J. (2013). Vorlesungsmanuskript Regelungstechnik und Flugregler.
    # GRIN Verlag. Retrieved from http://www.grin.com/de/e-book/82818/regelungstechnik-und-flugregler
    yaw   =  np.arctan2(2.0*(x*y + w*z), w**2+x**2-y**2-z**2)
    pitch = -np.arcsin(2.0*(w*y - x*z))
    roll  = -np.arctan2(2.0*(y*z + w*x), -(w**2-x**2-y**2+z**2))

    print("\t\t\t\tyaw:\t%+6.0f\troll:\t%+6.0f\tpitch:\t%+6.0f" % (rad2deg(yaw), rad2deg(roll), rad2deg(pitch)))
    print(80*'=')


if __name__ == "__main__":
    ipcon = IPConnection() # Create IP connection
    imu = IMU(UID, ipcon) # Create device object

    ipcon.connect(HOST, PORT) # Connect to brickd
    # Don't use device before ipcon is connected

    # Set period for quaternion callback
    imu.set_quaternion_period(100)
    imu.set_acceleration_period(100)

    # Register quaternion callback
    imu.register_callback(imu.CALLBACK_QUATERNION, calcattitude2)
    imu.register_callback(imu.CALLBACK_ACCELERATION, calcattitude)

    raw_input('Press key to exit\n') # Use input() in Python 3
    
    ipcon.disconnect()