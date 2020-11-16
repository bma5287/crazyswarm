#!/usr/bin/env python3

import numpy as np
import rospy
from crazyflie_driver.msg import State

def state_callback(data):
    state = np.array(data.state)
    print(state)

if __name__ == "__main__":
    rospy.init_node('crazyflie_node', anonymous=True)
    rospy.Subscriber("/cf1/state", State, state_callback)
    rospy.spin() 