#!/usr/bin/env python3

import numpy as np
from pycrazyswarm.crazyswarm import Crazyswarm

if __name__ == "__main__":

    # Connect to Crazyflie objects
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # Define constant velociteis for examples
    vx = 0.0
    vy = 0.0
    vz = 50.0
    wx = 0.0
    wy = 50.0
    wz = 0.0

    rate = 25 # rate at which cmds are sent in Hz

    # send linear velocity command
    for i in range(0,100):
        for cf in allcfs.crazyflies:
            cf.cmdGtc(mode = 2, cmd1=vx, cmd2=vy, cmd3= vz)   
            timeHelper.sleepForRate(rate)   

    # send angular velocity command
    for i in range(0,100):
        for cf in allcfs.crazyflies:
            cf.cmdGtc(mode = 4,cmd1=wx, cmd2=wy, cmd3= wz)   
            timeHelper.sleepForRate(rate) 

    # send stop command
    for i in range(0, 100):
        for cf in allcfs.crazyflies:
            cf.cmdStop() 