#!/usr/bin/env python3

import numpy as np
from pycrazyswarm.crazyswarm import Crazyswarm
import rospy
from crazyflie_driver.msg import State
from threading import Thread

class Cfexample:
    def __init__(self):
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.allcfs = self.swarm.allcfs
        self.statethread = Thread(target=self.state_sub) # thread for collecting state
        self.statethread.daemon = True
        self.state = 0

        self.run()

    def state_callback(self,data):
        self.state = np.array(data) 
        #print(self.state)

        print("(x,y,z) = ({0:.3f},{1:.3f},{2:.3f}) m \n".format(self.state[0],self.state[1],self.state[2]))
        print("(qw,qx,qy,qz) = ({0:.4f},{1:.4f},{2:.4f},{3:.4f}) m \n".format(self.state[3],self.state[4],self.state[5],self.state[6]))
        print("(vx,vy,vz) = ({0:.3f},{1:.3f},{2:.3f}) m \n".format(self.state[7],self.state[8],self.state[9]))
        print("(wx,wy,wz) = ({0:.3f},{1:.3f},{2:.3f}) m \n \n".format(self.state[10],self.state[11],self.state[12]))

    def state_sub(self):
        #rospy.init_node('crazyflie_node',anonymous=True)
        rospy.Subscriber('/cf1/state',State,self.state_callback)
        rospy.spin()

    def run(self):
        self.statethread.start()

    def numcf(self):
        return len(self.allcfs.crazyflies)

    def isConnected(self):
        return self.numcf() > 0

if __name__ == "__main__":
    cfe = Cfexample()
    while not cfe.isConnected():
        print("No Crazyflie Connected")
    print("Found Crazyflie!")    
    cf = cfe.allcfs.crazyflies[0]
    rate = 25

    vx = 0.0
    vy = 0.0
    vz = 50.0
    wx = 0.0
    wy = 50.0
    wz = 0.0

    RREV = (1.5 - cfe.state[2])/cfe.state[9]

    # send linear velocity command for 4 seconds
    for i in range(0,100):
        cf.cmdGtc(mode = 2, cmd1=vx, cmd2=vy, cmd3= vz)   
        cfe.timeHelper.sleepForRate(rate)   

    # send angular velocity command for 4 seconds
    for i in range(0,100):
        cf.cmdGtc(mode = 4,cmd1=wx, cmd2=wy, cmd3= wz)   
        cfe.timeHelper.sleepForRate(rate) 

    # send stop command
    for i in range(0, 100):
        cf.cmdStop() 