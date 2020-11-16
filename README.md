# crazyswarm (modified)

The documentation for the original package: http://crazyswarm.readthedocs.io/en/latest/.



This package includes some additions to the original crazyswarm code mentioned above, which are:

1) Adjustemnts to the IMU and flow deck readings to account for flipping the crazyflie mounting.

2) A geometric tracking controller (GTC) onboard the crazyflie.

3) A new setpoint of the form (mode, command1, command2, command3) for the GTC, where the modes are (2: velocity, 1: 3:attitude, 4: attitudeRate).

4) A ros node that subscribes to the full compressed state of the crazyflie ,which includes a compressed form of x and y (StateCompressedZ.xy). The node decompresses the state data and publishes to a new topic with msg type Float32MultiArray.

## Usage

### Installation and Dependencies

Install Ubuntu 18.04 and ROS Melodic: http://wiki.ros.org/melodic/Installation/Ubuntu.

Get the following dependencies.
```
$ sudo apt install git swig libpython-dev python-numpy python-yaml python-matplotlib libpcl-dev libusb-1.0-0-dev sdcc ros-melodic-vrpn-client-ros python3-pip
```
Clone the repository to your home directory

``` 
$ git clone https://github.com/bma5287/crazyswarm.git
```

Then, get the latest arm embedded toolchain from https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm. You can also just enter the following in the command line

```
$ sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
$ sudo apt-get update
$ sudo apt install gcc-arm-embedded
```

Once you have everything installed, run the build file provided by USC-ACTLab, and source the workspace

```
$ cd crazyswarm
$ ./build.sh
$ echo "source ~/crazyswarm/ros_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

### Change Crazyflie Address

Install and run the crazyflie client provided by Bitcraze. In the terminal, enter

```
$ cd crazyswarm/crazyflie-clients-python
$ pip3 install -e .
$ cfclient
```

Connect the crazyflie 2.X using a usb, and click scan. Once located, click connect. Then, in the top toolbar, click 1) connect, 2) Configure 2.X, and then change the value in the address field. Finally click write to save the changes. The crazyflie needs to be rebooted for the change to take effect. Make sure the address and channel match the one included in allCrazyflies.yaml. 

### Flash Crazyflie

Before flashing the crazyflie 2.X, use this link to enable usb permissions [usb permissions](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/0.1.9/usb_permissions/). Then, compile the firmware

```
$ cd crazyswarm/crazyflie-firmware
$ make PLATFORM=cf2
```

Then, hold down the crazyflie power button for a few seconds to enter bootloader mode. Then type

``` 
$ make cload
```

### State Sub Node

Since only 9 log variables are allowed, and 13 (position + attitudeQuaternion + velocity + attitudeRate) state variables are needed, compression is necessary. The quaternion compression function avaliable in the firmware, along with the added x and y position compression, reduce the variable count to 9. A maximum x/y value of +-32m is assumed when using the compression. 

This node handles decompressing the state information from the ros topic /cf1/log1 and converting the data into a Float32MultiArray of dimension 13 for easy integration with numpy in python. 

### Start Server and Gtc Example

For an example python script that collects the decompressed state infromation and published GTC commands, run

```
$ roslaunch crazyswarm hover_swarm.launch
````

Make sure the crazyflie is secured or in an enclosed area before running the example script. In a seperate terminal

``` 
$ cd crazyswarm/ros_ws/src/crazyswarm/scripts
$ python State_Sub_Pub_example.py
```

The two main features are the crazyflie full state subscriber and command publisher.

The subscriber is defined in a thread within the example class as

```python
from crazyflie_driver.msg import State
...
def state_sub(self):
        rospy.Subscriber('/cf1/state',State,self.state_callback)
        rospy.spin()
```

The subscribed topic is /cf1/state and the message type is State.

The function to publish GTC commands for a given crazyflie object is 

```python
cf.cmdGtc(mode,cmd1,cmd2,cmd3)
```


## Future Work:

1. Add onboard filter for angular rate estimation to reduce noise.
  * low pass butterworth
  * rolling average

2. Tune Gtc gains.


## Notes

Information regarding the firmware can be found here: [crazyflie-firmware](https://github.com/bma5287/crazyflie-firmware/tree/b07031c066ca2e6b82421c65d00cb49cb713bb74).

Information regarding the ROS message types can be found here: [crazyflie_ros](https://github.com/bma5287/crazyflie_ros/tree/8a9298572e30dd6c79639ba0ab4d101d58ff34a5).

Using this package assumes a crazyflie 2.X with the flow deck v2 mounted on top. This requires flipping the crazyflie board when assembling.
