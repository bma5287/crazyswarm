#include "ros/ros.h"
#include "crazyflie_driver/GenericLogData.h"
#include "crazyflie_driver/State.h"
#include "std_msgs/Float32MultiArray.h"
#include <stdint.h>

ros::Publisher state_pub;


// Function taken from crazyflie firmware , quatcompress.h
static inline void quatdecompress(uint32_t comp, float q[4])
{
	unsigned const mask = (1 << 9) - 1;

	int const i_largest = comp >> 30;
	float sum_squares = 0;
	for (int i = 3; i >= 0; --i) {
		if (i != i_largest) {
			unsigned mag = comp & mask;
			unsigned negbit = (comp >> 9) & 0x1;
			comp = comp >> 10;
			q[i] = ((float)M_SQRT1_2) * ((float)mag) / mask;
			if (negbit == 1) {
				q[i] = -q[i];
			}
			sum_squares += q[i] * q[i];
		}
	}
	q[i_largest] = sqrtf(1.0f - sum_squares);
}


void state_callback (const crazyflie_driver::GenericLogData::ConstPtr& msg) 
{
  double x,y,z,vx,vy,vz,wx,wy,wz;
  uint32_t quatc;
  float quat[4];
  float full_state[13];

  double const deg2millirad = ((double)M_PI * 1000.0) / 180.0;

  // x and y have max values of +-32 meters
  uint32_t xy = (uint32_t)msg->values[0];
  uint16_t xd = (uint32_t)xy & 0xFFFF;
  uint16_t yd = ((uint32_t)xy >> 16) & 0xFFFF;
  x = ((float)xd - 32767.0f)/1000.0f;
  y = ((float)yd - 32767.0f)/1000.0f;

  //ROS_INFO("xy = %d \t (x,y) = (%lf,%lf)  \t actual (x,y) = (%f,%f)",xy,x,y,x2,y2);

  wz = msg->values[1]; 
  z = msg->values[2]/1000;

  vx = msg->values[3]/1000;  vy = msg->values[4]/1000;  vz = msg->values[5]/1000;

  quatc = (uint32_t)msg->values[6];
  quatdecompress(quatc,quat);

  wx = msg->values[7]/deg2millirad;  wy = msg->values[8]/deg2millirad; //wz = 0.0;

  /*ROS_INFO("(x,y,z) = (%f,%f,%f) \t (vx,vy,vz) = (%f,%f,%f) \t (q1,q2,q3,q4) = (%f,%f,%f,%f) \t (wx,wy,wz) = (%f,%f,%f)",
              x,y,z,vx,vy,vz,quat[0],quat[1],quat[2],quat[3],wx,wy,wz);
  */

  full_state[0] = x; full_state[1] = y; full_state[2] = z;
  full_state[3] = quat[3]; full_state[4] = quat[0]; full_state[5] = quat[1]; full_state[6] = quat[2];
  full_state[7] = vx; full_state[8] = vy; full_state[9] = vz;
  full_state[10] = wx; full_state[11] = wy; full_state[12] = wz;

  crazyflie_driver::State state_msg;
  std_msgs::Float32MultiArray state;

  for (int i = 0; i < 13; i++) 
  {
  state.data.insert(state.data.end(), full_state[i]);
  }

  state_msg.state = state;
  state_msg.header.stamp = ros::Time::now();
  state_msg.header.seq += 1;

  state_pub.publish(state_msg);
}


int main(int argc, char **argv)
{

  ros::init(argc,argv,"crazyflie_state");
  ros::NodeHandle n;

  ros::Subscriber state_sub = n.subscribe("/cf1/log1", 1000, state_callback);
  state_pub = n.advertise<crazyflie_driver::State>("/cf1/state", 1000);
  //ros::Publisher state_pub = n.publisher("/cf1/state",State,anonymous=true);

  ros::spin();

  return 0;

}