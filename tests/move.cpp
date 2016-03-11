#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <sensor_msgs/JointState.h>

#include "robotis_op_ros_control/PIDPosVelAcc.h"

std::string joints[20] = {
   "j_shoulder_r_position_controller", // 1
   "j_shoulder_l_position_controller", // 2
   "j_high_arm_r_position_controller", // 3
   "j_high_arm_l_position_controller", // 4
   "j_low_arm_r_position_controller", // 5
   "j_low_arm_l_position_controller", // 6
   "j_pelvis_r_position_controller", // 7
   "j_pelvis_l_position_controller", // 8
   "j_thigh1_r_position_controller", // 9
   "j_thigh1_l_position_controller", // 10
   "j_thigh2_r_position_controller", // 11
   "j_thigh2_l_position_controller", // 12
   "j_tibia_r_position_controller", // 13
   "j_tibia_l_position_controller", // 14
   "j_ankle1_r_position_controller", // 15
   "j_ankle1_l_position_controller", // 16
   "j_ankle2_r_position_controller", // 17
   "j_ankle2_l_position_controller", // 18
   "j_pan_position_controller", // 19
   "j_tilt_position_controller" // 20
};

int stateMap[20] = {15, 14, 8, 7, 10, 9, 13, 12, 17, 16, 19, 18,21, 20, 2, 1, 4, 3, 11, 22};

using namespace robotis_op_ros_control;
using namespace message_filters;

/**
 * Get a controller of the joint
**/
ros::Publisher getPublisher(ros::NodeHandle & nh, int jointNum)
{
  std::string joint = joints[jointNum - 1];
	std::string topic = "/" + joint + "/commandMsg";
	ROS_INFO("Getting controller for joint %s", topic.c_str());
	return nh.advertise<PIDPosVelAcc>(topic.c_str(), 1);
}

/**
 * Get a joint position
**/
float getJointPos(Cache<sensor_msgs::JointState> &cache, int jointNum)
{
  sensor_msgs::JointStateConstPtr last = cache.getElemBeforeTime(ros::Time::now());
  ROS_INFO("Knee at pos: %f",last->position[stateMap[jointNum-1] - 1]);
  return last->position[stateMap[jointNum-1] - 1];
}


/**
 * Send a command to joint to go to pos (rads), using vel (rads/s) with a p gain
**/
void sendCmd(ros::Publisher pub, float pos, float vel, float p)
{
  PIDPosVelAcc msg;
  msg.pos = pos;
  msg.vel = vel;
  msg.P = p;
  pub.publish(msg);
}

int main (int argc, char ** argv)
{
  // ROS Boilerplate
	ros::init(argc, argv, "move_tester");
	ros::NodeHandle nh;

  Subscriber<sensor_msgs::JointState> sub(nh, "/joint_states", 1);
  Cache<sensor_msgs::JointState> cache(sub, 1);
	
  // Get joint controllers
  int rightKneeNum = 13;
  int leftKneeNum = 14;
	ros::Publisher right_knee = getPublisher(nh, rightKneeNum);
  ros::Publisher left_knee = getPublisher(nh, leftKneeNum);

  // Get all controllers before this sleep
  ros::Duration(2).sleep();


  //sendCmd(right_knee, 2.0f, 12.0f, 32.0f);
  sendCmd(left_knee, -2.0f, 1.0f, 32.0f);

  ros::Duration(2).sleep();
  
  sendCmd(left_knee, 0.0f, 1.0f, 32.0f);

  // Wait for leftKnee to be over 1 rad
  // We check 10 times per second for changes
  ros::Rate rate(10); 
  do {
      ros::spinOnce();
      rate.sleep();
  } while (ros::ok() && getJointPos(cache, leftKneeNum) < -1.0);

  //sendCmd(right_knee, -2.0f, 12.0f, 32.0f);
  sendCmd(left_knee, -1.0f, 4.0f, 32.0f);

  ros::Duration(3).sleep();


	

}

