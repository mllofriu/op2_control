#ifndef C_INTERFACE_H_
#define ROBOTIS_OP_INTERFACE_H_

#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Eigen>

// Roobotis framework
#include <LinuxDARwIn.h>

// ROS
#include <ros/ros.h>
#include <tf/tf.h>

// ROS Messages
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

// ROS Control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/robot_hw.h>

#include "op2_control/PIDPosVelAcc.h"
#include "op2_control/posvelacc_pid_command_interface.h"
#include "op2_control/msg_command_inteface.h"


namespace op2_control
{

class OP2RobotHW :
    public hardware_interface::RobotHW
{
public:
    OP2RobotHW(int wakeup_motion);
    ~OP2RobotHW(){};

    // ROS Control
    void read();
    void write();

    // Motion functions
    void cmdWalking(const geometry_msgs::Twist::ConstPtr& msg);
    void enableWalking(std_msgs::BoolConstPtr enable);
    void checkFall();
    void startAction(std_msgs::Int32 action_index);

private:
    // Helper functions
    void readMotors(int * ids, int numMotors);

    // UIDs of joints and sensors
    static const std::string jointUIDs[Robot::JointData::NUMBER_OF_JOINTS+3];

    // ros control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PosVelAccPIDJointInterface joint_posvelacc_pid_inteface_;
    hardware_interface::ImuSensorInterface imu_sensor_interface_;

    // Status arrays containing values for each joint (+ dummy joints to model the gripper)
    // vel_dummy_ and eff_dummy_ are required to meet the interface, but not actually used
    // robotis_op_ros_control::PIDPosVelAcc cmd_[Robot::JointData::NUMBER_OF_JOINTS];
    double pos_[Robot::JointData::NUMBER_OF_JOINTS+3];
    double vel_[Robot::JointData::NUMBER_OF_JOINTS+3];
    double eff_dummy_[Robot::JointData::NUMBER_OF_JOINTS+3];
    double cmd_pos_[Robot::JointData::NUMBER_OF_JOINTS+3];
    double cmd_vel_[Robot::JointData::NUMBER_OF_JOINTS+3];
    double cmd_eff_dummy_[Robot::JointData::NUMBER_OF_JOINTS+3];
    double cmd_p_[Robot::JointData::NUMBER_OF_JOINTS+3];
    double cmd_i_[Robot::JointData::NUMBER_OF_JOINTS+3];
    double cmd_d_[Robot::JointData::NUMBER_OF_JOINTS+3];

    // IMU
    hardware_interface::ImuSensorHandle::Data imu_data_;
    double imu_orientation_[4];
    double imu_angular_velocity_[3];
    double imu_linear_acceleration_[3];


    //Robot
    Robot::LinuxCM730 * linux_cm730_;
    Robot::CM730 * cm730_;
    Robot::LinuxMotionTimer *motion_timer_;
    std::string cm730_device_, cm730_device2_, action_file_, config_file_;
};
}

#endif

