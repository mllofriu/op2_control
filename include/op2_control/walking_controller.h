/*
 * walking_controller.h
 *
 *  Created on: Mar 14, 2016
 *      Author: ludo
 */

#ifndef INCLUDE_OP2_CONTROL_WALKING_CONTROLLER_H_
#define INCLUDE_OP2_CONTROL_WALKING_CONTROLLER_H_

#include <ros/node_handle.h>
#include <ros/console.h>
#include <controller_interface/controller.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <dynamic_reconfigure/server.h>

#include <position_controllers/joint_position_controller.h>
#include <pluginlib/class_list_macros.h>

// Roobotis framework
#include <LinuxDARwIn.h>

#include "op2_control/posvelacc_pid_command_interface.h"
#include "op2_control/WalkingConfig.h"

namespace op2_control {

using namespace Robot;

/**
 * \brief Control all joints to perform a goalie jumb
 *
 * \section ROS interface
 *
 * \param joints Names of the joints to control.
 */
class WalkingController: public controller_interface::Controller<
		hardware_interface::PosVelAccPIDJointInterface> {
public:
	WalkingController() {
	}
	~WalkingController() {
	}

	bool init(hardware_interface::PosVelAccPIDJointInterface * hw,
			ros::NodeHandle &n);
	void starting(const ros::Time& time);

	void update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
	}

	void start_action(std_msgs::Int32 action);

	void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr msg);

	void enable_walking(std_msgs::BoolConstPtr enable);

	void reconfigure(op2_control::WalkingConfig &config, uint32_t level);

	std::vector<std::string> joint_names_;
	unsigned int n_joints_;
	ros::Subscriber start_action_sub_;
	ros::Subscriber cmd_vel_sub_;
	ros::Subscriber enable_walking_sub_;

	MotionManager * m_manager_;
	Action * action_;
	Walking * walking_;
};

}

#endif /* INCLUDE_OP2_CONTROL_WALKING_CONTROLLER_H_ */
