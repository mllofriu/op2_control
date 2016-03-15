/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  Copyright (c) 2012, hiDOF, Inc.
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  Copyright (c) 2014, Fraunhofer IPA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "op2_control/walking_controller.h"

namespace op2_control {

using namespace Robot;

bool WalkingController::init(
		hardware_interface::PosVelAccPIDJointInterface * hw,
		ros::NodeHandle &n) {
	// List of controlled joints
	std::string param_name = "joints";
	if (!n.getParam(param_name, joint_names_)) {
		ROS_ERROR_STREAM(
				"Failed to getParam '" << param_name << "' (namespace: "
						<< n.getNamespace() << ").");
		return false;
	}
	n_joints_ = joint_names_.size();

	if (n_joints_ == 0) {
		ROS_ERROR_STREAM("List of joint names is empty.");
		return false;
	}

	// Request all joints to ensure exclusive access
	for (unsigned int i = 0; i < n_joints_; i++) {
		try {
			hw->getHandle(joint_names_[i]);
		} catch (const hardware_interface::HardwareInterfaceException& e) {
			ROS_ERROR_STREAM("Exception thrown: " << e.what());
			return false;
		}
	}

	// Hack to set cm730 already connected
//	CM730 * cm730 = hw->getHandle(joint_names_[0]).getCM730();
////	MotionManager::GetInstance()->setCM730(cm730);
//	MotionManager::GetInstance()->Initialize(cm730);
	m_manager_ = hw->getHandle(joint_names_[0]).getMManager();
	action_ = hw->getHandle(joint_names_[0]).getAction();
	walking_ = hw->getHandle(joint_names_[0]).getWalking();

//	ROS_INFO("Starting Robotis framework...");
//
//	ROS_INFO("Finished initializing robotis op framework");

	ROS_INFO("Subscribing to topics");
	ros::NodeHandle nh;
	start_action_sub_ = nh.subscribe("start_action", 10,
			&WalkingController::start_action, this);
	cmd_vel_sub_ = nh.subscribe("cmd_vel", 1, &WalkingController::cmd_vel_cb,
			this);
	enable_walking_sub_ = nh.subscribe("enable_walking", 1,
			&WalkingController::enable_walking, this);

	ROS_INFO("Setting up dynamic reconfigure");
	dynamic_reconfigure::Server < op2_control::WalkingConfig > server;
	server.setCallback(boost::bind(&WalkingController::reconfigure, this, _1, _2));

	ROS_INFO("Walker controller initialized");
	return true;
}

void WalkingController::starting(const ros::Time& time) {
	// Wake up motion to crouch
//	std_msgs::Int32 msg = std_msgs::Int32();
//	msg.data = 15;
//	start_action(msg);
//
//	ros::Duration(10).sleep();
//
//	ROS_INFO("Starting walk");
//	walking_->m_Joint.SetEnableBodyWithoutHead(true, true);
//	walking_->Start();
}

void WalkingController::start_action(std_msgs::Int32 action) {
	/** Init(stand up) pose */
//	action_->m_Joint.SetEnableBody(true, true);
//	m_manager_->SetEnable(true);
	if (action_->Start(action.data))
		ROS_INFO("Executing action %d ...", action.data);
	else
		ROS_ERROR("Action execution failed");
	while (action_->IsRunning()) {
		usleep(8 * 1000);
	}
	ROS_INFO("Action execution completed");
}

void WalkingController::cmd_vel_cb(const geometry_msgs::Twist::ConstPtr msg) {
	double period = walking_->PERIOD_TIME;
	walking_->X_MOVE_AMPLITUDE = (msg->linear.x / period * 1000.0
			* 10.0);
	walking_->Y_MOVE_AMPLITUDE = (msg->linear.y / period * 1000.0
			* 10.0);
	// compute the angular motion parameters to achieve the desired angular speed
	walking_->A_MOVE_AMPLITUDE = (msg->angular.z / period); // in rad per sec
	// ROS_INFO("Walking: periode %f x %f y %f a %f",walking_->PERIOD_TIME,walking_->X_MOVE_AMPLITUDE,walking_->Y_MOVE_AMPLITUDE,walking_->A_MOVE_AMPLITUDE);
}

void WalkingController::enable_walking(std_msgs::BoolConstPtr enable) {
	if (enable->data) {
		walking_->Start();
	} else {
		walking_->Stop();
	}
}

void WalkingController::reconfigure(op2_control::WalkingConfig &config,
		uint32_t level) {
	walking_->X_OFFSET = config.x_offset;
	walking_->Y_OFFSET = config.y_offset;
	walking_->Z_OFFSET = config.z_offset;
	walking_->A_OFFSET = config.yaw_offset;
	walking_->R_OFFSET = config.roll_offset;
	walking_->P_OFFSET = config.pitch_offset;
	walking_->HIP_PITCH_OFFSET = config.hip_pitch_offset;
	walking_->PERIOD_TIME = config.period_time;
	walking_->DSP_RATIO = config.dsp_ratio;
	walking_->STEP_FB_RATIO = config.step_forward_back_ratio;
	walking_->BALANCE_ANKLE_PITCH_GAIN =
			config.balance_ankle_pitch_gain;
	walking_->BALANCE_ANKLE_ROLL_GAIN =
			config.balance_ankle_roll_gain;
	walking_->Z_MOVE_AMPLITUDE = config.foot_height;
	walking_->Y_SWAP_AMPLITUDE = config.swing_right_left;
	walking_->Z_SWAP_AMPLITUDE = config.swing_top_down;
	walking_->PELVIS_OFFSET = config.pelvis_offset;
	walking_->P_GAIN = config.p_gain;
	walking_->D_GAIN = config.d_gain;
	walking_->I_GAIN = config.i_gain;
}

}

PLUGINLIB_EXPORT_CLASS(op2_control::WalkingController,
		controller_interface::ControllerBase)
