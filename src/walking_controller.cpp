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

#include <ros/node_handle.h>
#include <ros/console.h>
#include <controller_interface/controller.h>

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
			ros::NodeHandle &n) {
		// List of controlled joints
		std::string param_name = "joints";
		if (!n.getParam(param_name, joint_names_)) {
			ROS_ERROR_STREAM(
					"Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
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

		std::string action_file_ = std::string("/robotis/Data/motion_4096.bin");
		std::string config_file_ = std::string("/robotis/Data/config.ini");

		if (!(Action::GetInstance()->LoadFile((char *) action_file_.c_str()))) {
			ROS_ERROR("Reading Action File failed!");
		}

		minIni ini = minIni(config_file_);
		MotionManager::GetInstance()->LoadINISettings(&ini);
		MotionManager::GetInstance()->AddModule(
				(MotionModule*) Action::GetInstance());

		LinuxMotionTimer* motion_timer_ = new LinuxMotionTimer(
				MotionManager::GetInstance());
		ROS_INFO("Starting Motion Timer...");
		motion_timer_->Start();
		ROS_INFO("Finished starting Motion Timer");

		MotionManager::GetInstance()->SetEnable(true);

		MotionManager::GetInstance()->AddModule(
				(MotionModule*) Walking::GetInstance());
		MotionManager::GetInstance()->AddModule(
				(MotionModule*) Head::GetInstance());
		MotionStatus::m_CurrentJoints.SetEnableBody(true, true);
		Walking::GetInstance()->LoadINISettings(&ini);

		ros::NodeHandle nh;
		start_action_sub_ = nh.subscribe("start_action", 10,
				&WalkingController::start_action, this);
		cmd_vel_sub_ = nh.subscribe("cmd_vel", 1,
				&WalkingController::cmd_vel_cb, this);
		enable_walking_sub_ = nh.subscribe("enable_walking", 1,
				&WalkingController::enable_walking, this);

		dynamic_reconfigure::Server<op2_control::WalkingConfig> server;
		dynamic_reconfigure::Server<op2_control::WalkingConfig>::CallbackType f;

		f = boost::bind(&WalkingController::reconfigure, _1, _2);
		server.setCallback(f);


		return true;
	}

	void starting(const ros::Time& time) {
		// Wake up motion to crouch
		std_msgs::Int32 msg = std_msgs::Int32();
		msg.data = 15;
		start_action(msg);
	}

	void update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
	}

	void start_action(std_msgs::Int32 action) {
		/** Init(stand up) pose */
		if (Action::GetInstance()->Start(action.data))
			ROS_INFO("Executing action %d ...", action.data);
		else
			ROS_ERROR("Action execution failed");
		while (Action::GetInstance()->IsRunning()) {
			usleep(8 * 1000);
		}
	}

	void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr msg) {
		double period = Walking::GetInstance()->PERIOD_TIME;
		Walking::GetInstance()->X_MOVE_AMPLITUDE = (msg->linear.x / period
				* 1000.0 * 10.0);
		Walking::GetInstance()->Y_MOVE_AMPLITUDE = (msg->linear.y / period
				* 1000.0 * 10.0);
		// compute the angular motion parameters to achieve the desired angular speed
		Walking::GetInstance()->A_MOVE_AMPLITUDE = (msg->angular.z / period); // in rad per sec
		// ROS_INFO("Walking: periode %f x %f y %f a %f",Walking::GetInstance()->PERIOD_TIME,Walking::GetInstance()->X_MOVE_AMPLITUDE,Walking::GetInstance()->Y_MOVE_AMPLITUDE,Walking::GetInstance()->A_MOVE_AMPLITUDE);
	}

	void enable_walking(std_msgs::BoolConstPtr enable) {
		if(enable->data){
			Walking::GetInstance()->Start();
		} else {
			Walking::GetInstance()->Stop();
		}
	}

	void reconfigure(op2_control::WalkingConfig &config, uint32_t level) {
		Walking::GetInstance()->X_OFFSET = config.x_offset;
		Walking::GetInstance()->Y_OFFSET = config.y_offset;
		Walking::GetInstance()->Z_OFFSET = config.z_offset;
		Walking::GetInstance()->A_OFFSET = config.yaw_offset;
		Walking::GetInstance()->R_OFFSET = config.roll_offset;
		Walking::GetInstance()->P_OFFSET = config.pitch_offset;
		Walking::GetInstance()->HIP_PITCH_OFFSET = config.hip_pitch_offset;
		Walking::GetInstance()->PERIOD_TIME = config.period_time;
		Walking::GetInstance()->DSP_RATIO = config.dsp_ratio;
		Walking::GetInstance()->STEP_FB_RATIO = config.step_forward_back_ratio;
		Walking::GetInstance()->BALANCE_ANKLE_PITCH_GAIN = config.balance_ankle_pitch_gain;
		Walking::GetInstance()->BALANCE_ANKLE_ROLL_GAIN = config.balance_ankle_roll_gain;
		Walking::GetInstance()->Z_MOVE_AMPLITUDE = config.foot_height;
		Walking::GetInstance()->Y_SWAP_AMPLITUDE = config.swing_right_left;
		Walking::GetInstance()->Z_SWAP_AMPLITUDE = config.swing_top_down;
		Walking::GetInstance()->PELVIS_OFFSET = config.pelvis_offset;
		Walking::GetInstance()->P_GAIN = config.p_gain;
		Walking::GetInstance()->D_GAIN = config.d_gain;
		Walking::GetInstance()->I_GAIN = config.i_gain;
	}

	std::vector<std::string> joint_names_;
	unsigned int n_joints_;
	ros::Subscriber start_action_sub_;
	ros::Subscriber cmd_vel_sub_;
	ros::Subscriber enable_walking_sub_;
};

}

PLUGINLIB_EXPORT_CLASS(op2_control::WalkingController,
		controller_interface::ControllerBase)
