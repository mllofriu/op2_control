#include <angles/angles.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string>

#include <MX28.h>
#include <JointData.h>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <ros/console.h>

#include "op2_control/op2_robot_hw.h"

namespace op2_control {
using namespace Robot;

const std::string OP2RobotHW::jointUIDs[JointData::NUMBER_OF_JOINTS + 3] = {
		"j_shoulder_r", "j_shoulder_l", "j_high_arm_r", "j_high_arm_l",
		"j_low_arm_r", "j_low_arm_l", "j_pelvis_r", "j_pelvis_l", "j_thigh1_r",
		"j_thigh1_l", "j_thigh2_r", "j_thigh2_l", "j_tibia_r", "j_tibia_l",
		"j_ankle1_r", "j_ankle1_l", "j_ankle2_r", "j_ankle2_l", "j_pan",
		"j_tilt", "j_wrist_r", "j_wrist_l", "j_gripper_r", "j_gripper_l", };

OP2RobotHW::OP2RobotHW(int wakeup_motion) :
		hardware_interface::RobotHW() {
	/**
	 * ROS control interface register
	 **/
	for (unsigned int id = 0; id < JointData::NUMBER_OF_JOINTS + 3; id++) {
		// connect and register the joint state interface
		hardware_interface::JointStateHandle joint_state_handle(jointUIDs[id],
				&states_[id].pos, &states_[id].vel, &states_[id].eff);
		joint_state_interface_.registerHandle(joint_state_handle);
		hardware_interface::PosVelAccPIDJointHandle joint_handle(
				joint_state_handle, &cmds_[id].pos, &cmds_[id].vel,
				&cmds_[id].eff, &cmds_[id].P, &cmds_[id].I, &cmds_[id].D);
		joint_posvelacc_pid_inteface_.registerHandle(joint_handle);
		prev_cmds_[id] = cmds_[id];
	}

	registerInterface(&joint_state_interface_);
	registerInterface(&joint_posvelacc_pid_inteface_);

	/** register sensors */
	// IMU
	imu_data_.name = "imu";
	imu_data_.frame_id = "MP_BODY";
	imu_data_.orientation = imu_orientation_;
	imu_data_.angular_velocity = imu_angular_velocity_;
	imu_data_.linear_acceleration = imu_linear_acceleration_;
	hardware_interface::ImuSensorHandle imu_sensor_handle(imu_data_);
	imu_sensor_interface_.registerHandle(imu_sensor_handle);
	registerInterface(&imu_sensor_interface_);

	/**
	 * OP Framework initialization
	 **/
	// Initialize ROBOTIS-OP  Framework
	cm730_device_ = std::string("/dev/ttyUSB0");
	cm730_device2_ = std::string("/dev/ttyUSB1");
	action_file_ = std::string("/robotis/Data/motion_4096.bin");
	config_file_ = std::string("/robotis/Data/config.ini");

	if (!(Action::GetInstance()->LoadFile((char *) action_file_.c_str()))) {
		ROS_ERROR("Reading Action File failed!");
	}

	linux_cm730_ = new LinuxCM730((char *) cm730_device_.c_str());
	cm730_ = new Robot::CM730(linux_cm730_);
	if (MotionManager::GetInstance()->Initialize(cm730_) == false) {
		linux_cm730_->SetPortName((char *) cm730_device2_.c_str());
		if (MotionManager::GetInstance()->Initialize(cm730_) == false) {
			ROS_ERROR("Fail to initialize Motion Manager!");
			ros::shutdown();
		}
	}

	minIni ini = minIni(config_file_);
	MotionManager::GetInstance()->LoadINISettings(&ini);
	MotionManager::GetInstance()->AddModule(
			(MotionModule*) Action::GetInstance());

	motion_timer_ = new LinuxMotionTimer(MotionManager::GetInstance());
	ROS_INFO("Starting Motion Timer...");
	motion_timer_->Start();
	ROS_INFO("Finished starting Motion Timer");

	MotionManager::GetInstance()->SetEnable(true);

	/** Init(stand up) pose */
	ROS_INFO("Wake up position is %d", wakeup_motion);
	if (Action::GetInstance()->Start(wakeup_motion))
		ROS_INFO("Moving to wake up position ...");
	else
		ROS_ERROR("Wake up action failed");
	while (Action::GetInstance()->IsRunning()) {
		usleep(8 * 1000);
	}

	MotionManager::GetInstance()->AddModule(
			(MotionModule*) Walking::GetInstance());
	MotionManager::GetInstance()->AddModule(
			(MotionModule*) Head::GetInstance());
	MotionStatus::m_CurrentJoints.SetEnableBody(true, true);
	Walking::GetInstance()->LoadINISettings(&ini);
}

void OP2RobotHW::readMotors(int * ids, int numMotors) {
	cm730_->MakeBulkReadMotorData(ids, numMotors);
	cm730_->BulkRead();
	for (int id = 0; id < numMotors; id++) {
		if (cm730_->m_BulkReadData[ids[id]].error > 0) {
			ROS_ERROR("Error on read data for motor %d", ids[id]);
		} else {
			int pos = cm730_->m_BulkReadData[ids[id]].ReadWord(
					MX28::P_PRESENT_POSITION_L);
			states_[ids[id] - 1].pos = MX28::Value2Angle(pos) * (M_PI / 180.0);
			int vel = cm730_->m_BulkReadData[ids[id]].ReadWord(
					MX28::P_PRESENT_SPEED_L);
			states_[ids[id] - 1].vel = MX28::Value2Angle(vel) * (M_PI / 180.0);
			int load = cm730_->m_BulkReadData[ids[id]].ReadWord(
					MX28::P_PRESENT_LOAD_L);
			states_[ids[id] - 1].eff = load;
		}
	}
}

void OP2RobotHW::read() {
	int rLeg[6] = { 7, 9, 11, 13, 15, 17 };
	readMotors(rLeg, 6);
	int lLeg[6] = { 8, 10, 12, 14, 16, 18 };
	readMotors(lLeg, 6);
	int rArm[3] = { 1, 3, 5 };
	readMotors(rArm, 3);
	int lArm[3] = { 2, 4, 6 };
	readMotors(lArm, 3);
	int head[2] = { 19, 20 };
	readMotors(head, 2);
}

void OP2RobotHW::write() {
	int param[(JointData::NUMBER_OF_JOINTS - 1) * 9];
	int n = 0;
	int joint_num = 0;
	for (int id = 1; id < JointData::NUMBER_OF_JOINTS; id++) {
		// Only write if cmd has changed
		if (cmds_[id - 1] != prev_cmds_[id - 1]) {
			param[n++] = id;

			// Set PID
			param[n++] = floor(cmds_[id - 1].D);
			param[n++] = floor(cmds_[id - 1].I);
			param[n++] = floor(cmds_[id - 1].P);
			// Reserved position
			param[n++] = 0;

			// Set position
			int value = MX28::Angle2Value(cmds_[id - 1].pos * 180.0 / M_PI);
			param[n++] = CM730::GetLowByte(value);
			param[n++] = CM730::GetHighByte(value);
			// Velocity goes from 0..1023. each unit is 0.114 rpm = 0.0119381 rad/sec
			// so vel in motor is vel in rad/2 / 0.0119381
			int vel = cmds_[id - 1].vel / 0.0119381;
			param[n++] = CM730::GetLowByte(vel);
			param[n++] = CM730::GetHighByte(vel);
			joint_num++;

			prev_cmds_[id] = cmds_[id];
		}
	}

	if (joint_num > 0)
		cm730_->SyncWrite(MX28::P_D_GAIN, 9, joint_num, param);
}

} // namespace

int main(int argc, char** argv) {
	ros::init(argc, argv, "op2_robot_hw");

	ros::NodeHandle nh;

	ros::NodeHandle private_nh("~");

	int wakeup_motion;
	private_nh.getParam("wakeup_motion", wakeup_motion);

	double control_rate;
	private_nh.param("control_rate", control_rate, 100.0);
	ROS_INFO("Controlling at %f Hz", control_rate);

	op2_control::OP2RobotHW op_robot_hw(wakeup_motion);
	controller_manager::ControllerManager cm(&op_robot_hw);

	ros::AsyncSpinner spinner(4);
	spinner.start();

	ros::Time last_time = ros::Time::now();
	ros::Time current_time;
	ros::Rate rate(control_rate);

	while (ros::ok()) {
		rate.sleep();
		ros::Time current_time = ros::Time::now();
		ros::Duration elapsed_time = current_time - last_time;

		op_robot_hw.read();
		cm.update(current_time, elapsed_time);
		op_robot_hw.write();

		last_time = current_time;
	}

}
