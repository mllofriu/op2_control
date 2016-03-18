#include <angles/angles.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string>

#include <pthread.h>
#include <stdlib.h>

#include <MX28.h>
#include <CM730.h>
#include <JointData.h>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <dynamixel_sdk/PacketHandler.h>

#include "op2_control/op2_robot_hw.h"

namespace op2_control {
using namespace Robot; // TODO: needed?
using namespace ROBOTIS;
const std::string OP2RobotHW::jointUIDs[JointData::NUMBER_OF_JOINTS + 3] = {
		"j_shoulder_r", "j_shoulder_l", "j_high_arm_r", "j_high_arm_l",
		"j_low_arm_r", "j_low_arm_l", "j_pelvis_r", "j_pelvis_l", "j_thigh1_r",
		"j_thigh1_l", "j_thigh2_r", "j_thigh2_l", "j_tibia_r", "j_tibia_l",
		"j_ankle1_r", "j_ankle1_l", "j_ankle2_r", "j_ankle2_l", "j_pan",
		"j_tilt", "j_wrist_r", "j_wrist_l", "j_gripper_r", "j_gripper_l", };

OP2RobotHW::OP2RobotHW() :
		hardware_interface::RobotHW() {
	/**
	 * OP Framework initialization
	 **/
	// Initialize ROBOTIS-OP  Framework
//	cm730_device_ = std::string("/dev/ttyUSB0");
//	cm730_device2_ = std::string("/dev/ttyUSB1");
//	std::string action_file_ = std::string("/robotis/Data/motion_4096.bin");
//	std::string config_file_ = std::string("/robotis/Data/config.ini");
//
//	ROS_INFO("Initializing CM730");
//	linux_cm730_ = new LinuxCM730((char *) cm730_device_.c_str());
//	cm730_ = new Robot::CM730(linux_cm730_);
//	if (MotionManager::GetInstance()->Initialize(cm730_) == false) {
//		linux_cm730_->SetPortName((char *) cm730_device2_.c_str());
//		if (MotionManager::GetInstance()->Initialize(cm730_) == false) {
//			ROS_ERROR("Fail to initialize Motion Manager!");
//			ros::shutdown();
//		}
//	}
//	if ((Action::GetInstance()->LoadFile((char *) action_file_.c_str())) == false) {
//		ROS_ERROR("Reading Action File failed!");
//	}
//	minIni ini = minIni(config_file_);
//	MotionManager::GetInstance()->LoadINISettings(&ini);
//	MotionManager::GetInstance()->AddModule(
//			(MotionModule*) Action::GetInstance());
//
//	LinuxMotionTimer* motion_timer_ = new LinuxMotionTimer(
//			MotionManager::GetInstance());
//	motion_timer_->Start();
//
//	Action::GetInstance()->m_Joint.SetEnableBody(true, true);
//	MotionManager::GetInstance()->SetEnable(true);
//
//	MotionManager::GetInstance()->AddModule(
//			(MotionModule*) Walking::GetInstance());
//	MotionManager::GetInstance()->AddModule(
//			(MotionModule*) Head::GetInstance());
////	MotionStatus::m_CurrentJoints.SetEnableBody(true, true);
//	Walking::GetInstance()->LoadINISettings(&ini);
	PacketHandler * pkt_handler = PacketHandler::GetPacketHandler(1.0);
	PortHandler * port_handler = (PortHandler*)PortHandler::GetPortHandler("/dev/ttyUSB0");
	if (port_handler->OpenPort()) {
		ROS_INFO("Succeeded to open the port!\n");
	} else {
		ROS_ERROR("Failed to open the port!\n");
		ROS_ERROR("Press any key to terminate...\n");
	}
	if (port_handler->SetBaudRate(1000000)) {
		printf("Succeeded to change the baudrate!\n");
	} else {
		ROS_ERROR("Failed to change the baudrate!\n");
		ROS_ERROR("Press any key to terminate...\n");
	}

	// TODO:
	// for loop ping --> no response : return false
	ROS_INFO("Enable motors power");
	int dxl_comm_result;
	dxl_comm_result = pkt_handler->Write1ByteTxOnly(port_handler, CM730::ID_CM, CM730::P_DXL_POWER, 1);
	if (dxl_comm_result != COMM_SUCCESS)
		ROS_ERROR("Could not activate MX-28 power");

	for (int id = 1; id < 20; id++) {
		UINT8_T dxl_error = 0;                                  // DXL error
		UINT16_T dxl_model_number;                           // DXL model number
		dxl_comm_result = COMM_TX_FAIL;
		dxl_comm_result = pkt_handler->Ping(port_handler, id, &dxl_model_number,
				&dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
			ROS_INFO("Joint %id did not respond, error: %d", id, dxl_comm_result);
		else
			ROS_INFO("Joint %id responded", id);
	}

	/**
	 * ROS control interface register
	 **/
	for (unsigned int id = 0; id < JointData::NUMBER_OF_JOINTS + 3; id++) {
		// connect and register the joint state interface
		hardware_interface::JointStateHandle joint_state_handle(jointUIDs[id],
				&states_[id].pos, &states_[id].vel, &states_[id].eff);
		joint_state_interface_.registerHandle(joint_state_handle);
		hardware_interface::PosVelAccPIDJointHandle joint_handle(
				joint_state_handle, &(cmds_[id].pos), &(cmds_[id].vel),
				&(cmds_[id].eff), &(cmds_[id].P), &(cmds_[id].I),
				&(cmds_[id].D), MotionManager::GetInstance(),
				Action::GetInstance(), Walking::GetInstance());
		joint_posvelacc_pid_inteface_.registerHandle(joint_handle);

	}

	registerInterface(&joint_state_interface_);
	registerInterface(&joint_posvelacc_pid_inteface_);

	/** register sensors */
	// IMU
	/*imu_data_.name = "imu";
	 imu_data_.frame_id = "MP_BODY";
	 imu_data_.orientation = imu_orientation_;
	 imu_data_.angular_velocity = imu_angular_velocity_;
	 imu_data_.linear_acceleration = imu_linear_acceleration_;
	 hardware_interface::ImuSensorHandle imu_sensor_handle(imu_data_);
	 imu_sensor_interface_.registerHandle(imu_sensor_handle);
	 registerInterface(&imu_sensor_interface_);*/

	// Send 0 p gain and enable torque
	for (unsigned int id = 0; id < JointData::NUMBER_OF_JOINTS + 3; id++) {
		cmds_[id].P = 0;
		prev_cmds_[id] = cmds_[id];
	}
	// Force write to all motors for the first time
	this->write(true);
	// Enable torque

	ROS_INFO("OP robot created");
}

int OP2RobotHW::readMotors(int * ids, int numMotors) {
//	cm730_->MakeBulkReadMotorData(ids, numMotors);
//	int ret_val = cm730_->BulkRead();
//	if (ret_val != CM730::SUCCESS) {
//		ROS_ERROR("Error reading bulk: %d", ret_val);
//	} else {
//		for (int id = 0; id < numMotors; id++) {
//			int error = cm730_->m_BulkReadData[ids[id]].error;
//			if (error == 0) {
//				int pos = cm730_->m_BulkReadData[ids[id]].ReadWord(
//						MX28::P_PRESENT_POSITION_L);
//				states_[ids[id] - 1].pos = MX28::Value2Angle(pos)
//						* (M_PI / 180.0);
//				int vel = cm730_->m_BulkReadData[ids[id]].ReadWord(
//						MX28::P_PRESENT_SPEED_L);
//				states_[ids[id] - 1].vel = MX28::Value2Angle(vel)
//						* (M_PI / 180.0);
//				int load = cm730_->m_BulkReadData[ids[id]].ReadWord(
//						MX28::P_PRESENT_LOAD_L);
//				states_[ids[id] - 1].eff = load;
//			} else if (error == -1) {
//				ROS_ERROR("Packet not received for motor %d", ids[id]);
//			} else {
//				ROS_ERROR("Error on read data for motor %d, with error num %d",
//						ids[id], error);
//			}
//		}
//	}
	int ret_val = 0;
	for (int id = 0; id < numMotors; id++) {

		int value;
		int error;
		int read_return = cm730_->ReadWord(ids[id], MX28::P_PRESENT_POSITION_L,
				&value, &error);
		if (read_return == CM730::SUCCESS) {
			states_[ids[id] - 1].pos = MX28::Value2Angle(value)
					* (M_PI / 180.0);
		} else {
			ROS_ERROR(
					"Error reading position for joint %d, read return error: %d",
					ids[id], read_return);
			return read_return;
		}

//		int error = cm730_->m_BulkReadData[ids[id]].error;
//		if (error == 0) {
//			int pos = cm730_->m_BulkReadData[ids[id]].ReadWord(
//					MX28::P_PRESENT_POSITION_L);
//
//			int vel = cm730_->m_BulkReadData[ids[id]].ReadWord(
//					MX28::P_PRESENT_SPEED_L);
//			states_[ids[id] - 1].vel = MX28::Value2Angle(vel) * (M_PI / 180.0);
//			int load = cm730_->m_BulkReadData[ids[id]].ReadWord(
//					MX28::P_PRESENT_LOAD_L);
//			states_[ids[id] - 1].eff = load;
//		} else if (error == -1) {
//			ROS_ERROR("Packet not received for motor %d", ids[id]);
//		} else {
//			ROS_ERROR("Error on read data for motor %d, with error num %d",
//					ids[id], error);
//		}
	}

	return CM730::SUCCESS;
}

int OP2RobotHW::read() {
//	int rLeg[6] = { 7, 9, 11, 13, 15, 17 };
//	readMotors(rLeg, 6);
//	int lLeg[6] = { 8, 10, 12, 14, 16, 18 };
//	readMotors(lLeg, 6);
//	int rArm[3] = { 1, 3, 5 };
//	readMotors(rArm, 3);
	int many[20] = { 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
			19, 20 };
//	int many[15] = { 5 };
	return readMotors(many, 19);
//	int lArm[3] = { 2, 4, 6 };
//	readMotors(lArm, 3);
//	int head[2] = { 19, 20 };
//	readMotors(head, 2);
}

void OP2RobotHW::write(bool force_write) {
	int param[(JointData::NUMBER_OF_JOINTS - 1) * 9];
	int n = 0;
	int joint_num = 0;
	for (int id = 1; id < JointData::NUMBER_OF_JOINTS; id++) {
		// Only write if cmd has changed
		if (force_write || cmds_[id - 1] != prev_cmds_[id - 1]) {
			param[n++] = id;

			// Set PID
			param[n++] = floor(cmds_[id - 1].D);
			param[n++] = floor(cmds_[id - 1].I);
			param[n++] = floor(cmds_[id - 1].P);
			// Reserved position
			param[n++] = 0;

			// Set position
//			ROS_INFO("Desired pos in radians %f", cmds_[id - 1].pos);
			int value = MX28::Angle2Value(cmds_[id - 1].pos * 180.0 / M_PI);
			param[n++] = CM730::GetLowByte(value);
			param[n++] = CM730::GetHighByte(value);
//			ROS_INFO("Writing pos %d to motor %d", value, id);
			// Velocity goes from 0..1023. each unit is 0.114 rpm = 0.0119381 rad/sec
			// so vel in motor is vel in rad/2 / 0.0119381
			int vel = cmds_[id - 1].vel / 0.0119381;
			param[n++] = CM730::GetLowByte(vel);
			param[n++] = CM730::GetHighByte(vel);
			joint_num++;

			prev_cmds_[id - 1] = cmds_[id - 1];
		}
	}

	if (joint_num > 0) {
		ROS_INFO("Sending movement commands for %d joints", joint_num);
		int ret_val = cm730_->SyncWrite(MX28::P_D_GAIN, 9, joint_num, param);
		if (ret_val != CM730::SUCCESS) {
			ROS_ERROR("Error writing instructions: %d", ret_val);
		}
	}

}

} // namespace

op2_control::OP2RobotHW * op_robot_hw;
controller_manager::ControllerManager * cm;

void* control_loop(void * params) {
	ros::NodeHandle private_nh("~");

	double control_rate;
	private_nh.param("control_rate", control_rate, 100.0);
	ROS_INFO("Controlling at %f Hz", control_rate);

	ros::Time last_time = ros::Time::now();
	ros::Time current_time;
	ros::Rate rate(control_rate);

	while (ros::ok()) {
		rate.sleep();
		ros::Time current_time = ros::Time::now();
		ros::Duration elapsed_time = current_time - last_time;

		int ret_val = op_robot_hw->read();
		if (ret_val == 0) {
			cm->update(current_time, elapsed_time);
			op_robot_hw->write(false);
		} else
			rate.sleep(); // Wait an extra cycle if something went wrong

		//		ROS_INFO("Elapsed time: %li", elapsed_time.toNSec());
		last_time = current_time;
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "op2_robot_hw");

	ros::NodeHandle nh;

	op_robot_hw = new op2_control::OP2RobotHW();
	cm = new controller_manager::ControllerManager(op_robot_hw);

	ros::AsyncSpinner spinner(4);
	spinner.start();

//	int wakeup_motion;
//	private_nh.getParam("wakeup_motion", wakeup_motion);

	int error;
	struct sched_param param;
	pthread_t thread;
	pthread_attr_t attr;

	pthread_attr_init(&attr);

	error = pthread_attr_setschedpolicy(&attr, SCHED_RR);
	if (error != 0)
		printf("error = %d\n", error);
	error = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	if (error != 0)
		printf("error = %d\n", error);

	memset(&param, 0, sizeof(param));
	param.sched_priority = 31; // RT
	error = pthread_attr_setschedparam(&attr, &param);
	if (error != 0)
		printf("error = %d\n", error);

	// create and start the thread
	if ((error = pthread_create(&thread, &attr, &control_loop, (void*) 0)) != 0)
		exit(-1);

	ros::waitForShutdown();

}
