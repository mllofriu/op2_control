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

#include "op2_control/op2_robot_hw.h"

#define ARRAY_SIZE(a)                               \
  ((sizeof(a) / sizeof(*(a))) /                     \
  static_cast<size_t>(!(sizeof(a) % sizeof(*(a)))))

namespace op2_control {
using namespace Robot;
// TODO: needed?
using namespace ROBOTIS;
const std::string OP2RobotHW::jointUIDs[NUM_JOINTS + 1] = { "not_used",
		"j_shoulder_r", "j_shoulder_l", "j_high_arm_r", "j_high_arm_l",
		"j_low_arm_r", "j_low_arm_l", "j_pelvis_r", "j_pelvis_l", "j_thigh1_r",
		"j_thigh1_l", "j_thigh2_r", "j_thigh2_l", "j_tibia_r", "j_tibia_l",
		"j_ankle1_r", "j_ankle1_l", "j_ankle2_r", "j_ankle2_l", "j_pan",
		"j_tilt" };

OP2RobotHW::OP2RobotHW() :
		hardware_interface::RobotHW() {
	pkt_handler = PacketHandler::GetPacketHandler(1.0);
	port_handler = (PortHandler*) PortHandler::GetPortHandler("/dev/ttyUSB0");

	open_port();

	// Reset motors
	ROS_INFO("Reset motors power");
	int dxl_comm_result;
	dxl_comm_result = pkt_handler->Write1ByteTxOnly(port_handler, CM730::ID_CM,
			CM730::P_DXL_POWER, 0);
	ros::Duration(1).sleep();
	dxl_comm_result = pkt_handler->Write1ByteTxOnly(port_handler, CM730::ID_CM,
			CM730::P_DXL_POWER, 1);
	ros::Duration(1).sleep();
	if (dxl_comm_result != COMM_SUCCESS)
		ROS_ERROR("Could not activate MX-28 power");

	ROS_INFO("Building bulk read packet with present motors");
	int motor_count = 0;
	groupBulkRead = new GroupBulkRead(port_handler, pkt_handler);
	for (int m = 1; m <= NUM_JOINTS; m++) {
		UINT8_T dxl_error = 0;                                  // DXL error
		UINT16_T dxl_model_number;                       // DXL model number
		dxl_comm_result = COMM_TX_FAIL;
		dxl_comm_result = pkt_handler->Ping(port_handler, m, &dxl_model_number,
				&dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
			ROS_INFO("Joint %i did not respond, error: %d", m, dxl_comm_result);
		else {
			ROS_INFO("Joint %id responded, adding it to bulk read", m);
			bool dxl_addparam_result = groupBulkRead->AddParam(m,
					MX28::P_PRESENT_POSITION_L, 2);
			if (!dxl_addparam_result) {
				ROS_ERROR("[ID:%03d] grouBulkRead addparam failed", m);
			}
			motor_count++;

		}
	}
	bool all_motors_present = motor_count == NUM_JOINTS;
	if (!all_motors_present) {
		ROS_ERROR("Not all active motors are responding, quitting");
		ros::shutdown();
	}

	/**
	 * ROS control interface register
	 **/
	for (int id = 1; id <= NUM_JOINTS; id++) {
// connect and register the joint state interface
		hardware_interface::JointStateHandle joint_state_handle(jointUIDs[id],
				&states_[id].pos, &states_[id].vel, &states_[id].eff);
		joint_state_interface_.registerHandle(joint_state_handle);
		hardware_interface::PosVelAccPIDJointHandle joint_handle(
				joint_state_handle, &(cmds_[id].pos), &(cmds_[id].vel),
				&(cmds_[id].eff), &(cmds_[id].P), &(cmds_[id].I),
				&(cmds_[id].D));
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
	for (unsigned int id = 1; id <= NUM_JOINTS; id++) {
		cmds_[id].P = 0;
		prev_cmds_[id] = cmds_[id];
	}
// Force write to all motors for the first time
	this->write(true);

	// Enable torque
	UINT8_T dxl_error;
	dxl_comm_result = pkt_handler->Write1ByteTxRx(port_handler, BROADCAST_ID,
			MX28::P_TORQUE_ENABLE, 1, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS) {
		ROS_ERROR("Could not activate torque");
		ros::shutdown();
	}
	ROS_INFO("OP robot created");

//	GroupSyncWrite syncwrite(port_handler, pkt_handler, MX28::P_GOAL_POSITION_L, 2);
//	int value = MX28::Angle2Value(90);
//	UINT8_T param[2];
//	param[0] = DXL_LOBYTE(value);
//	param[1] = DXL_HIBYTE(value);
//	syncwrite.AddParam(19, param);
//	syncwrite.TxPacket();
}

void OP2RobotHW::open_port() {
	if (port_handler->OpenPort()) {
		ROS_INFO("Succeeded to open the port!");
	} else {
		ROS_ERROR("Failed to open the port!");
		ROS_ERROR("Press any key to terminate...");
	}
	if (port_handler->SetBaudRate(1000000)) {
		ROS_INFO("Succeeded to change the baudrate!");
	} else {
		ROS_ERROR("Failed to change the baudrate!");
		ROS_ERROR("Press any key to terminate...");
	}
}

void OP2RobotHW::reset_port() {
	port_handler->ClearPort();
	port_handler->ClosePort();
	open_port();
//	pkt_handler->Write1ByteTxOnly(port_handler, CM730::ID_CM,
//			CM730::P_DXL_POWER, 1);
//	UINT8_T dxl_error;
//	int dxl_comm_result = pkt_handler->Write1ByteTxRx(port_handler, BROADCAST_ID,
//			MX28::P_TORQUE_ENABLE, 1, &dxl_error);
//	if (dxl_comm_result != COMM_SUCCESS) {
//		ROS_ERROR("Could not activate torque");
//		ros::shutdown();
//	}
}

bool OP2RobotHW::read() {
	UINT8_T dxl_error = 0;                                  // DXL error
	UINT16_T dxl_model_number;                               // DXL model number

//	pkt_handler->Ping(port_handler, 200, &dxl_model_number, &dxl_error);
//	ros::Duration(0.01).sleep();

	int dxl_comm_result = groupBulkRead->TxRxPacket();
	if (dxl_comm_result != COMM_SUCCESS) {
		ROS_ERROR("Bulk read failed!");
//		ros::Duration(0.001).sleep();
//		for (int m = 0; m < NUM_JOINTS; m++) {
//			// Ping all motors
//			UINT8_T dxl_error = 0;                                  // DXL error
//			UINT16_T dxl_model_number;                       // DXL model number
//			int dxl_comm_result = pkt_handler->Ping(port_handler,
//					active_joints[m], &dxl_model_number, &dxl_error);
//			if (dxl_comm_result == COMM_SUCCESS) {
//				ROS_INFO("Ping to %d succeded", active_joints[m]);
//			} else {
//				ROS_INFO("Ping to %d failed", active_joints[m]);
//			}
//		}
		return dxl_comm_result == COMM_SUCCESS;
	}

	for (int id = 1; id <= NUM_JOINTS; id++) {
		UINT16_T present_pos;
		bool dxl_getdata_result = groupBulkRead->GetData(id,
				MX28::P_PRESENT_POSITION_L, &present_pos);
		if (!dxl_getdata_result) {
			ROS_ERROR("[ID:%03d] getdata failed", id);
			return COMM_RX_TIMEOUT;
		}
		states_[id - 1].pos = MX28::Value2Angle(present_pos) * (M_PI / 180.0);
//		ROS_INFO("Successfully read joint %d, position %f", ids[id],
//				states_[ids[id] - 1].pos);

	}

//	int ret_val = 0;
//	bool all_fail = true;
//	for (int id = 0; id < NUM_JOINTS; id++) {
//		UINT16_T data;
//		UINT8_T error;
//		int read_return = pkt_handler->Read2ByteTxRx(port_handler,
//				id, MX28::P_PRESENT_POSITION_L, &data, &error);
//		if (read_return == COMM_SUCCESS) {
//			states_[id - 1].pos = MX28::Value2Angle(data)
//					* (M_PI / 180.0);
////			ROS_INFO("Successfully read joint %d, position %f", id, states_[id - 1].pos);
//			all_fail = false;
//		} else {
//			ROS_ERROR(
//					"Error reading position for joint %d, read return error: %d",
//					id, read_return);
//
//		}
//		ros::Duration(0.0005).sleep();
//	}

//	return !all_fail;
	return dxl_comm_result == COMM_SUCCESS;
}

void OP2RobotHW::write(bool force_write) {
	UINT16_T dataLength = 8;
	GroupSyncWrite syncwrite(port_handler, pkt_handler, MX28::P_D_GAIN,
			dataLength);

	int num_joints = 0;
	for (unsigned int id = 1; id <= NUM_JOINTS; id++) {
		if (force_write || cmds_[id] != prev_cmds_[id]) {
//		if (true) {
			UINT8_T param[dataLength];
			int n = 0;
			param[n++] = floor(cmds_[id].D);
			param[n++] = floor(cmds_[id].I);
			param[n++] = floor(cmds_[id].P);
			ROS_INFO("Desired P %d for motor %u", param[n - 1], id);
			// Reserved position
			param[n++] = 0;

			// Set position
			ROS_INFO("Desired pos in radians %f for motor %u", cmds_[id].pos,
					id);
			int value = MX28::Angle2Value(cmds_[id].pos * 180.0 / M_PI);
			param[n++] = DXL_LOBYTE(value);
			param[n++] = DXL_HIBYTE(value);
//			ROS_INFO("Writing pos %d to motor %d", value, id);
			// Velocity goes from 0..1023. each unit is 0.114 rpm = 0.0119381 rad/sec
			// so vel in motor is vel in rad/2 / 0.0119381
			int vel = cmds_[id].vel / 0.0119381;
			param[n++] = DXL_LOBYTE(vel);
			param[n++] = DXL_HIBYTE(vel);
			bool res = syncwrite.AddParam(id, param);
			if (!res)
				ROS_ERROR("Could not add param");
			num_joints++;
			cmds_[id] = prev_cmds_[id];
		}
	}

	if (num_joints > 0) {
		int dxl_comm_result = syncwrite.TxPacket();
		if (dxl_comm_result != COMM_SUCCESS)
			ROS_ERROR("Write failure with error %d", dxl_comm_result);
		else
			ROS_INFO("Sent commands to %d joints", num_joints);
	}

//	int param[(JointData::NUMBER_OF_JOINTS - 1) * 9];
//	int n = 0;
//	int joint_num = 0;
//	for (int id = 1; id < JointData::NUMBER_OF_JOINTS; id++) {
//		// Only write if cmd has changed
//		if (force_write || cmds_[id - 1] != prev_cmds_[id - 1]) {
//			param[n++] = id;
//
//			// Set PID
//			param[n++] = floor(cmds_[id - 1].D);
//			param[n++] = floor(cmds_[id - 1].I);
//			param[n++] = floor(cmds_[id - 1].P);
//			// Reserved position
//			param[n++] = 0;
//
//			// Set position
////			ROS_INFO("Desired pos in radians %f", cmds_[id - 1].pos);
//			int value = MX28::Angle2Value(cmds_[id - 1].pos * 180.0 / M_PI);
//			param[n++] = CM730::GetLowByte(value);
//			param[n++] = CM730::GetHighByte(value);
////			ROS_INFO("Writing pos %d to motor %d", value, id);
//			// Velocity goes from 0..1023. each unit is 0.114 rpm = 0.0119381 rad/sec
//			// so vel in motor is vel in rad/2 / 0.0119381
//			int vel = cmds_[id - 1].vel / 0.0119381;
//			param[n++] = CM730::GetLowByte(vel);
//			param[n++] = CM730::GetHighByte(vel);
//			joint_num++;
//
//			prev_cmds_[id - 1] = cmds_[id - 1];
//		}
//	}
//
//	if (joint_num > 0) {
//		ROS_INFO("Sending movement commands for %d joints", joint_num);
//		int ret_val = cm730_->SyncWrite(MX28::P_D_GAIN, 9, joint_num, param);
//		if (ret_val != CM730::SUCCESS) {
//			ROS_ERROR("Error writing instructions: %d", ret_val);
//		}
//	}

}

} // namespace

op2_control::OP2RobotHW * op_robot_hw;
controller_manager::ControllerManager * cm;
bool shutdown_thread = false;
void* control_loop(void * params) {
	ros::NodeHandle private_nh("~");

	double control_rate;
	private_nh.param("control_rate", control_rate, 100.0);
	ROS_INFO("Controlling at %f Hz", control_rate);

	ros::Time last_time = ros::Time::now();
	ros::Time current_time;
	ros::Rate rate(control_rate);
	int to_skip = 0;
	while (!shutdown_thread) {
		rate.sleep();
		ros::Time current_time = ros::Time::now();
		ros::Duration elapsed_time = current_time - last_time;

		if (to_skip == 0) {
			if (op_robot_hw->read()) {
				cm->update(current_time, elapsed_time);
				op_robot_hw->write(false);
			} else {
				// All failed, skip a bunch of cycles
				to_skip = 5; // skip next 5 cycles
				op_robot_hw->reset_port();
			}
		} else {
			to_skip--;

		}

		ROS_INFO("Elapsed time: %li", elapsed_time.toNSec());
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

	shutdown_thread = true;
	int val;
	pthread_join(thread, NULL);

}
