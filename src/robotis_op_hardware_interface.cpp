#include <robotis_op_ros_control/robotis_op_hardware_interface.h>
#include <angles/angles.h>
#include <math.h>

#include <MX28.h>
#include <JointData.h>

#include <std_msgs/Float64.h>

namespace robotis_op
{
using namespace Robot;

const std::string RobotisOPHardwareInterface::jointUIDs[JointData::NUMBER_OF_JOINTS+3] =
{
    "j_shoulder_r",
    "j_shoulder_l",
    "j_high_arm_r",
    "j_high_arm_l",
    "j_low_arm_r",
    "j_low_arm_l",
    "j_pelvis_r",
    "j_pelvis_l",
    "j_thigh1_r",
    "j_thigh1_l",
    "j_thigh2_r",
    "j_thigh2_l",
    "j_tibia_r",
    "j_tibia_l",
    "j_ankle1_r",
    "j_ankle1_l",
    "j_ankle2_r",
    "j_ankle2_l",
    "j_pan",
    "j_tilt",
    "j_wrist_r",
    "j_wrist_l",
    "j_gripper_r",
    "j_gripper_l",
};


const int RobotisOPHardwareInterface::ros_joint_offsets[JointData::NUMBER_OF_JOINTS+3] =
{
    0,        //j_shoulder_r
    0,        //j_shoulder_l
    0,        //j_high_arm_r
    0,        //j_high_arm_l
    0,        //j_low_arm_r
    0,        //j_low_arm_l
    0,        //j_pelvis_r
    0,        //j_pelvis_l
    0,        //j_thigh1_r
    0,        //j_thigh1_l
    0,        //j_thigh2_r
    0,        //j_thigh2_l
    0,        //j_tibia_r
    0,        //j_tibia_l
    0,        //j_ankle1_r
    0,        //j_ankle1_l
    0,        //j_ankle2_r
    0,        //j_ankle2_l
    0,        //j_pan
    0,        //j_tilt
    0,        //j_wrist_r
    0,        //j_wrist_l
    0,        //j_gripper_r
    0,        //j_gripper_l
};

RobotisOPHardwareInterface::Ptr RobotisOPHardwareInterface::singelton = RobotisOPHardwareInterface::Ptr();

RobotisOPHardwareInterface::RobotisOPHardwareInterface() :
    hardware_interface::RobotHW()
    , joint_state_intervall_(20.0)
    , last_joint_state_read_(ros::Time::now())
{
    ros::NodeHandle nh;

    // Initialize ROBOTIS-OP  Framework
    linux_cm730_ = new Robot::LinuxCM730("/dev/ttyUSB0");
    cm730_ = new Robot::CM730(linux_cm730_);
    if(!cm730_->Connect())
    {
        ROS_ERROR("Failed to connect CM-730\n");
    }


    /** register joints */
    // dispatching joints
    for (unsigned int id_index = 0; id_index < JointData::NUMBER_OF_JOINTS+3; id_index++)
    {
        // connect and register the joint state interface
        hardware_interface::JointStateHandle joint_state_handle(jointUIDs[id_index], &pos_[id_index], &vel_[id_index], &eff_dummy_[id_index]);
        joint_state_interface_.registerHandle(joint_state_handle);
        // connect and register the joint position interface
        hardware_interface::JointHandle joint_handle(joint_state_handle, &cmd_[id_index]);
        pos_joint_interface_.registerHandle(joint_handle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&pos_joint_interface_);

    // Set goals to current positions
    for(int id = 1; id < JointData::NUMBER_OF_JOINTS; id++){
        int value, error;
        if(cm730_->ReadWord(id, MX28::P_PRESENT_POSITION_L, &value, &error) == CM730::SUCCESS)
        {
            cmd_[id-1] = value;    
	    pos_[id-1] = value;
        }
    }



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

}

RobotisOPHardwareInterface::RobotisOPHardwareInterface(RobotisOPHardwareInterface const&)
{
}

RobotisOPHardwareInterface::~RobotisOPHardwareInterface()
{
}

RobotisOPHardwareInterface& RobotisOPHardwareInterface::operator=(RobotisOPHardwareInterface const&)
{
}

RobotisOPHardwareInterface::Ptr& RobotisOPHardwareInterface::Instance()
{
    if (!singelton)
        singelton.reset(new RobotisOPHardwareInterface());
    return singelton;
}


double lowPassFilter(double alpha, double x_new, double x_old)
{
    return alpha*x_new + (1.0-alpha)*x_old;
}

void RobotisOPHardwareInterface::readMotors(int * ids, int numMotors)
{
    cm730_->MakeBulkReadMotorData(ids, numMotors);
    cm730_->BulkRead();
    for(int id = 1; id < JointData::NUMBER_OF_JOINTS; id++){
        if (cm730_->m_BulkReadData[id].error > 0){
            ROS_ERROR("Error on read data for motor %d", id);
        } else {
		int pos = cm730_->m_BulkReadData[id].ReadWord(MX28::P_PRESENT_POSITION_L);
		pos_[id-1] = MX28::Value2Angle(pos) * (3.14159 / 180.0);   
		int vel = cm730_->m_BulkReadData[id].ReadWord(MX28::P_PRESENT_SPEED_L);
		vel_[id-1] = MX28::Value2Angle(vel) * (3.14159 / 180.0); 
		int load = cm730_->m_BulkReadData[id].ReadWord(MX28::P_PRESENT_LOAD_L);
		eff_dummy_[id-1] = load;       
	}
    }   
    

}

void RobotisOPHardwareInterface::read(ros::Time time, ros::Duration period)
{
    int rLeg[6] = {7,9,11,13,15,17};
    readMotors(rLeg, 6);
    int lLeg[6] = {8, 10,12,14,16,18};
    readMotors(lLeg, 6);
    int rArm[3] = {1,3,5};
    readMotors(rArm, 3);
    int lArm[3] = {2,4,6};
    readMotors(lArm, 3);
    int head[2] = {19,20};
    readMotors(head, 2);

    /*
for(int id = 1; id < JointData::NUMBER_OF_JOINTS; id++){
        int value, error;
        if(cm730_->ReadWord(id, MX28::P_PRESENT_POSITION_L, &value, &error) == CM730::SUCCESS)
        {
		pos_[id-1] = MX28::Value2Angle(pos) * (3.14159 / 180.0);
        }
    }
*/
    // TODO: put this back in CM730.cpp
    //IMU
    double filter_alpha = 0.5;

    //in rad/s
    imu_angular_velocity_[0] = lowPassFilter(filter_alpha,(cm730_->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L)-512)*1600.0*M_PI/(512.0*180.0),imu_angular_velocity_[0]);
    imu_angular_velocity_[1] = lowPassFilter(filter_alpha,(cm730_->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L)-512)*1600.0*M_PI/(512.0*180.0),imu_angular_velocity_[1]);
    imu_angular_velocity_[2] = lowPassFilter(filter_alpha,(cm730_->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Z_L)-512)*1600.0*M_PI/(512.0*180.0),imu_angular_velocity_[2]);

    //in m/s^2
    imu_linear_acceleration_[0] = lowPassFilter(filter_alpha,(cm730_->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_X_L)-512)*G_ACC*4.0/512.0,imu_linear_acceleration_[0]);
    imu_linear_acceleration_[1] = lowPassFilter(filter_alpha,(cm730_->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Y_L)-512)*G_ACC*4.0/512.0,imu_linear_acceleration_[1]);
    imu_linear_acceleration_[2] = lowPassFilter(filter_alpha,(cm730_->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Z_L)-512)*G_ACC*4.0/512.0,imu_linear_acceleration_[2]);

    //Estimation of roll and pitch based on accelometer data, see http://theccontinuum.com/2012/09/24/arduino-imu-pitch-roll-from-accelerometer/
    double sign = copysignf(1.0,  imu_linear_acceleration_[2]/G_ACC);
    double roll = atan2( imu_linear_acceleration_[1]/G_ACC, sign * sqrt( imu_linear_acceleration_[0]/G_ACC* imu_linear_acceleration_[0]/G_ACC +  imu_linear_acceleration_[2]/G_ACC* imu_linear_acceleration_[2]/G_ACC));
    double pitch = -atan2( imu_linear_acceleration_[0]/G_ACC, sqrt( imu_linear_acceleration_[1]/G_ACC* imu_linear_acceleration_[1]/G_ACC +  imu_linear_acceleration_[2]/G_ACC* imu_linear_acceleration_[2]/G_ACC));
    double yaw = 0.0;

    tf2::Quaternion imu_orient;
    imu_orient.setRPY(roll, pitch, yaw);

    imu_orientation_[0] = imu_orient.getX();
    imu_orientation_[1] = imu_orient.getY();
    imu_orientation_[2] = imu_orient.getZ();
}

void RobotisOPHardwareInterface::write(ros::Time time, ros::Duration period)
{
    //int param[(JointData::NUMBER_OF_JOINTS-1) * 3];
    int param[1 * 2];
    int n = 0;
    int joint_num = 0;
    // for(int id=1; id<JointData::NUMBER_OF_JOINTS; id++)
    // {
    //     param[n++] = id;
    //     param[n++] = CM730::GetLowByte(cmd_[id-1]);
    //     param[n++] = CM730::GetHighByte(cmd_[id-1]);
    //     joint_num++;
    // }
    param[n++] = 5;
    param[n++] = CM730::GetLowByte(2000);
    param[n++] = CM730::GetHighByte(2000);
    joint_num++;

    cm730_->SyncWrite(MX28::P_GOAL_POSITION_L, 3, 1, param);
    // int error;
    // cm730_->WriteWord(5, MX28::P_GOAL_POSITION_L, 2000, &error);
}


void RobotisOPHardwareInterface::setPIDGains(int id, double p_gain, double i_gain, double d_gain)
{
    MotionStatus::m_CurrentJoints.SetPGain(id, p_gain);
    MotionStatus::m_CurrentJoints.SetIGain(id, i_gain);
    MotionStatus::m_CurrentJoints.SetDGain(id, d_gain);
}


void RobotisOPHardwareInterface::setJointStateRate(double joint_state_rate)
{
    this->joint_state_intervall_ = 1.0/joint_state_rate;
}

void RobotisOPHardwareInterface::setTorqueOn(int id, bool enable)
{
    int error;
    cm730_->WriteByte(id, MX28::P_TORQUE_ENABLE, enable ? 1 : 0, &error);
}

void RobotisOPHardwareInterface::setTorqueOn(bool enable)
{
    if (enable)
        ROS_INFO("Enable torque!");
    else
        ROS_INFO("Disable torque!");

    int error;
    cm730_->WriteByte(CM730::ID_BROADCAST, MX28::P_TORQUE_ENABLE, enable, &error);
}

void RobotisOPHardwareInterface::cmdWalking(const geometry_msgs::Twist::ConstPtr& msg)
{
    double period = Walking::GetInstance()->PERIOD_TIME;
    Walking::GetInstance()->X_MOVE_AMPLITUDE=(msg->linear.x/period*1000.0*10.0);
    Walking::GetInstance()->Y_MOVE_AMPLITUDE=(msg->linear.y/period*1000.0*10.0);
    // compute the angular motion parameters to achieve the desired angular speed
    Walking::GetInstance()->A_MOVE_AMPLITUDE=(msg->angular.z/period);// in rad per sec
    // ROS_INFO("Walking: periode %f x %f y %f a %f",Walking::GetInstance()->PERIOD_TIME,Walking::GetInstance()->X_MOVE_AMPLITUDE,Walking::GetInstance()->Y_MOVE_AMPLITUDE,Walking::GetInstance()->A_MOVE_AMPLITUDE);
}


void RobotisOPHardwareInterface::enableWalking(std_msgs::BoolConstPtr enable)
{
    if(!Walking::GetInstance()->IsRunning() && enable->data)
    {
        setBlockWrite(true);
        Walking::GetInstance()->m_Joint.SetEnableLowerBody(true, true);
        Walking::GetInstance()->Start();
    }
    else if(Walking::GetInstance()->IsRunning() && !enable->data)
    {
        Walking::GetInstance()->m_Joint.SetEnableLowerBody(false, true);
        Walking::GetInstance()->Stop();
        setBlockWrite(false);
    }
}

void RobotisOPHardwareInterface::checkFall()
{
    if(Walking::GetInstance()->IsRunning() && MotionStatus::FALLEN != 0)
    {
        if(1 == MotionStatus::FALLEN)
        {
            ROS_INFO("Forward Fall! Getting up...");
            std_msgs::Int32 i;
            i.data=10;
            startAction(i);
        }
        else
        {
            ROS_INFO("Backwards Fall! Getting up...");
            std_msgs::Int32 i;
            i.data=11;
            startAction(i);
        }
    }
    if(print_check_fall_debug_info_)
        if(Walking::GetInstance()->IsRunning() || MotionStatus::FALLEN != 0)
            ROS_INFO("checkFall: walking %i fallen %i fb_acc %i rl_acc %i fb_gyro %i rl_gyro %i", Walking::GetInstance()->IsRunning(), MotionStatus::FALLEN,MotionStatus::FB_ACCEL,MotionStatus::RL_ACCEL,MotionStatus::FB_GYRO,MotionStatus::RL_GYRO);
}

void RobotisOPHardwareInterface::startAction(std_msgs::Int32 action_index)
{
    setBlockWrite(true);
    //enable MotionManager if necessary
    if(!MotionManager::GetInstance()->GetEnable())
        MotionManager::GetInstance()->SetEnable(true);
    // stop walking if necessary
    if(Walking::GetInstance()->IsRunning())
        Walking::GetInstance()->Stop();
    // stop any previous action
    if(Action::GetInstance()->IsRunning())
        Action::GetInstance()->Stop();

    // enable all the body servos to the action module
    Action::GetInstance()->m_Joint.SetEnableBody(true,true);
    if(!Action::GetInstance()->Start((int)action_index.data))
    {
        ROS_ERROR("RobotisOPHardwareInterface: Could not start Action %i",(int)action_index.data);
    }

    while(Action::GetInstance()->IsRunning())
    {
        usleep(8*1000);
    }

    setBlockWrite(false);
    Action::GetInstance()->m_Joint.SetEnableBody(false,true);
    MotionStatus::m_CurrentJoints.SetEnableBody(true,true);
}

void RobotisOPHardwareInterface::setBlockWrite(bool block)
{
    if(block)
    {
        block_write_ = true;
        //ROS_INFO("Blocked Write");
    }
    else
    {
        block_write_ = false;
        std::vector<ros::Publisher> joint_publisher;
        ros::NodeHandle n;

        std_msgs::Float64 angle_msg;

        ros::Publisher j15_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_ankle1_l_position_controller/command", 100);
        ros::Publisher j14_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_ankle1_r_position_controller/command", 100);
        ros::Publisher j17_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_ankle2_l_position_controller/command", 100);
        ros::Publisher j16_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_ankle2_r_position_controller/command", 100);
        ros::Publisher j03_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_high_arm_l_position_controller/command", 100);
        ros::Publisher j02_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_high_arm_r_position_controller/command", 100);
        ros::Publisher j05_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_low_arm_l_position_controller/command", 100);
        ros::Publisher j04_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_low_arm_r_position_controller/command", 100);
        ros::Publisher j18_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_pan_position_controller/command", 100);
        ros::Publisher j07_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_pelvis_l_position_controller/command", 100);
        ros::Publisher j06_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_pelvis_r_position_controller/command", 100);
        ros::Publisher j01_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_shoulder_l_position_controller/command", 100);
        ros::Publisher j00_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_shoulder_r_position_controller/command", 100);
        ros::Publisher j09_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_thigh1_l_position_controller/command", 100);
        ros::Publisher j08_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_thigh1_r_position_controller/command", 100);
        ros::Publisher j11_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_thigh2_l_position_controller/command", 100);
        ros::Publisher j10_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_thigh2_r_position_controller/command", 100);
        ros::Publisher j13_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_tibia_l_position_controller/command", 100);
        ros::Publisher j12_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_tibia_r_position_controller/command", 100);
        ros::Publisher j19_pub = n.advertise<std_msgs::Float64>("/robotis_op/j_tilt_position_controller/command", 100);

        joint_publisher.push_back(j00_pub);
        joint_publisher.push_back(j01_pub);
        joint_publisher.push_back(j02_pub);
        joint_publisher.push_back(j03_pub);
        joint_publisher.push_back(j04_pub);
        joint_publisher.push_back(j05_pub);
        joint_publisher.push_back(j06_pub);
        joint_publisher.push_back(j07_pub);
        joint_publisher.push_back(j08_pub);
        joint_publisher.push_back(j09_pub);
        joint_publisher.push_back(j10_pub);
        joint_publisher.push_back(j11_pub);
        joint_publisher.push_back(j12_pub);
        joint_publisher.push_back(j13_pub);
        joint_publisher.push_back(j14_pub);
        joint_publisher.push_back(j15_pub);
        joint_publisher.push_back(j16_pub);
        joint_publisher.push_back(j17_pub);
        joint_publisher.push_back(j18_pub);
        joint_publisher.push_back(j19_pub);

        for(unsigned int i = 0; i<joint_publisher.size(); ++i)
        {

            angle_msg.data = pos_[i];
            joint_publisher[i].publish(angle_msg);

        }
        //ROS_INFO("Unblocked Write");
    }
}
}
