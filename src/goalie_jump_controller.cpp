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

#include <vector>
#include <map>
#include <string>

#include <ros/node_handle.h>
#include <ros/console.h>
#include <controller_interface/controller.h>

#include <position_controllers/joint_position_controller.h>
#include <pluginlib/class_list_macros.h>

#include "op2_control/posvelacc_pid_command_interface.h"

namespace op2_control
{

/**
 * \brief Control all joints to perform a goalie jumb
 *
 * \section ROS interface
 *
 * \param joints Names of the joints to control.
 */
class GoalieJumpController : public controller_interface::Controller<hardware_interface::PosVelAccPIDJointInterface>
{
public:
  GoalieJumpController() {}
  ~GoalieJumpController() {}

  float limitknees;
  float pi=3.1415962;
  float dtr=pi/180;
  float h=-65;
  float k=128;
  float a=25;
  float Tetha_Hip=-65*dtr;
  float Tetha_Knee=128*dtr;
  float Tetha_Ankle=(66)*dtr;
  int count=0;


  bool init(hardware_interface::PosVelAccPIDJointInterface * hw, ros::NodeHandle &n)
  {
    // List of controlled joints
    std::string param_name = "joints";
    if(!n.getParam(param_name, joint_names_))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
      return false;
    }
    n_joints_ = joint_names_.size();

    if(n_joints_ == 0){
      ROS_ERROR_STREAM("List of joint names is empty.");
      return false;
    }

    for(unsigned int i=0; i<n_joints_; i++)
    {
      try
      {
        joints_[joint_names_[i]] = hw->getHandle(joint_names_[i]);
      }
      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }
    }


    param_name = "limitknees";
    if(!n.getParam(param_name, limitknees))
    {
      ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
      return false;
    }

    return true;
  }



void ZeroP(void)
  {
    // Semantic zero
    ROS_INFO("JuanProgamo 10 y leyo %f", limitknees);
    ROS_INFO("Crouching joint");
    joints_["j_ankle1_l"].setCommandP(62.0);
    joints_["j_ankle1_r"].setCommandP(63.0);
    joints_["j_ankle2_l"].setCommandP(62.0);
    joints_["j_ankle2_r"].setCommandP(62.0);
    joints_["j_tibia_l"].setCommandP(62);
    joints_["j_tibia_r"].setCommandP(62.0);
    joints_["j_thigh1_l"].setCommandP(62.0);
 	joints_["j_thigh1_r"].setCommandP(62.0);
 	joints_["j_thigh2_l"].setCommandP(62.0);
	joints_["j_thigh2_r"].setCommandP(62.0);
	joints_["j_pelvis_l"].setCommandP(62.0);
	joints_["j_pelvis_r"].setCommandP(62.0);

	joints_["j_ankle1_l"].setCommandVelocity(0.5);
    joints_["j_ankle1_r"].setCommandVelocity(0.5);
    joints_["j_ankle2_l"].setCommandVelocity(0.5);
    joints_["j_ankle2_r"].setCommandVelocity(0.5);
    joints_["j_tibia_l"].setCommandVelocity(1.0);
    joints_["j_tibia_r"].setCommandVelocity(1.0);
    joints_["j_thigh1_l"].setCommandVelocity(0.5);
    joints_["j_thigh1_r"].setCommandVelocity(0.5);
    joints_["j_thigh2_l"].setCommandVelocity(0.5);
    joints_["j_thigh2_r"].setCommandVelocity(0.5);
    joints_["j_pelvis_l"].setCommandVelocity(0.5);
    joints_["j_pelvis_r"].setCommandVelocity(0.5);

	joints_["j_ankle1_l"].setCommandPosition(0);
    joints_["j_ankle1_r"].setCommandPosition(0);
    joints_["j_ankle2_l"].setCommandPosition(0);
    joints_["j_ankle2_r"].setCommandPosition(0);
    joints_["j_tibia_l"].setCommandPosition(0);
    joints_["j_tibia_r"].setCommandPosition(0);
	joints_["j_thigh1_l"].setCommandPosition(0);
	joints_["j_thigh1_r"].setCommandPosition(0);
	joints_["j_thigh2_l"].setCommandPosition(0);
	joints_["j_thigh2_r"].setCommandPosition(0);
	joints_["j_pelvis_l"].setCommandPosition(0);
	joints_["j_pelvis_r"].setCommandPosition(0);

//    joints_["j_ankle1_l"].setCommandVelocity(1.0);
//    joints_["j_ankle1_l"].setCommandPosition(0);




  }


  void starting(const ros::Time& time)
  {
    // Semantic zero
    for (std::map<std::string, hardware_interface::PosVelAccPIDJointHandle>::iterator it = joints_.begin(); it!=joints_.end(); ++it){
      it->second.setCommandPosition(it->second.getPosition());
      it->second.setCommandP(3.0);
    }
    //if (count>300)
    //	{
    		state_ = crouching;
   // 		count=0;
   // 	}
   // else
   // 	{
   // 		count++;
   // 		ROS_INFO("\n \n \n ESPERANDP 10 SEGUNDOS \n \n \n");
   // 	}
    ROS_INFO("JuanProgamo 10 y leyo %f", limitknees);
    ROS_INFO("Crouching joint");
    ZeroP();


  }

  // "j_shoulder_r","j_shoulder_l","j_high_arm_r","j_high_arm_l","j_low_arm_r","j_low_arm_l","j_pelvis_r",
  // "j_pelvis_l","j_thigh1_r","j_thigh1_l","j_thigh2_r","j_thigh2_l","j_tibia_r","j_tibia_l","j_ankle1_r",
  // "j_ankle1_l","j_ankle2_r","j_ankle2_l","j_pan","j_tilt","j_wrist_r","j_wrist_l","j_gripper_r","j_gripper_l"]
  std::string r_elbow = "j_low_arm_r";

  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/)
  {
  	float d=0.85;
//	ROS_INFO("Elbow pos: %f", joints_[r_elbow].getPosition());
    switch (state_){
    case crouching:
      if ((joints_["j_tibia_r"].getPosition() < 0.1) && (count>800))
      {

      	joints_["j_thigh2_l"].setCommandPosition(-Tetha_Hip*d);
		joints_["j_thigh2_r"].setCommandPosition(Tetha_Hip*d);

		joints_["j_tibia_r"].setCommandPosition(Tetha_Knee*d);
		joints_["j_tibia_l"].setCommandPosition(-Tetha_Knee*d);

		joints_["j_ankle1_l"].setCommandPosition((-Tetha_Ankle)*d);
    	joints_["j_ankle1_r"].setCommandPosition((Tetha_Ankle)*d);

        state_ = extending;
        ROS_INFO("Extending joint");
      }
      else {
      		count++;
      		ROS_INFO("ESPERANDO");
      }
      break;

    case extending:
      if (joints_["j_tibia_r"].getPosition() > (Tetha_Knee*d-(3*dtr))){
		float d=0.2;

		joints_["j_ankle2_l"].setCommandVelocity((6));
    	joints_["j_ankle2_r"].setCommandVelocity((0));

		//joints_["j_thigh2_r"].setCommandPosition(Tetha_Hip*d);
		//joints_["j_tibia_r"].setCommandPosition(Tetha_Knee*d);
    	//joints_["j_ankle1_r"].setCommandPosition((Tetha_Ankle)*d);



		joints_["j_ankle2_l"].setCommandPosition((45)*dtr);
    	joints_["j_ankle2_r"].setCommandPosition((45)*dtr);



		joints_["j_thigh2_r"].setCommandVelocity((4));
		joints_["j_tibia_r"].setCommandVelocity((0));
		joints_["j_ankle1_r"].setCommandVelocity((4));


		//joints_["j_thigh2_r"].setCommandPosition(Tetha_Hip*d);
		//joints_["j_tibia_r"].setCommandPosition(Tetha_Knee*d);
    	//joints_["j_ankle1_r"].setCommandPosition((Tetha_Ankle)*d);




        state_ = holding;
        ROS_INFO("Holding joint");
      }
      break;

	case holding:

      if (joints_["j_ankle2_l"].getPosition() > (30*dtr)){
//      	joints_["j_thigh2_l"].setCommandPosition(-Tetha_Hip*d);
		float d=0.0;


		joints_["j_thigh2_l"].setCommandVelocity((4));
		joints_["j_tibia_l"].setCommandVelocity((0));
		joints_["j_ankle1_l"].setCommandVelocity((4));


		joints_["j_thigh2_l"].setCommandPosition(Tetha_Hip*d);
		joints_["j_tibia_l"].setCommandPosition(Tetha_Knee*d);
    	joints_["j_ankle1_l"].setCommandPosition((Tetha_Ankle)*d);
		//joints_["j_thigh2_r"].setCommandPosition(Tetha_Hip*d);

		//joints_["j_tibia_r"].setCommandPosition(Tetha_Knee*d);
//		joints_["j_tibia_l"].setCommandPosition(-Tetha_Knee*d);

//		joints_["j_ankle1_l"].setCommandPosition((-Tetha_Ankle)*d);
    	//joints_["j_ankle1_r"].setCommandPosition((Tetha_Ankle)*d);

        state_ = relaxing;
        ROS_INFO("relaxing joint");
      }


    case relaxing:
      break;
    }


  }

  std::vector< std::string > joint_names_;
  std::map<std::string, hardware_interface::PosVelAccPIDJointHandle> joints_;
  // std::vector< hardware_interface::PosVelAccPIDJointHandle > joints_;
  unsigned int n_joints_;

  enum State { extending, crouching, holding ,relaxing};
  State state_;


};

}

PLUGINLIB_EXPORT_CLASS(op2_control::GoalieJumpController,controller_interface::ControllerBase)
