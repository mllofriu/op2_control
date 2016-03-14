#include <position_controllers/joint_position_controller.h>
#include <pluginlib/class_list_macros.h>

// #include "op2_control/forward_msg_cmd_controller.h"
#include "op2_control/PIDPosVelAcc.h"

// namespace op2_control
// {

// typedef forward_command_controller::ForwardMsgCmdController<hardware_interface::JointMsgCmdHandle<op2_control::PIDPosVelAcc> >
//         DarwinJointController;

// } // namespace

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <dynamic_reconfigure/server.h>

#include "op2_control/msg_command_inteface.h"

#include <boost/shared_ptr.hpp>

namespace op2_control
{

/**
 * \brief Single joint controller.
 *
 * This class passes the command msg down to the joint.
 * Command signal and joint hardware interface are of the same type, e.g. effort commands for an effort-controlled
 * joint.
 *
 * \tparam M Type implementing the msg data type
 *
 * \section ROS interface
 *
 * \param type hardware interface type.
 * \param joint Name of the joint to control.
 *
 * Subscribes to:
 * - \b command (M) : The joint command to apply.
 */
class DarwinJointController : public controller_interface::Controller<hardware_interface::JointMsgCmdInterface<PIDPosVelAcc> >
{
public:
  DarwinJointController() {}
  ~DarwinJointController() {sub_command_.shutdown();}

  bool init(hardware_interface::JointMsgCmdInterface<PIDPosVelAcc> * hw, ros::NodeHandle &n)
  {
    std::string joint_name;
    if (!n.getParam("joint", joint_name))
    {
      ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
      return false;
    }
    joint_ = hw->getHandle(joint_name);
    sub_command_ = n.subscribe<PIDPosVelAcc>("commandMsg", 1, &DarwinJointController::commandCB, this);
    return true;
  }

  // TODO: set to current position
  void starting(const ros::Time& time){ }
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {joint_.setCommand(*command_buffer_.readFromRT());}

  hardware_interface::JointMsgCmdHandle<PIDPosVelAcc> joint_;
  realtime_tools::RealtimeBuffer<PIDPosVelAcc> command_buffer_;

private:
  ros::Subscriber sub_command_;
  void commandCB(const PIDPosVelAccConstPtr & msg) {command_buffer_.writeFromNonRT(*msg);}
};


} // namespace


PLUGINLIB_EXPORT_CLASS(op2_control::DarwinJointController,controller_interface::ControllerBase)
