#ifndef FORWARD_MSG_CMD_CONTROLLER_FORWARD_COMMAND_CONTROLLER_H
#define FORWARD_MSG_CMD_CONTROLLER_FORWARD_COMMAND_CONTROLLER_H

#include <boost/shared_ptr.hpp>

#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <std_msgs/Float64.h>
#include <realtime_tools/realtime_buffer.h>

#include "op2_control/msg_command_inteface.h"



namespace forward_command_controller
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
template <class M>
class ForwardMsgCmdController : public controller_interface::Controller<hardware_interface::JointMsgCmdInterface<M> >
{
public:
  ForwardMsgCmdController() {}
  ~ForwardMsgCmdController() {sub_command_.shutdown();}

  bool init(hardware_interface::JointMsgCmdInterface<M> * hw, ros::NodeHandle &n)
  {
    std::string joint_name;
    if (!n.getParam("joint", joint_name))
    {
      ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
      return false;
    }
    joint_ = hw->getHandle(joint_name);
    sub_command_ = n.subscribe<M>("command", 1, ForwardMsgCmdController<M>::commandCB, this);
    return true;
  }

  void starting(const ros::Time& time);
  void update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {joint_.setCommand(*command_buffer_.readFromRT());}

  hardware_interface::JointMsgCmdHandle<M> joint_;
  realtime_tools::RealtimeBuffer<M> command_buffer_;

private:
  ros::Subscriber sub_command_;
  void commandCB(const boost::shared_ptr<M const>& msg) {command_buffer_.writeFromNonRT(msg->data);}
};


} // namespace

#endif