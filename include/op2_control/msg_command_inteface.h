/// \author Martin Llofriu

#ifndef HARDWARE_INTERFACE_MSG_COMMAND_INTERFACE_H
#define HARDWARE_INTERFACE_MSG_COMMAND_INTERFACE_H

#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace hardware_interface
{

/** \brief A handle used to command a single joint using variables from a template message M. */
template <class M>
class JointMsgCmdHandle
{
public:
	JointMsgCmdHandle() {}

	JointMsgCmdHandle(const std::string& name, M * msg) : name_(name) {cmd_ = msg;}

	void setCommand(M & command) {*cmd_ = command;}
	M getCommand() const {return *cmd_;}

	std::string getName() const {return name_;}	
private:
	M * cmd_;
	std::string name_;
};

/** \brief Hardware interface to support commanding an array of joints.
 *
 * This \ref HardwareInterface supports commanding the output of an array of
 * named joints. Note that these commands can have any semantic meaning as long
 * as they each can be represented by a message.
 *
 * \note Getting a joint handle through the getHandle() method \e will claim that resource.
 *
 */
template <class M>
class JointMsgCmdInterface : public HardwareResourceManager<hardware_interface::JointMsgCmdHandle<M>, ClaimResources> {};

} // namespace

#endif