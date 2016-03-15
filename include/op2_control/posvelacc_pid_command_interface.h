///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Martin Llofriu

#ifndef HARDWARE_INTERFACE_POSVELACC_PID_COMMAND_INTERFACE_H
#define HARDWARE_INTERFACE_POSVELACC_PID_COMMAND_INTERFACE_H

#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/posvelacc_command_interface.h>

#include <LinuxDARwIn.h>

namespace hardware_interface {

/** \brief A handle used to read and command a single joint. */
class PosVelAccPIDJointHandle: public PosVelAccJointHandle {
public:
	PosVelAccPIDJointHandle() :
			PosVelAccJointHandle(), cmd_p_(0), cmd_i_(0), cmd_d_(0), m_manager_(
					0), action_(0), walking_(0) {
	}

	/**
	 * \param js This joint's state handle
	 * \param cmd_pos A pointer to the storage for this joint's output command position
	 * \param cmd_vel A pointer to the storage for this joint's output command velocity
	 * \param eff_cmd A pointer to the storage for this joint's output command acceleration
	 */
	PosVelAccPIDJointHandle(const JointStateHandle& js, double* cmd_pos,
			double* cmd_vel, double* cmd_acc, double* cmd_p, double* cmd_i,
			double* cmd_d, Robot::MotionManager * m_manager,
			Robot::Action * action, Robot::Walking * walking) :
			PosVelAccJointHandle(js, cmd_pos, cmd_vel, cmd_acc), cmd_p_(cmd_p), cmd_i_(
					cmd_i), cmd_d_(cmd_d), m_manager_(m_manager), action_(
					action), walking_(walking) {
		if (!cmd_p || !cmd_i || !cmd_d) {
			throw HardwareInterfaceException(
					"Cannot create handle '" + js.getName()
							+ "'. One of the PID constants commands data pointers is null.");
		}
	}

	void setCommand(double cmd_pos, double cmd_vel, double cmd_acc,
			double cmd_p, double cmd_i, double cmd_d) {
		setCommandPosition(cmd_pos);
		setCommandVelocity(cmd_vel);
		setCommandAcceleration(cmd_acc);
		setCommandP(cmd_p);
		setCommandI(cmd_i);
		setCommandD(cmd_d);
	}

	void setCommandP(double cmd_p) {
		assert(cmd_p_);
		*cmd_p_ = cmd_p;
	}
	void setCommandI(double cmd_i) {
		assert(cmd_i_);
		*cmd_i_ = cmd_i;
	}
	void setCommandD(double cmd_d) {
		assert(cmd_d_);
		*cmd_d_ = cmd_d;
	}

	double getCommandP() const {
		assert(cmd_p_);
		return *cmd_p_;
	}
	double getCommandI() const {
		assert(cmd_i_);
		return *cmd_i_;
	}
	double getCommandD() const {
		assert(cmd_d_);
		return *cmd_d_;
	}
	Robot::MotionManager * getMManager() const {
		return m_manager_;
	}
	Robot::Action * getAction() const {
		return action_;
	}
	Robot::Walking * getWalking() const {
		return walking_;
	}

private:
	double* cmd_p_;
	double* cmd_i_;
	double* cmd_d_;
	Robot::MotionManager * m_manager_;
	Robot::Action * action_;
	Robot::Walking * walking_;
};

/** \brief Hardware interface to support commanding an array of joints.
 *
 * This \ref HardwareInterface supports commanding joints by position, velocity,
 * acceleration and PID constants together in one command.
 *
 * \note Getting a joint handle through the getHandle() method \e will claim that resource.
 *
 */
class PosVelAccPIDJointInterface: public HardwareResourceManager<
		PosVelAccPIDJointHandle, ClaimResources> {
};

}

#endif /*HARDWARE_INTERFACE_POSVELACC_COMMAND_INTERFACE_H*/
