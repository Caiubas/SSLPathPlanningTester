//
// Created by caiu on 25/08/25.
//

#include "RoleSquaredTrajectory.h"
#include "../RobotController.h"

namespace roles {
	void RoleSquaredTrajectory::act(RobotController& robot){
		if (size(robot.get_m_current_trajectory()) == 0) {
			robot.get_m_current_trajectory() = default_trajectory;
		}
		followTrajectory.act(robot, robot.get_m_current_trajectory());
    }

} // roles