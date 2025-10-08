//
// Created by caiu on 25/08/25.
//

#include "RoleFieldTrajectory.h"
#include "../RobotController.h"

namespace roles {
	void RoleSquaredTrajectory::act(RobotController& robot){
		if (size(robot.mCurrent_trajectory) == 0) {
			robot.mCurrent_trajectory = default_trajectory;
		}
		followTrajectory.act(robot, robot.mCurrent_trajectory);
    }

} // roles