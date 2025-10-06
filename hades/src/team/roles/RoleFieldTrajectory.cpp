//
// Created by caiu on 25/08/25.
//

#include "RoleSquaredTrajectory.h"
#include "../RobotController.h"

namespace roles {
	void RoleSquaredTrajectory::act(RobotController& robot){
		if (size(robot.mCurrent_trajectory) == 0) {
			robot.mCurrent_trajectory.push_back({robot.mWorld.field.inside_dimensions.getMajorPoint().getX(), robot.mWorld.field.inside_dimensions.getMajorPoint().getY()});
			robot.mCurrent_trajectory.push_back({robot.mWorld.field.inside_dimensions.getMajorPoint().getX(), robot.mWorld.field.inside_dimensions.getMinorPoint().getY()});
			robot.mCurrent_trajectory.push_back({robot.mWorld.field.inside_dimensions.getMinorPoint().getX(), robot.mWorld.field.inside_dimensions.getMinorPoint().getY()});
			robot.mCurrent_trajectory.push_back({robot.mWorld.field.inside_dimensions.getMinorPoint().getX(), robot.mWorld.field.inside_dimensions.getMajorPoint().getY()});
		}
		followTrajectory.act(robot, robot.mCurrent_trajectory);
    }

} // roles