//
// Created by caiu on 25/08/25.
//

#include "RoleCircularTrajectory.h"
#include "../RobotController.h"

namespace roles {
	void RoleCircularTrajectory::act(RobotController& robot) {
		if (size(robot.mCurrent_trajectory) == 0) {
			for (int i = 0; i < resolution; i++) {
				robot.mCurrent_trajectory.emplace_back(radius*sin(2*i*M_PI/resolution + robot.getId()*2*M_PI/size(robot.mWorld.allies)), radius*cos(2*i*M_PI/resolution + robot.getId()*2*M_PI/size(robot.mWorld.allies)));
			}
		}
		followTrajectory.act(robot, robot.mCurrent_trajectory);
    }
}