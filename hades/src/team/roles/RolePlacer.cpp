//
// Created by caiu on 23/09/25.
//

#include "RolePlacer.h"
#include "../RobotController.h"
#include "../TeamInfo.h"

namespace roles {
	void RolePlacer::act(RobotController& robot) {
		try {
			positionAndKick.act(robot, robot.get_m_team()->getRobotofRole(Robot::placeHolder));
		} catch (...) {
			positionAndKick.act(robot, robot.get_m_team()->getBallPlacementSpot());
		}
	}
} // roles