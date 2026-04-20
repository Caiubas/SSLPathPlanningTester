//
// Created by caiu on 19/09/25.
//

#include "RolePenaltier.h"
#include "../RobotController.h"
#include "../TeamInfo.h"

namespace roles {
	void RolePenaltier::act(RobotController& robot) {
		Point goal = robot.get_world().getGoalPosition();
		if (robot.get_m_team()->getEvent() == TeamInfo::ourPenalty) {
			positionAndKick.act(robot, goal, true);
		}
		if (robot.get_m_team()->getEvent() == TeamInfo::runningOurPenalty) {
			positionAndKick.act(robot, goal, false);
		}
	}
} // roles