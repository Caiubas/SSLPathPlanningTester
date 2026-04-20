//
// Created by caiu on 19/09/25.
//

#include "RoleFreeKicker.h"
#include "../RobotController.h"
#include "../TeamInfo.h"

namespace roles {

	void RoleFreeKicker::act(RobotController& robot) {
		Point goal = robot.get_world().field.theirGoal.getMiddle();
		bool can_score = false;
		try {
			goal = robot.get_world().getGoalPosition();
			can_score = true;
		} catch (...) {}

		Robot support = Robot(-1);
		bool has_support = false;
		try {
			robot.get_m_team()->getRobotofRole(Robot::support);
			has_support = true;
		} catch (...) {}

		bool shound_wait = true;
		if (robot.get_m_team()->getEvent() == TeamInfo::runningOurFreeKick) shound_wait = false;


		if (can_score) {
			positionAndKick.act(robot, goal, true);
		} else if (has_support) {
			positionAndKick.act(robot, support, shound_wait);
		} else {
			positionAndKick.act(robot, goal, shound_wait);
		}
	}
} // roles