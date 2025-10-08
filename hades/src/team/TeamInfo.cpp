//
// Created by caiu on 09/04/25.
//

#include "TeamInfo.h"

Robot TeamInfo::getRobotofRole(enum Robot::role role) {
	for (Robot robot : robots) {
		if (robot.getRole() == role) return robot;
	}
	throw std::runtime_error("No robot of desired role");
}

Robot TeamInfo::getEnemyofRole(enum Robot::role role, std::array<Robot, 16> enemies) {
	for (Robot robot : enemies) {
		if (enemy_roles[robot.getId()] == role) return robot;
	}
	throw std::runtime_error("No robot of desired role");
}

Robot TeamInfo::getRobotToKickTo(RobotController robot) {
	for (Robot r : robots) {
		if (r.getId() != robot.getId() && r.isDetected() && r.getRole() != Robot::goal_keeper) return robot;
	}
	throw std::runtime_error("No robot to kick to");
}
