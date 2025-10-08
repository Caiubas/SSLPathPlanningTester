//
// Created by caiu on 25/08/25.
//

#include "TacticFollowTrajectory.h"

#include <iostream>

#include "../RobotController.h"
#include "../TeamInfo.h"

namespace tactics {
	void TacticFollowTrajectory::act(RobotController& robot, std::vector<Point> trajectory) {

		int i = 0;
		while (size(trajectory) > 0) {
			double distance = robot.getPosition().getDistanceTo(trajectory[0]);
			if (size(trajectory) == 1 && distance < robot.get_m_static_position_tolarance()) {
				trajectory.erase(trajectory.begin());
				break;
			}
			std::cout << "next point: " << trajectory[0].getX() << " " << trajectory[0].getY() << std::endl;

			if (distance < robot.get_m_dynamic_position_tolarance()) {
				trajectory.erase(trajectory.begin());
				continue;
			}
			break;
		}
		if (size(trajectory) == 0) {
			robot.set_mtarget_vel({0, 0});
			robot.setPositioned(true);
			robot.get_m_team()->setPositioned(robot.getId(), true);
			return;
		}

		moveTo.act(robot, trajectory[0], false); //TODO mudar avoid ball

		robot.setPositioned(false);
		robot.get_m_team()->setPositioned(robot.getId(), false);
	}
	void TacticFollowTrajectory::act(RobotController& robot) {
		std::cout << "this is a dummy method" << std::endl;
	}
} // tactics