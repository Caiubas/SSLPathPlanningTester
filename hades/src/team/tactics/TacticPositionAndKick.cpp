//
// Created by caiu on 25/08/25.
//

#include "TacticPositionAndKick.h"

#include <iostream>

#include "../RobotController.h"
#include "../TeamInfo.h"

namespace tactics {
	void TacticPositionAndKick::act(RobotController& robot, Point goal, bool wait) {
		Point kick_pos = robot.get_world().getKickingPosition(robot.get_world().ball.getPosition(), goal, robot.get_m_ball_avoidance_radius() + robot.getRadius());
		//std::cout << robot.is_double_touch() << (robot.get_m_team()->getEvent() == TeamInfo::stop) << wait << !robot.isPositioned() << !robot.isOriented() << (!(robot.getRole() == Robot::placer && robot.get_m_team()->getEvent() == TeamInfo::ourballPlacement) && !robot.get_world().isBallReachable(robot.getRole() != Robot::goal_keeper)) << (robot.getPosition().getDistanceTo(robot.get_world().ball.getPosition()) > distance_to_kick + robot.getRadius()) << std::endl;

		if (robot.is_double_touch() or robot.get_m_team()->getEvent() == TeamInfo::stop or wait or !robot.isPositioned() or !robot.isOriented() or (!(robot.getRole() == Robot::placer && robot.get_m_team()->getEvent() == TeamInfo::ourballPlacement) && !robot.get_world().isBallReachable(robot.getRole() != Robot::goal_keeper)) or robot.getPosition().getDistanceTo(robot.get_world().ball.getPosition()) > distance_to_kick + robot.getRadius()) {
			moveTo.act(robot, kick_pos, true);
			turnTo.act(robot, goal);
		} else {
			kick.act(robot);
		}
	}

	void TacticPositionAndKick::act(RobotController &robot, Robot sup, bool wait) {
		Point kick_pos = robot.get_world().getKickingPosition(robot.get_world().ball.getPosition(), sup.getPosition(), robot.get_m_ball_avoidance_radius() + robot.getRadius());
		//ISSO AQUI ESTA HORROROSO
		//std::cout << robot.is_double_touch() << (robot.get_m_team()->getEvent() == TeamInfo::stop) << wait << !robot.isPositioned() << !robot.isOriented() << !sup.isPositioned() << (!(robot.getRole() == Robot::placer && robot.get_m_team()->getEvent() == TeamInfo::ourballPlacement) && !robot.get_world().isBallReachable(robot.getRole() != Robot::goal_keeper)) << (robot.getPosition().getDistanceTo(robot.get_world().ball.getPosition()) > distance_to_kick + robot.getRadius()) << std::endl;

		if (robot.is_double_touch() or robot.get_m_team()->getEvent() == TeamInfo::stop or wait or !robot.isPositioned() or !robot.isOriented() or !sup.isPositioned() or (!(robot.getRole() == Robot::placer && robot.get_m_team()->getEvent() == TeamInfo::ourballPlacement) && !robot.get_world().isBallReachable(robot.getRole() != Robot::goal_keeper)) or robot.getPosition().getDistanceTo(robot.get_world().ball.getPosition()) > distance_to_kick + robot.getRadius()) {
			moveTo.act(robot, kick_pos, true);
			turnTo.act(robot, sup.getPosition());
		} else {
			kick.act(robot);
		}
	}


	void TacticPositionAndKick::act(RobotController& robot) {
          std::cout << "this is a dummy method" << std::endl;
	}

} // tactics