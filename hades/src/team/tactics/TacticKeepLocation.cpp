//
// Created by caiu on 25/08/25.
//

#include "TacticKeepLocation.h"
#include "../RobotController.h"
#include <iostream>

namespace tactics {
	void TacticKeepLocation::act(RobotController& robot){
        std::cout << "this is a dummy method" << std::endl;
    }

	void TacticKeepLocation::act(RobotController& robot, Point keep){
		if (robot.get_world().ball.isStopped() || !robot.get_world().isBallMovingIdDirection(robot.getId())) {
			moveTo.act(robot, keep, true);
		}
		else {
			if (robot.get_world().ball.isMoving()) {
				LineSegment line = {robot.get_world().ball.getPosition(), robot.get_world().ball.getVelocity()};
				if (robot.getPosition().getDistanceTo(robot.get_world().ball.getPosition()) < distanceThreshold && line.isPointAligned(robot.getPosition(), angle_tolerance)) {
					cushion.act(robot);
				} else moveTo.act(robot, robot.get_world().ball.getStopPosition(), false);
			}
		}
		turnTo.act(robot, robot.get_world().ball.getPosition());
	}
} // tactics