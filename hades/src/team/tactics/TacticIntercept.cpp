//
// Created by caiu on 26/08/25.
//

#include "TacticIntercept.h"

#include <iostream>
#include <ostream>

#include "../RobotController.h"

namespace tactics {
	void TacticIntercept::act(RobotController& robot) {
    	if (robot.get_world().ball.isStopped()) moveTo.act(robot, robot.get_world().ball.getPosition(), true);

        if (robot.get_world().ball.isMoving()) {
          	LineSegment line = robot.get_world().ball.getMovementLine();
        	if (robot.getPosition().getDistanceTo(robot.get_world().ball.getPosition()) < distanceThreshold && line.isPointAligned(robot.getPosition(), angle_tolerance)) {
        		cushion.act(robot);
        	} else if (line.isPointAligned(robot.getPosition(), angle_tolerance) && robot.get_world().isBallMovingIdDirection(robot.getId(), angle_tolerance)) {
        		stop.act(robot);
        		turnTo.act(robot, robot.get_world().ball.getPosition());
        	}
        	else moveTo.act(robot, robot.get_world().ball.getStopPosition(), false);
        }
		turnTo.act(robot, robot.get_world().ball.getPosition());
	}
} // tactics