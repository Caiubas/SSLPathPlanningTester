//
// Created by caiu on 23/09/25.
//

#include "RolePlaceHolder.h"
#include "../RobotController.h"
#include "../TeamInfo.h"

namespace roles {
	void RolePlaceHolder::act(RobotController& robot) {
		LineSegment line{robot.mWorld.ball.getPosition(), robot.mTeam->ball_placement_spot};
		line = line.getResized(line.getLength() + robot.getRadius());
		Point p = line.getEnd();
		moveTo.act(robot, p, false);
		turnTo.act(robot, robot.mWorld.ball.getPosition());
    }
} // roles