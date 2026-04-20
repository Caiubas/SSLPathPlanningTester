//
// Created by caiu on 23/09/25.
//

#include "RolePlaceHolder.h"
#include "../RobotController.h"
#include "../TeamInfo.h"

namespace roles {
	void RolePlaceHolder::act(RobotController& robot) {
		LineSegment line{robot.get_world().ball.getPosition(), robot.get_m_team()->getBallPlacementSpot()};
		line = line.getResized(line.getLength() + robot.getRadius());
		Point p = line.getEnd();
		moveTo.act(robot, p, false);
		turnTo.act(robot, robot.get_world().ball.getPosition());
    }
} // roles