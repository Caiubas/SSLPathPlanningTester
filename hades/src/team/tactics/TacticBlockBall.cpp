//
// Created by caiu on 18/09/25.
//

#include "TacticBlockBall.h"
#include "../TeamInfo.h"

namespace tactics {
	void TacticBlockBall::act(RobotController& robot, Robot toBlock, double block_distance) {
		LineSegment line(Point(0, 0), Point(0, 0));
		line = LineSegment(toBlock.getPosition(), robot.mWorld.ball.getPosition());
		line = line.getResized(line.getLength() + block_distance);
		Point p = line.getEnd();
		moveTo.act(robot, p, false, false);
		turnTo.act(robot, robot.mWorld.ball.getPosition());
	}
} // tactics