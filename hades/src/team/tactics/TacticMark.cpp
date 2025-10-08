//
// Created by caiu on 13/09/25.
//

#include <iostream>
#include "TacticMark.h"
#include "../RobotController.h"

namespace tactics {

void TacticMark::act(RobotController& robot) {
	std::cout << "This is a dummy method Mark" << std::endl;
}

Point TacticMark::findMarkPosition(RobotController& robot, Robot toMark) {
	LineSegment l(toMark.getPosition(), robot.get_world().ball.getPosition());
	l = l.getResized(l.getLength()/2);
	return l.getEnd();
}

void TacticMark::act(RobotController& robot, Robot toMark) {
	moveTo.act(robot, findMarkPosition(robot, toMark), false);
	turnTo.act(robot, robot.get_world().ball.getPosition());
}

} // tactics