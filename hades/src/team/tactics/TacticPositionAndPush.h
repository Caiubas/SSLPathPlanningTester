//
// Created by caiu on 06/10/25.
//

#ifndef TACTICPOSITIONANDPUSH_H
#define TACTICPOSITIONANDPUSH_H

#include "TacticBase.h"

class Robot;

namespace tactics {

class TacticPositionAndPush {
private:
	double distance_to_kick = 300;
public:
	void act(RobotController& robot, Point goal, bool wait = false);
	void act(RobotController& robot, Robot sup, bool wait = false);
	void act(RobotController& robot);
};

} // tactics

#endif //TACTICPOSITIONANDPUSH_H
