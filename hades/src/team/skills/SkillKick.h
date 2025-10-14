//
// Created by caiu on 25/08/25.
//

#ifndef SKILLKICK_H
#define SKILLKICK_H
#include "SkillBase.h"

class Point;

namespace skills {

class SkillKick {
private:
	double distancethreshold = 500;
	double find_angle_error(RobotController& robot, Point goal);
public:
	void act(RobotController& robot);
};

} // skills

#endif //SKILLKICK_H
