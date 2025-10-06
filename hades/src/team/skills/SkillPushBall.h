//
// Created by caiu on 06/10/25.
//

#ifndef SKILLPUSHBALL_H
#define SKILLPUSHBALL_H
#include "SkillBase.h"

class Point;

namespace skills {

class SkillPushBall {
private:
	double distancethreshold = 350;
	double find_angle_error(RobotController robot, Point goal);
public:
	void act(RobotController& robot);
};

} // skills

#endif //SKILLPUSHBALL_H
