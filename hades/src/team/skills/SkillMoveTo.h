//
// Created by caiu on 25/08/25.
//

#ifndef SKILLMOVETO_H
#define SKILLMOVETO_H
#include "SkillBase.h"
#include "../geometry/AreaRectangular.h"
#include "../geometry/Vector2d.h"
#include "../geometry/Point.h"
#include "../../c_trajectory/geometry/Rectangle.h"

namespace skills {

class SkillMoveTo : public SkillBase {
public:
	void act(RobotController& robot, Point goal, bool avoid_ball, bool full_field = false, bool ignore_stop = false);
	void act(RobotController& robot);
private:
	Vector2d motion_planner(RobotController& robot, std::vector<Point> trajectory);
	Vector2d motion_control(RobotController& robot, Vector2d v_target_world, double yaw);
	std::vector<Point> find_trajectory(RobotController& robot, Point start, Point goal, bool avoid_ball, bool ignore_stop = false, bool full_field = false);
	Rectangle getRectangle(AreaRectangular r);
};

} // skills

#endif //SKILLMOVETO_H
