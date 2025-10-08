//
// Created by caiu on 26/08/25.
//

#include "SkillCushion.h"

#include <iostream>

#include "../geometry/Vector2d.h"
#include "../RobotController.h"

namespace skills {
void SkillCushion::act(RobotController& robot) {
    Vector2d vector = robot.get_world().ball.getVelocity().getNormalized(robot.get_m_vxy_min());
    robot.set_mtarget_vel(vector.getRotated(-robot.getYaw()));
}
} // skills