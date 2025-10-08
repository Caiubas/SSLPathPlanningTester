//
// Created by caiu on 18/08/25.
//

#include "RoleKickOffSupport.h"
#include "../RobotController.h"
#include "../TeamInfo.h"

namespace roles {
    void RoleKickOffSupport::act(RobotController& robot) {
        Point goal(0, 0);
        if (robot.get_m_team()->getOurSide() == TeamInfo::left) goal = {-robot.getRadius(), 1000};
        else goal = {robot.getRadius(), 1000};
        moveTo.act(robot, goal, true);
        turnTo.act(robot, robot.get_world().ball.getPosition());
    }
} // roles