//
// Created by caiu on 18/08/25.
//

#include "RoleKickOffSupport.h"
#include "../RobotController.h"
#include "../TeamInfo.h"

namespace roles {
    void RoleKickOffSupport::act(RobotController& robot) {
        Point goal(0, 0);
        if (robot.mTeam->our_side == TeamInfo::left) goal = {-robot.mRadius, 1000};
        else goal = {robot.mRadius, 1000};
        moveTo.act(robot, goal, true);
        turnTo.act(robot, robot.mWorld.ball.getPosition());
    }
} // roles