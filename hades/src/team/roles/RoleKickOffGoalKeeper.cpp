//
// Created by caiu on 18/08/25.
//

#include "RoleKickOffGoalKeeper.h"
#include "../RobotController.h"

namespace roles {
    void RoleKickOffGoalKeeper::act(RobotController& robot) {
        Point goal = robot.get_world().field.ourGoal.getMiddle();
        moveTo.act(robot, goal, true);
    }
} // roles