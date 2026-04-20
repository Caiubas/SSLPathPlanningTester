//
// Created by caiu on 18/08/25.
//

#include "RoleKickOffGoalKeeper.h"
#include "../RobotController.h"

namespace roles {
    void RoleKickOffGoalKeeper::act(RobotController& robot) {
        Point goal = {(robot.get_world().field.ourGoal.getStart().getX() + robot.get_world().field.ourGoal.getEnd().getX())/2, (robot.get_world().field.ourGoal.getStart().getY() + robot.get_world().field.ourGoal.getEnd().getY())/2};
            moveTo.act(robot, goal, true);
    }
} // roles