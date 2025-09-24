//
// Created by caiu on 18/08/25.
//

#include "RoleGoalKeeper.h"

#include <iostream>
#include <ostream>

#include "../RobotController.h"
#include "../TeamInfo.h"

namespace roles {
    void RoleGoalKeeper::act(RobotController& robot) {
        //TODO melhorar goal_keeper_role
        bool has_support = false;
        Robot support(-1);
        bool has_goal = false;
        Point goal((robot.mWorld.field.theirGoal.getStart().getX() + robot.mWorld.field.theirGoal.getEnd().getX())/2, (robot.mWorld.field.theirGoal.getStart().getY() + robot.mWorld.field.theirGoal.getEnd().getY())/2);

        try {
            support = robot.mTeam->getRobotofRole(Robot::support);
            has_support = true;
        } catch (...) {}

        try {
            goal = robot.mWorld.getGoalPosition();
            has_goal = true;
        } catch (...) {}

        if (!robot.mWorld.isPointOnOurArea(robot.mWorld.ball.getPosition()) || robot.mWorld.ball.isMoving()) {
            LineSegment line = robot.mWorld.field.ourGoal;
            if (line.getStart().getX() < 0) line = line.getMovedOnX(robot.getRadius());
            if (line.getStart().getX() > 0) line = line.getMovedOnX(-robot.getRadius());

            keepXLine.act(robot, line, (robot.mWorld.field.ourGoal.getStart().getY() + robot.mWorld.field.ourGoal.getEnd().getY())/2);
        } else {
            if (has_goal) {
                positionAndKick.act(robot, goal);
            } else if (has_support) {
                positionAndKick.act(robot, support);
            } else {
                positionAndKick.act(robot, goal);
            }
        }
    }
} // roles