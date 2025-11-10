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
        bool has_striker = false;
        Robot striker(-1);
        bool has_goal = false;
        Point goal((robot.get_world().field.theirGoal.getStart().getX() + robot.get_world().field.theirGoal.getEnd().getX())/2, (robot.get_world().field.theirGoal.getStart().getY() + robot.get_world().field.theirGoal.getEnd().getY())/2);

        try {
            striker = robot.get_m_team()->getRobotofRole(Robot::striker);
            has_striker = true;
        } catch (...) {}

        try {
            goal = robot.get_world().getGoalPosition();
            has_goal = true;
        } catch (...) {}
        if (!robot.get_world().isPointOnOurArea(robot.get_world().ball.getPosition()) || robot.get_world().ball.isMoving()) {
            LineSegment line = robot.get_world().field.ourGoal;
            if (line.getStart().getX() < 0) line = line.getMovedOnX(robot.getRadius());
            if (line.getStart().getX() > 0) line = line.getMovedOnX(-robot.getRadius());
            keepXLine.act(robot, line, (robot.get_world().field.ourGoal.getStart().getY() + robot.get_world().field.ourGoal.getEnd().getY())/2);
        } else {
            if (has_goal && false) { //TODO remover
                positionAndKick.act(robot, goal);
            } else if (has_striker) {
                //positionAndKick.act(robot, striker); //TODO REMOVER
                positionAndKick.act(robot, Point(1000, 0));
            } else {
                positionAndKick.act(robot, goal);
            }
        }
    }
} // roles