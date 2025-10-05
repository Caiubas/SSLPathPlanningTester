//
// Created by caiu on 18/08/25.
//

#include "RoleDefender.h"
#include "../RobotController.h"
#include <iostream>
#include <ostream>

#include "../TeamInfo.h"

namespace roles {
    void RoleDefender::act(RobotController& robot) {
        if (robot.mWorld.isBallMovingRobotDirection(robot) && robot.mWorld.ball.isMoving()) {
            intercept.act(robot);
            std::cout << "INTERCEPTING" << std::endl;
        } else if (robot.mWorld.getClosestAllyToPoint(robot.mWorld.ball.getPosition()).getId() == robot.getId() && false) { //TODO remover
            try {
                positionAndKick.act(robot, robot.mTeam->getRobotToKickTo(robot));
            } catch (...) {
                positionAndKick.act(robot, robot.mWorld.field.theirGoal.getMiddle());
            }
        } else {
            LineSegment line = {Point(0, 0), Point(0, 0)};
            if (robot.mTeam->our_side == TeamInfo::left) line = LineSegment(robot.mWorld.field.ourDefenseArea.getMajorPoint(), Point(robot.mWorld.field.ourDefenseArea.getMajorPoint().getX(), robot.mWorld.field.ourDefenseArea.getMinorPoint().getY())).getMovedOnX(2*robot.getRadius());
            else if (robot.mTeam->our_side == TeamInfo::right) line = LineSegment(robot.mWorld.field.ourDefenseArea.getMinorPoint(), Point(robot.mWorld.field.ourDefenseArea.getMinorPoint().getX(), robot.mWorld.field.ourDefenseArea.getMajorPoint().getY())).getMovedOnX(-2*robot.getRadius());
            keepXLine.act(robot, line, line.getMiddle().getY());
        }
    }
} // roles