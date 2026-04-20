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
        if (robot.get_world().isBallMovingRobotDirection(robot) && robot.get_world().ball.isMoving()) {
            intercept.act(robot);
        } else if (robot.get_world().getClosestAllyToPoint(robot.get_world().ball.getPosition()).getId() == robot.getId()) {
            try {
                positionAndKick.act(robot, robot.get_m_team()->getRobotToKickTo(robot));
            } catch (...) {
                positionAndKick.act(robot, robot.get_world().field.theirGoal.getMiddle());
            }
        } else {
            LineSegment line = {Point(0, 0), Point(0, 0)};
            if (robot.get_m_team()->getOurSide() == TeamInfo::left) line = LineSegment(robot.get_world().field.ourDefenseArea.getMajorPoint(), Point(robot.get_world().field.ourDefenseArea.getMajorPoint().getX(), robot.get_world().field.ourDefenseArea.getMinorPoint().getY())).getMovedOnX(2*robot.getRadius());
            else if (robot.get_m_team()->getOurSide() == TeamInfo::right) line = LineSegment(robot.get_world().field.ourDefenseArea.getMinorPoint(), Point(robot.get_world().field.ourDefenseArea.getMinorPoint().getX(), robot.get_world().field.ourDefenseArea.getMajorPoint().getY())).getMovedOnX(-2*robot.getRadius());
            keepXLine.act(robot, line, line.getMiddle().getY());
        }
    }
} // roles