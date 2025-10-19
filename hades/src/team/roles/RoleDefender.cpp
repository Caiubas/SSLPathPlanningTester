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
        } else if (robot.get_world().isPointOnOurArea(robot.get_world().ball.getPosition())) {
            LineSegment l(Point(0, 0), (0, 0));
            if (robot.get_m_team()->getOurSide() == TeamInfo::left) l = LineSegment(Point(robot.get_world().field.ourDefenseArea.getMajorPoint().getX(), robot.get_world().field.ourDefenseArea.getMajorPoint().getY() - 500), Point(robot.get_world().field.ourDefenseArea.getMajorPoint().getX(), robot.get_world().field.ourDefenseArea.getMinorPoint().getY() + 500)).getMovedOnX(2*robot.getRadius());
            else l = LineSegment(Point(robot.get_world().field.ourDefenseArea.getMinorPoint().getX(), robot.get_world().field.ourDefenseArea.getMinorPoint().getY() - 500), Point(robot.get_world().field.ourDefenseArea.getMinorPoint().getX(), robot.get_world().field.ourDefenseArea.getMajorPoint().getY() + 500)).getMovedOnX(2*robot.getRadius());
            Point p(0, 0);

            p = l.intersection(LineSegment{Point(0, 0), robot.get_world().field.ourGoal.getMiddle()});
            moveTo.act(robot, p, true);
            turnTo.act(robot, robot.get_world().ball.getPosition());
        } else if (robot.get_world().getClosestAllyToPoint(robot.get_world().ball.getPosition()).getId() == robot.getId() && robot.get_world().getClosestAllyToPoint(robot.get_world().ball.getPosition()).getPosition().getDistanceTo(robot.get_world().ball.getPosition()) < robot.get_world().getClosestEnemyToPoint(robot.get_world().ball.getPosition()).getPosition().getDistanceTo(robot.get_world().ball.getPosition())) {
            //try {
            //positionAndKick.act(robot, robot.get_m_team()->getRobotofRole(Robot::striker));
            //} catch (...) {
            positionAndKick.act(robot, robot.get_world().field.theirGoal.getMiddle());
            //}
        }   else if (robot.get_world().isPointOnOurSide(robot.get_world().ball.getPosition()) && !robot.get_world().getBallOwner().isAlly()) {
            positionAndKick.act(robot, robot.get_world().field.theirGoal.getMiddle());
        } else {
            LineSegment line = {Point(0, 0), Point(0, 0)};
            if (robot.get_m_team()->getOurSide() == TeamInfo::left) line = LineSegment(robot.get_world().field.ourDefenseArea.getMajorPoint(), Point(robot.get_world().field.ourDefenseArea.getMajorPoint().getX(), robot.get_world().field.ourDefenseArea.getMinorPoint().getY())).getMovedOnX(2*robot.getRadius());
            else if (robot.get_m_team()->getOurSide() == TeamInfo::right) line = LineSegment(robot.get_world().field.ourDefenseArea.getMinorPoint(), Point(robot.get_world().field.ourDefenseArea.getMinorPoint().getX(), robot.get_world().field.ourDefenseArea.getMajorPoint().getY())).getMovedOnX(-2*robot.getRadius());
            keepXLine.act(robot, line, line.getMiddle().getY());
        }
    }
} // roles