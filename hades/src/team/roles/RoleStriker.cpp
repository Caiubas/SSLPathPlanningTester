//
// Created by caiu on 18/08/25.
//

#include "RoleStriker.h"

#include <iostream>

#include "../RobotController.h"
#include "../TeamInfo.h"
#include "../geometry/Point.h"

namespace roles {
    Point RoleStriker::getSupportPosition(RobotController& robot) {
        int N = 12;
        int k1 = 1;
        std::vector<Point> points;
        points.reserve(N);
        for (int i = 0; i < N; i++) {
            double angle = 2.0 * M_PI * i / N;
            double x = 0;
            double y = 0;
            try {
                x = robot.get_world().ball.getPosition().getX() + robot.get_m_team()->getRobotofRole(Robot::striker).getKickDistance() * cos(angle);
                y = robot.get_world().ball.getPosition().getY() + robot.get_m_team()->getRobotofRole(Robot::striker).getKickDistance() * sin(angle);
            } catch (...) { // no striker
                x = robot.get_world().ball.getPosition().getX() + robot.getKickDistance() * cos(angle);
                y = robot.get_world().ball.getPosition().getY() + robot.getKickDistance() * sin(angle);
            }
            Point p(x, y);
            if (!robot.get_world().ball.isVisible(p)) continue;
            if (!robot.get_world().field.inside_dimensions.detectIfContains(p)) continue;    ////TODO problema quando posicoes caem dentro da area de defesa
            points.push_back(p);
        }


        int best_idx = 0;
        for (int i = 1; i < points.size(); i++) {
            if (points[best_idx].getDistanceTo(robot.get_world().field.theirGoal.getMiddle())*k1 > points[i].getDistanceTo(robot.get_world().field.theirGoal.getMiddle())*k1) { //TODO melhorar essa funcao
                best_idx = i;
            }
        }
        if (points.size() == 0) throw std::runtime_error("No support position found");
        return points[best_idx];
    }

    void RoleStriker::act(RobotController& robot) {
        Point goal = robot.get_world().field.theirGoal.getMiddle();
        bool hasGoalPosition = false;
        try {
            goal = robot.get_world().getGoalPosition(robot.get_m_team()->getEnemyofRole(Robot::goal_keeper, robot.get_world().enemies));
            hasGoalPosition = true;
        } catch (...) {
            //std::cout << "no score position found" << std::endl;
            goal = robot.get_world().field.theirGoal.getMiddle();
        }
        bool theyHaveGoalKeeper = false;
        Robot theirGoalKeeper(-1);
        try {
            theirGoalKeeper = robot.get_m_team()->getEnemyofRole(Robot::goal_keeper, robot.get_world().enemies);
            theyHaveGoalKeeper = true;
        } catch (...) {}
        Robot support(-1);
        bool hasSupport = false;
        try {
            support = robot.get_m_team()->getRobotofRole(Robot::support);
            hasSupport = true;
        } catch (...) {}
        LineSegment robot_goal = {robot.get_world().ball.getPosition(), goal};
        if (robot.get_world().ball.isMoving() && robot.get_world().ball.getMovementLine().isPointAligned(robot.getPosition(), 3.1415/4)) {
            intercept.act(robot);
        }
        else if (((robot.get_world().ball.isMoving() && !robot.isKickingOnVision())|| robot.get_world().isPointOnOurArea(robot.get_world().ball.getPosition()) && hasSupport)) {
            Point p = getSupportPosition(robot);
            keepLocation.act(robot, p);

        } else if (robot.get_world().isPointOnTheirArea(robot.get_world().ball.getPosition()) && theyHaveGoalKeeper) {    ////TODO criar uma play pra quando a bola ta na area de defesa inimiga
            blockBall.act(robot, theirGoalKeeper, fabs(robot.get_world().field.theirDefenseArea.getMajorPoint().getX() - robot.get_world().field.theirDefenseArea.getMinorPoint().getX()));
        }
        else if (hasGoalPosition && robot_goal.getLength() <= robot.getKickDistance()) {
            positionAndKick.act(robot, goal);
        } else if (hasSupport) {
            positionAndKick.act(robot, support);
        } else {
            positionAndPush.act(robot, goal);
        }
    }
} // roles