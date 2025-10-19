//
// Created by caiu on 18/08/25.
//

#include "RoleStriker.h"

#include <iostream>

#include "../RobotController.h"
#include "../TeamInfo.h"
#include "../geometry/Point.h"
#include "../c_trajectory/C_trajectory.h"

namespace roles {
    Point RoleStriker::find_ball_trajectory(RobotController& robot, Point goal) {
        double minor[2];
        double major[2];
        minor[0] = robot.get_world().field.inside_dimensions.getResized(-robot.getRadius()).getMinorPoint().getX();
        minor[1] = robot.get_world().field.inside_dimensions.getResized(-robot.getRadius()).getMinorPoint().getY();
        major[0] = robot.get_world().field.inside_dimensions.getResized(-robot.getRadius()).getMajorPoint().getX();
        major[1] = robot.get_world().field.inside_dimensions.getResized(-robot.getRadius()).getMajorPoint().getY();
        goal.setX(std::clamp(goal.getX(), minor[0], major[0]));
        goal.setY(std::clamp(goal.getY(), minor[1], major[1]));
        C_trajectory pf(false, false, 0, 100, 50, 0, minor, major);

            std::vector<Circle> obs_circular = {};
            std::vector<Rectangle> obs_rectangular = {};
            std::vector<TiltedRectangle> obs_tilted = {};
            Rectangle r({0, 0}, {0, 0});

            //add static enemies to obstacles
            for (int i = 0; i < size(robot.get_world().enemies) ; i++) {
                if (!robot.get_world().enemies[i].isDetected()) {
                    continue;
                }
                Circle c({robot.get_world().enemies[i].getPosition().getX(), robot.get_world().enemies[i].getPosition().getY()}, robot.getRadius() + robot.get_world().enemies[i].getRadius());
                obs_circular.push_back(c);
            }

            auto trajectory_vector = pf.path_find(robot.get_world().ball.getPosition().getVector(), goal.getVector(), obs_circular, obs_rectangular, obs_tilted);
            std::vector<Point> trajectory = {};
            for (int i = 0; i < trajectory_vector.size(); i++) {
                if (trajectory_vector.size() > 1 && i == 1) {if (trajectory_vector[0][0] == trajectory_vector[1][0] && trajectory_vector[0][1] == trajectory_vector[1][1]) continue;}
                trajectory.emplace_back(std::clamp(trajectory_vector[i][0], minor[0], major[0]), std::clamp(trajectory_vector[i][1], minor[1], major[1]));
            }
            if (trajectory.size() == 1) return robot.get_world().field.theirGoal.getMiddle();
            return trajectory[1];
    }

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
        if (robot.get_world().ball.isMoving() && robot.get_world().ball.getMovementLine().isPointAligned(robot.getPosition(), 3.1415/4) && robot.get_world().isBallMovingRobotDirection(robot)) {
            intercept.act(robot);
        }
        else if ((robot.get_world().isPointOnOurArea(robot.get_world().ball.getPosition()))) {//|| ((robot.get_world().ball.isMoving() && !robot.isKickingOnVision()))) {
            Point p = getSupportPosition(robot);
            keepLocation.act(robot, p);

        } else if (robot.get_world().isPointOnTheirArea(robot.get_world().ball.getPosition()) && theyHaveGoalKeeper) {    ////TODO criar uma play pra quando a bola ta na area de defesa inimiga
            blockBall.act(robot, theirGoalKeeper, fabs(robot.get_world().field.theirDefenseArea.getMajorPoint().getX() - robot.get_world().field.theirDefenseArea.getMinorPoint().getX()));
        }
        else if (hasGoalPosition && robot_goal.getLength() <= robot.getKickDistance()) {
            positionAndKick.act(robot, goal);
            //std::cout << "kicking on goal: " << goal.getX() << " " << goal.getY() << std::endl;
        } else if (hasSupport) {
            positionAndKick.act(robot, support);
        } else if (hasGoalPosition){
            positionAndPush.act(robot, goal);
        } else {
            try {
                Point p = getSupportPosition(robot);
                positionAndPush.act(robot, p);
            } catch (...) {
                positionAndPush.act(robot, goal);
            }
        }
    }
} // roles