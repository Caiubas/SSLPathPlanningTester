//
// Created by caiu on 25/08/25.
//

#include "TacticKeepXLine.h"

#include <iostream>

#include "../RobotController.h"
#include "../TeamInfo.h"

namespace tactics {
void TacticKeepXLine::act(RobotController& robot, LineSegment y_segment, double y_rest) {
        bool hasEnemies = false;
        for (Robot rob : robot.get_world().enemies) if (rob.isDetected()) {hasEnemies = true; break;};

        bool hasStriker = false;
        int enemy_striker_id = -1;
        Robot enemy_striker(0);
        try {
            //enemy_striker = robot.get_m_team()->getEnemyofRole(Robot::striker, robot.get_world().enemies);
            enemy_striker = robot.get_m_team()->getRobotofRole(Robot::striker); //TODO REMOVER;
            enemy_striker_id = enemy_striker.getId();
            hasStriker = true;
        } catch (...) {
            std::cout << "no enemy striker" << std::endl;
        }

        double a = 1000;
        LineSegment line(Point(0, 0), Point(0, 0));
        if (true) {
            if (robot.get_world().ball.isMoving()) {
                line = robot.get_world().ball.getMovementLine().getResized(1500000);
            }
            else if (robot.get_world().ball.isStopped() && hasStriker) {    //para bola parada
                //line = LineSegment(enemy_striker.getPosition(), robot.get_world().ball.getPosition()).getResized(100000); //TODO REMOVER

                line = LineSegment(enemy_striker.getPosition(), robot.get_world().ball.getPosition()).getResized(100000); //TODO REMOVER
            } else {
                line = LineSegment(robot.get_world().ball.getPosition(), y_segment.getMiddle()).getResized(100000);
            }
        }
        Point p = {y_segment.getStart().getX(), y_rest};
        try {
            p = line.intersection(y_segment);
        } catch (...) {
        }


        double y_max = y_segment.getEnd().getY() - robot.getRadius();
        double y_min = y_segment.getStart().getY() + robot.getRadius();
        p.setY(std::clamp(p.getY(), y_min, y_max));

        if (robot.get_world().enemies.size() > 0 && hasStriker && false) {
            if ((robot.get_world().enemies[enemy_striker_id].getPosition().getDistanceTo(robot.get_world().ball.getPosition()) > 1000 && robot.get_world().ball.isStopped())) {
                p.setY(y_rest);
            }
        }
        moveTo.act(robot, p, false);
        robot.set_mkicker_x(0);
        turnTo.act(robot, robot.get_world().ball.getPosition());
    }
    void TacticKeepXLine::act(RobotController& robot) {
        std::cout << "this is a dummy method" << std::endl;
    }
} // tactics