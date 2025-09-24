//
// Created by caiu on 25/08/25.
//

#include <stdio.h>
#include "SkillMoveTo.h"

#include <iostream>

#include "../RobotController.h"
#include "../TeamInfo.h"
#include "../c_trajectory/C_trajectory.h"

namespace skills {
    Rectangle SkillMoveTo::getRectangle(AreaRectangular r) {
        return Rectangle({r.getMinorPoint().getX(), r.getMinorPoint().getY()}, {r.getMajorPoint().getX(), r.getMajorPoint().getY()});
    }

    Vector2d SkillMoveTo::motion_planner(RobotController& robot, std::vector<Point> trajectory) {
        Vector2d delta = {trajectory[1].getX() - robot.getPosition().getX(), trajectory[1].getY() - robot.getPosition().getY()};
        double dist = delta.getNorm() / 1000.0; // metros
        auto direction = delta.getNormalized(1);

        double curve_safe_speed = robot.mVxy_min;
        double v_target_magnitude = robot.mVxy_max;

        double max_speed = robot.mVxy_max;

        if ((robot.mTeam->event == TeamInfo::stop or robot.mTeam->event == TeamInfo::timeout or robot.mTeam->event == TeamInfo::prepareOurKickOff
            or robot.mTeam->event == TeamInfo::prepareTheirKickOff or robot.mTeam->event == TeamInfo::prepareOurPenalty or robot.mTeam->event == TeamInfo::prepareTheirPenalty
            or robot.mTeam->event == TeamInfo::ourballPlacement or robot.mTeam->event == TeamInfo::theirballPlacement)) {
            max_speed = robot.mTeam->stop_max_speed;
            v_target_magnitude = robot.mTeam->stop_max_speed;
        }
        if (trajectory.size() > 2) {
                Point p0 = robot.getPosition();
                Point p1 = trajectory[1];
                Point p2 = trajectory[2];

                Vector2d v1 = {p1 , p0};
                Vector2d v2 = {p2, p1};

                double angle_between = v1.getAngleBetween(v2);
                double chord_length = p2.getDistanceTo(p1); // mm
                chord_length /= 1000.0; // Para metros

                double radius = chord_length / (2.0 * sin(std::max(angle_between/2, 1e-3)));
                radius = std::max(radius, 0.05);
                curve_safe_speed = sqrt(max_speed * radius)/4;
                curve_safe_speed = std::clamp(curve_safe_speed, robot.mVxy_min, max_speed);

                if (angle_between > 70.0 * M_PI / 180.0) {
                    curve_safe_speed = robot.mVxy_min;
                }

            }

            double current_speed = robot.getVelocity().getNorm();

            double brake_distance = (max_speed*max_speed - curve_safe_speed * curve_safe_speed) / (2.0 * robot.mA_xy_brake);
            brake_distance = std::max(brake_distance, 0.0);
            if (dist <= brake_distance) {
                v_target_magnitude = curve_safe_speed;
            } else {
                v_target_magnitude = max_speed;
            }

            v_target_magnitude = std::clamp(v_target_magnitude, -max_speed, max_speed);
            std::vector<double> v_target = {v_target_magnitude * direction.getX(), v_target_magnitude * direction.getY()};
            std::vector<double> error = {v_target[0] - robot.mlast_target_vel.getX(), v_target[1] - robot.mlast_target_vel.getY()};
            std::vector<double> acceleration = {0, 0};

            if (fabs(v_target[0]) > fabs(robot.mlast_target_vel.getX())) {
                acceleration[0] = std::clamp(error[0] / robot.mDelta_time, -robot.mA_xy_max, robot.mA_xy_max);
            } else {
                acceleration[0] = std::clamp(error[0] / robot.mDelta_time, -robot.mA_xy_brake, robot.mA_xy_brake);
            }

            if (fabs(v_target[1]) > fabs(robot.mlast_target_vel.getY())) {
                acceleration[1] = std::clamp(error[1] / robot.mDelta_time, -robot.mA_xy_max, robot.mA_xy_max);
            } else {
                acceleration[1] = std::clamp(error[1] / robot.mDelta_time, -robot.mA_xy_brake, robot.mA_xy_brake);
            }


            Vector2d vel_cmd = {robot.mlast_target_vel.getX() + acceleration[0]*robot.mDelta_time, robot.mlast_target_vel.getY() + acceleration[1]*robot.mDelta_time};
            if (vel_cmd.getNorm() > max_speed) {
                vel_cmd = vel_cmd.getNormalized(max_speed);
            }

            if (std::isnan(vel_cmd.getX())) vel_cmd.setX(0);
            if (std::isnan(vel_cmd.getY())) vel_cmd.setY(0);

            robot.mlast_target_vel = vel_cmd;
            return vel_cmd;
        }

        Vector2d SkillMoveTo::motion_control(Vector2d v_vet, double yaw) {
            const double ang = yaw;
            return v_vet.getRotated(ang);
        }


        std::vector<Point> SkillMoveTo::find_trajectory(RobotController& robot, Point start, Point goal, bool avoid_ball = true, bool full_field, bool ignore_stop) {
            double minor[2];
            double major[2];
            if (full_field or robot.getRole()  == Robot::goal_keeper) {
                minor[0] = robot.mWorld.field.full_dimensions.getResized(-robot.getRadius()).getMinorPoint().getX();
                minor[1] = robot.mWorld.field.full_dimensions.getResized(-robot.getRadius()).getMinorPoint().getY();
                major[0] = robot.mWorld.field.full_dimensions.getResized(-robot.getRadius()).getMajorPoint().getX();
                major[1] = robot.mWorld.field.full_dimensions.getResized(-robot.getRadius()).getMajorPoint().getY();

            } else {
                minor[0] = robot.mWorld.field.inside_dimensions.getResized(-robot.getRadius()).getMinorPoint().getX();
                minor[1] = robot.mWorld.field.inside_dimensions.getResized(-robot.getRadius()).getMinorPoint().getY();
                major[0] = robot.mWorld.field.inside_dimensions.getResized(-robot.getRadius()).getMajorPoint().getX();
                major[1] = robot.mWorld.field.inside_dimensions.getResized(-robot.getRadius()).getMajorPoint().getY();
            }
            C_trajectory pf(false, false, 0, 100, 50, 0, minor, major);

            std::vector<Circle> obs_circular = {};
            std::vector<Rectangle> obs_rectangular = {};
            std::vector<TiltedRectangle> obs_tilted = {};
            Rectangle r({0, 0}, {0, 0});
            //rectangle r = field.their_defense_area;
            //obs_rectangular.push_back(r);

            if (!ignore_stop && (robot.mTeam->event == TeamInfo::stop or robot.mTeam->event == TeamInfo::timeout or robot.mTeam->event == TeamInfo::prepareOurKickOff
                or robot.mTeam->event == TeamInfo::prepareTheirKickOff or robot.mTeam->event == TeamInfo::prepareOurPenalty or robot.mTeam->event == TeamInfo::prepareTheirPenalty
                or robot.mTeam->event == TeamInfo::theirballPlacement  or robot.mTeam->event == TeamInfo::theirFreeKick
                or robot.mTeam->event == TeamInfo::runningTheirFreeKick)) {
                Circle c({robot.mWorld.ball.getPosition().getX(), robot.mWorld.ball.getPosition().getY()}, robot.mTeam->stop_distance_to_ball + robot.mRadius);
                obs_circular.push_back(c);
            }

            if (robot.mTeam->event == TeamInfo::theirballPlacement) {
                auto t = TiltedRectangle({robot.mWorld.ball.getPosition().getX(), robot.mWorld.ball.getPosition().getY()}, {robot.mTeam->ball_placement_spot.getX(), robot.mTeam->ball_placement_spot.getY()}, 500);
                obs_tilted.push_back(t);
                Circle c({robot.mWorld.ball.getPosition().getX(), robot.mWorld.ball.getPosition().getY()}, robot.mBall_avoidance_radius + robot.mRadius);
                obs_circular.push_back(c);
            }

            //add static ball to obstacles according to avoidance radius
            if (avoid_ball) {
                Circle c({robot.mWorld.ball.getPosition().getX(), robot.mWorld.ball.getPosition().getY()}, robot.mBall_avoidance_radius + robot.mRadius);
                obs_circular.push_back(c);
            }

            //add static allies to obstacles
            for (int i = 0; i < size(robot.mWorld.allies) ; i++) {
                if (!robot.mWorld.allies[i].isDetected() || i == robot.getId()) {
                    continue;
                }
                Circle c({robot.mWorld.allies[i].getPosition().getX(), robot.mWorld.allies[i].getPosition().getY()}, robot.mRadius);
                obs_circular.push_back(c);
            }

            //add static enemies to obstacles
            for (int i = 0; i < size(robot.mWorld.enemies) ; i++) {
                if (!robot.mWorld.enemies[i].isDetected()) {
                    continue;
                }
                Circle c({robot.mWorld.enemies[i].getPosition().getX(), robot.mWorld.enemies[i].getPosition().getY()}, robot.mRadius);
                obs_circular.push_back(c);
            }

            if (!((robot.getRole() == Robot::placer || robot.getRole() == Robot::placeHolder) && robot.mTeam->event == TeamInfo::ourballPlacement)) {
                if (robot.mTeam->roles[robot.getId()] != Robot::goal_keeper) {
                    obs_rectangular.push_back(getRectangle(robot.mWorld.field.ourDefenseArea.getResized(robot.getRadius())));
                }
                obs_rectangular.push_back(getRectangle(robot.mWorld.field.theirDefenseArea.getResized(robot.getRadius())));
            }

            double wall_thickness = robot.mWorld.field.goalBarrierThickness;
            AreaRectangular a({0, 0}, {0, 0});
            a = robot.mWorld.field.leftFisicalBarrier;
            a.setMajorPoint({a.getMajorPoint().getX(), a.getMinorPoint().getY() + wall_thickness});
            obs_rectangular.push_back(getRectangle(a.getResized(robot.getRadius()/2)));
            a = robot.mWorld.field.leftFisicalBarrier;
            a.setMinorPoint({a.getMinorPoint().getX(), a.getMajorPoint().getY() - wall_thickness});
            obs_rectangular.push_back(getRectangle(a.getResized(robot.getRadius()/2)));
            a = robot.mWorld.field.leftFisicalBarrier;
            a.setMajorPoint({a.getMinorPoint().getX() + wall_thickness, a.getMajorPoint().getY()});
            a.setMinorPoint({a.getMinorPoint().getX() - 10000, a.getMinorPoint().getY()});
            obs_rectangular.push_back(getRectangle(a.getResized(robot.getRadius()/2)));

            a = robot.mWorld.field.rightFisicalBarrier;
            a.setMajorPoint({a.getMajorPoint().getX(), a.getMinorPoint().getY() + wall_thickness});
            obs_rectangular.push_back(getRectangle(a.getResized(robot.getRadius()/2)));
            a = robot.mWorld.field.rightFisicalBarrier;
            a.setMinorPoint({a.getMinorPoint().getX(), a.getMajorPoint().getY() - wall_thickness});
            obs_rectangular.push_back(getRectangle(a.getResized(robot.getRadius()/2)));
            a = robot.mWorld.field.rightFisicalBarrier;
            a.setMinorPoint({a.getMajorPoint().getX() - wall_thickness, a.getMinorPoint().getY()});
            a.setMajorPoint({a.getMajorPoint().getX() + 10000, a.getMajorPoint().getY()});
            obs_rectangular.push_back(getRectangle(a.getResized(robot.getRadius()/2)));

            auto trajectory_vector = pf.path_find(start.getVector(), goal.getVector(), obs_circular, obs_rectangular, obs_tilted);
            std::vector<Point> trajectory = {};
            for (int i = 0; i < trajectory_vector.size(); i++) {
                if (trajectory_vector.size() > 1 && i == 1) {if (trajectory_vector[0][0] == trajectory_vector[1][0] && trajectory_vector[0][1] == trajectory_vector[1][1]) continue;}
                trajectory.emplace_back(trajectory_vector[i][0], trajectory_vector[i][1]);
            }
            if (trajectory.size() == 1) return {};
            return trajectory;
        }


    void SkillMoveTo::act(RobotController& robot, Point goal, bool avoid_ball, bool full_field, bool ignore_stop) {
        auto trajectory = find_trajectory(robot, robot.getPosition(), goal, avoid_ball, full_field, ignore_stop);

        if (trajectory.size() > 1) {
            if (robot.getPosition().getDistanceTo(trajectory[size(trajectory) - 1]) < robot.mStatic_position_tolarance) {
                robot.mtarget_vel = {0, 0};
                robot.mtarget_vyaw = 0;
                robot.positioned = true;
                robot.mTeam->positioned[robot.getId()] = true;
                return;
            }
        }
        robot.positioned = false;
        robot.mTeam->positioned[robot.getId()] = false;
        Vector2d v_vet;
        std::size(trajectory) > 1 ? v_vet = motion_planner(robot, trajectory) : v_vet = Vector2d({0, 0}, robot.getPosition()).getNormalized(robot.mVxy_min);

        v_vet = motion_control(v_vet, -robot.getYaw());

        robot.mtarget_vel = v_vet;
        robot.mkicker_x = 0;
    }

    void SkillMoveTo::act(RobotController& robot) {
    	std::cout << "Dummy method" << std::endl;
    }
} // skills