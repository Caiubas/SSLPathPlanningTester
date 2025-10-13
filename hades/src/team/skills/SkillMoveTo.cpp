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

        double curve_safe_speed = robot.get_m_vxy_min();
        double v_target_magnitude = robot.get_m_vxy_max();

        double max_speed = robot.get_m_vxy_max();

        if ((robot.get_m_team()->getEvent() == TeamInfo::stop or robot.get_m_team()->getEvent() == TeamInfo::timeout or robot.get_m_team()->getEvent() == TeamInfo::prepareOurKickOff
            or robot.get_m_team()->getEvent() == TeamInfo::prepareTheirKickOff or robot.get_m_team()->getEvent() == TeamInfo::prepareOurPenalty or robot.get_m_team()->getEvent() == TeamInfo::prepareTheirPenalty
            or robot.get_m_team()->getEvent() == TeamInfo::theirballPlacement)) {
            max_speed = robot.get_m_team()->getStopMaxSpeed();
            v_target_magnitude = robot.get_m_team()->getStopMaxSpeed();
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
                curve_safe_speed = std::clamp(curve_safe_speed, robot.get_m_vxy_min(), max_speed);

                if (angle_between > 70.0 * M_PI / 180.0) {
                    curve_safe_speed = robot.get_m_vxy_min();
                }

        }

            double current_speed = robot.getVelocity().getNorm();

            double brake_distance = 1.5*(max_speed*max_speed - curve_safe_speed * curve_safe_speed) / (2.0 * robot.get_m_a_xy_max());    //TODO usar aceleracao normal?
            brake_distance = std::max(brake_distance, 0.0);
            if (dist <= brake_distance) {
                v_target_magnitude = curve_safe_speed;
            } else {
                v_target_magnitude = max_speed;
            }

            v_target_magnitude = std::clamp(v_target_magnitude, -max_speed, max_speed);
            std::vector<double> v_target = {v_target_magnitude * direction.getX(), v_target_magnitude * direction.getY()};
            std::vector<double> error = {v_target[0] - robot.get_mlast_target_vel().getX(), v_target[1] - robot.get_mlast_target_vel().getY()};
            std::vector<double> acceleration = {0, 0};

            if (fabs(v_target[0]) > fabs(robot.get_mlast_target_vel().getX())) {
                acceleration[0] = std::clamp(error[0] / robot.get_m_delta_time(), -robot.get_m_a_xy_max(), robot.get_m_a_xy_max());
            } else {
                acceleration[0] = std::clamp(error[0] / robot.get_m_delta_time(), -robot.get_m_a_xy_brake(), robot.get_m_a_xy_brake());
            }

            if (fabs(v_target[1]) > fabs(robot.get_mlast_target_vel().getY())) {
                acceleration[1] = std::clamp(error[1] / robot.get_m_delta_time(), -robot.get_m_a_xy_max(), robot.get_m_a_xy_max());
            } else {
                acceleration[1] = std::clamp(error[1] / robot.get_m_delta_time(), -robot.get_m_a_xy_brake(), robot.get_m_a_xy_brake());
            }


            Vector2d vel_cmd = {robot.get_mlast_target_vel().getX() + acceleration[0]*robot.get_m_delta_time(), robot.get_mlast_target_vel().getY() + acceleration[1]*robot.get_m_delta_time()};
            if (vel_cmd.getNorm() > max_speed) {
                vel_cmd = vel_cmd.getNormalized(max_speed);
            }

            if (std::isnan(vel_cmd.getX())) vel_cmd.setX(0);
            if (std::isnan(vel_cmd.getY())) vel_cmd.setY(0);

            robot.set_mlast_target_vel(vel_cmd);
            return vel_cmd;
        }

    Vector2d SkillMoveTo::motion_control(RobotController& robot, Vector2d v_target_world, double yaw) {
        // --- 1) Medir velocidade atual ---
        Vector2d v_meas_world = robot.getVelocity();

        // --- 2) Calcular erro em cada eixo ---
        double delta_x = v_target_world.getX() - v_meas_world.getX();
        double delta_y = v_target_world.getY() - v_meas_world.getY();

        // --- 3) Ganhos PID ---
        double Kp = robot.get_m_kp_vxy();
        double Ki = robot.get_m_ki_vxy();
        double Kd = robot.get_m_kd_vxy();

        // --- 4) Delta time ---
        double dt = robot.get_m_delta_time();
        if (dt <= 0) dt = 1e-3;

        // --- 5) PID eixo X ---
        robot.set_m_i_vx(robot.get_m_i_vx() + delta_x * dt);
        robot.set_m_i_vx(std::clamp(robot.get_m_i_vx(), -robot.get_m_i_vxy_max(), robot.get_m_i_vxy_max()));
        double d_delta_x = (delta_x - robot.get_m_last_delta_vx()) / dt;
        double pid_x = Kp * delta_x + Ki * robot.get_m_i_vx() + Kd * d_delta_x;

        // --- 6) PID eixo Y ---
        robot.set_m_i_vy(robot.get_m_i_vy() + delta_y * dt);
        robot.set_m_i_vy(std::clamp(robot.get_m_i_vy(), -robot.get_m_i_vxy_max(), robot.get_m_i_vxy_max()));
        double d_delta_y = (delta_y - robot.get_m_last_delta_vy()) / dt;
        double pid_y = Kp * delta_y + Ki * robot.get_m_i_vy() + Kd * d_delta_y;

        // --- 7) Armazenar últimos deltas ---
        robot.set_m_last_delta_vx(delta_x);
        robot.set_m_last_delta_vy(delta_y);

        // --- 8) Somar feedforward ---
        double out_x = v_target_world.getX() + pid_x;
        double out_y = v_target_world.getY() + pid_y;

        // --- 10) Saturação de velocidade máxima ---
        double max_v = robot.get_m_vxy_max();
        if (std::hypot(out_x, out_y) > max_v) {
            double s = max_v*1.5 / std::hypot(out_x, out_y);
            out_x *= s;
            out_y *= s;       //TODO verificar isso. Saída não pode superar em 50% a velocidade maxima
        }

        // --- 11) Rotacionar para enviar ao robô ---
        return Vector2d(out_x, out_y).getRotated(yaw);
    }


        std::vector<Point> SkillMoveTo::find_trajectory(RobotController& robot, Point start, Point goal, bool avoid_ball = true, bool full_field, bool ignore_stop) {
            double minor[2];
            double major[2];
            if (full_field or robot.getRole()  == Robot::goal_keeper) {
                minor[0] = robot.get_world().field.full_dimensions.getResized(-robot.getRadius()).getMinorPoint().getX();
                minor[1] = robot.get_world().field.full_dimensions.getResized(-robot.getRadius()).getMinorPoint().getY();
                major[0] = robot.get_world().field.full_dimensions.getResized(-robot.getRadius()).getMajorPoint().getX();
                major[1] = robot.get_world().field.full_dimensions.getResized(-robot.getRadius()).getMajorPoint().getY();

            } else {
                minor[0] = robot.get_world().field.inside_dimensions.getResized(-robot.getRadius()).getMinorPoint().getX();
                minor[1] = robot.get_world().field.inside_dimensions.getResized(-robot.getRadius()).getMinorPoint().getY();
                major[0] = robot.get_world().field.inside_dimensions.getResized(-robot.getRadius()).getMajorPoint().getX();
                major[1] = robot.get_world().field.inside_dimensions.getResized(-robot.getRadius()).getMajorPoint().getY();
            }
            C_trajectory pf(false, false, 0, 100, 50, 0, minor, major);

            std::vector<Circle> obs_circular = {};
            std::vector<Rectangle> obs_rectangular = {};
            std::vector<TiltedRectangle> obs_tilted = {};
            Rectangle r({0, 0}, {0, 0});
            //rectangle r = field.their_defense_area;
            //obs_rectangular.push_back(r);

            if (!ignore_stop && (robot.get_m_team()->getEvent() == TeamInfo::stop or robot.get_m_team()->getEvent() == TeamInfo::timeout or robot.get_m_team()->getEvent() == TeamInfo::prepareOurKickOff
                or robot.get_m_team()->getEvent() == TeamInfo::prepareTheirKickOff or robot.get_m_team()->getEvent() == TeamInfo::prepareOurPenalty or robot.get_m_team()->getEvent() == TeamInfo::prepareTheirPenalty
                or robot.get_m_team()->getEvent() == TeamInfo::theirballPlacement  or robot.get_m_team()->getEvent() == TeamInfo::theirFreeKick
                or robot.get_m_team()->getEvent() == TeamInfo::runningTheirFreeKick)) {
                Circle c({robot.get_world().ball.getPosition().getX(), robot.get_world().ball.getPosition().getY()}, robot.get_m_team()->getStopDistanceToBall() + robot.getRadius());
                obs_circular.push_back(c);
            }

            if (robot.get_m_team()->getEvent() == TeamInfo::theirballPlacement) {
                auto t = TiltedRectangle({robot.get_world().ball.getPosition().getX(), robot.get_world().ball.getPosition().getY()}, {robot.get_m_team()->getBallPlacementSpot().getX(), robot.get_m_team()->getBallPlacementSpot().getY()}, robot.getRadius() + 500);
                obs_tilted.push_back(t);
                Circle c1({robot.get_m_team()->getBallPlacementSpot().getX(), robot.get_m_team()->getBallPlacementSpot().getY()}, 500 + robot.getRadius());
                obs_circular.push_back(c1);
                Circle c2({robot.get_world().ball.getPosition().getX(), robot.get_world().ball.getPosition().getY()}, 500 + robot.getRadius());
                obs_circular.push_back(c2);
            }

            //add static ball to obstacles according to avoidance radius
            if (avoid_ball) {
                Circle c({robot.get_world().ball.getPosition().getX(), robot.get_world().ball.getPosition().getY()}, robot.get_m_ball_avoidance_radius() + robot.getRadius());
                obs_circular.push_back(c);
            }

            //add static allies to obstacles
            for (int i = 0; i < size(robot.get_world().allies) ; i++) {
                if (!robot.get_world().allies[i].isDetected() || i == robot.getId()) {
                    continue;
                }
                Circle c({robot.get_world().allies[i].getPosition().getX(), robot.get_world().allies[i].getPosition().getY()}, robot.getRadius() + robot.get_world().allies[i].getRadius());
                obs_circular.push_back(c);
            }

            //add static enemies to obstacles
            for (int i = 0; i < size(robot.get_world().enemies) ; i++) {
                if (!robot.get_world().enemies[i].isDetected()) {
                    continue;
                }
                Circle c({robot.get_world().enemies[i].getPosition().getX(), robot.get_world().enemies[i].getPosition().getY()}, robot.getRadius() + robot.get_world().enemies[i].getRadius());
                obs_circular.push_back(c);
            }

            if (!((robot.getRole() == Robot::placer || robot.getRole() == Robot::placeHolder) && robot.get_m_team()->getEvent() == TeamInfo::ourballPlacement)) {
                if (robot.get_m_team()->getAllyRole(robot.getId()) != Robot::goal_keeper) {
                    obs_rectangular.push_back(getRectangle(robot.get_world().field.ourDefenseArea.getResized(robot.getRadius())));
                }
                if (robot.get_m_team()->getEvent() == TeamInfo::ourFreeKick or robot.get_m_team()->getEvent() == TeamInfo::theirFreeKick or robot.get_m_team()->getEvent() == TeamInfo::stop)
                    obs_rectangular.push_back(getRectangle(robot.get_world().field.theirDefenseArea.getResized(robot.getRadius() + 200)));
                else obs_rectangular.push_back(getRectangle(robot.get_world().field.theirDefenseArea.getResized(robot.getRadius())));

            }

            double wall_thickness = robot.get_world().field.goalBarrierThickness;
            AreaRectangular a({0, 0}, {0, 0});
            a = robot.get_world().field.leftFisicalBarrier;
            a.setMajorPoint({a.getMajorPoint().getX(), a.getMinorPoint().getY() + wall_thickness});
            obs_rectangular.push_back(getRectangle(a.getResized(robot.getRadius())));
            a = robot.get_world().field.leftFisicalBarrier;
            a.setMinorPoint({a.getMinorPoint().getX(), a.getMajorPoint().getY() - wall_thickness});
            obs_rectangular.push_back(getRectangle(a.getResized(robot.getRadius())));

            obs_rectangular.push_back(getRectangle(a.getResized(robot.getRadius()/2)));
            a = robot.get_world().field.leftFisicalBarrier;
            a.setMinorPoint({a.getMinorPoint().getX(), a.getMajorPoint().getY() - wall_thickness});
            obs_rectangular.push_back(getRectangle(a.getResized(robot.getRadius()/2)));
            a = robot.get_world().field.leftFisicalBarrier;
            a.setMajorPoint({a.getMinorPoint().getX() + wall_thickness, a.getMajorPoint().getY()});
            a.setMinorPoint({a.getMinorPoint().getX() - 10000, a.getMinorPoint().getY()});
            obs_rectangular.push_back(getRectangle(a.getResized(robot.getRadius())));

            a = robot.get_world().field.rightFisicalBarrier;
            a.setMajorPoint({a.getMajorPoint().getX(), a.getMinorPoint().getY() + wall_thickness});
            obs_rectangular.push_back(getRectangle(a.getResized(robot.getRadius())));
            a = robot.get_world().field.rightFisicalBarrier;
            a.setMinorPoint({a.getMinorPoint().getX(), a.getMajorPoint().getY() - wall_thickness});
            obs_rectangular.push_back(getRectangle(a.getResized(robot.getRadius())));
            obs_rectangular.push_back(getRectangle(a.getResized(robot.getRadius()/2)));
            a = robot.get_world().field.rightFisicalBarrier;
            a.setMinorPoint({a.getMinorPoint().getX(), a.getMajorPoint().getY() - wall_thickness});
            obs_rectangular.push_back(getRectangle(a.getResized(robot.getRadius()/2)));
            a = robot.get_world().field.rightFisicalBarrier;
            a.setMinorPoint({a.getMajorPoint().getX() - wall_thickness, a.getMinorPoint().getY()});
            a.setMajorPoint({a.getMajorPoint().getX() + 10000, a.getMajorPoint().getY()});
            obs_rectangular.push_back(getRectangle(a.getResized(robot.getRadius())));

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
            if (robot.getPosition().getDistanceTo(trajectory[size(trajectory) - 1]) < robot.get_m_static_position_tolarance()) {
                robot.set_mtarget_vel({0, 0});
                robot.set_mtarget_vyaw(0);
                robot.set_mlast_target_vel({0, 0});
                robot.set_m_last_delta_vx(0);
                robot.set_m_last_delta_vy(0);
                robot.set_m_i_vx(0);
                robot.set_m_i_vy(0);
                if (robot.isStopped()) {
                    robot.setPositioned(true);
                    robot.get_m_team()->setPositioned(robot.getId(), true);
                }
                return;
            }
        }

        robot.setPositioned(false);
        robot.get_m_team()->setPositioned(robot.getId(), false);
        Vector2d v_vet;
        if (std::size(trajectory) > 1) {
            v_vet = motion_planner(robot, trajectory);
        } else {
            if (robot.getPosition().getDistanceTo(Point(0, 0)) > robot.get_m_static_position_tolarance()) {
            v_vet = Vector2d({0, 0}, robot.getPosition()).getNormalized(robot.get_m_vxy_min());
            } else {
                v_vet = {0, 0};
            }
        }
        v_vet = motion_control(robot, v_vet, -robot.getYaw());

        robot.set_mtarget_vel(v_vet);
        robot.set_mkicker_x(0);
    }

    void SkillMoveTo::act(RobotController& robot) {
    	std::cout << "Dummy method" << std::endl;
    }
} // skills