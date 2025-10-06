//
// Created by caiom on 4/4/2025.
//

#include "RobotController.h"
#include <iostream>
#include <thread>
#include "../c_trajectory/C_trajectory.h"
#include <math.h>
#include "../include/handlers.hpp"
#include <chrono>
#include <numeric>
#include <set>
#include <unordered_set>
#include "TeamInfo.h"
#include "geometry/LineSegment.h"


void RobotController::start(TeamInfo* team) {
    han.new_ia.robots[id].kick_speed_x = 3;
    setActive(true);
    this->mTeam = team;
    mTeam->num_of_active_robots++;
    mTerminate = false;
    mOffline_counter = 0;
    loadCalibration();
    std::cout << "iniciado " << id << std::endl;
    std::thread t(&RobotController::loop, this);
    t.detach();
}

void RobotController::stop() {
    mTeam->active_robots[id] = false;
    mTeam->roles[id] = unknown;
    mTerminate = true;
    skills::SkillStop stop;
    stop.act(*this);
    publish();
    mTeam->num_of_active_robots--;
}

void RobotController::loop() {
    auto t0 = std::chrono::steady_clock::now();

    while (not mTerminate) {
        //mTeam->role_map[support]->act(*this);
        if (mLast_time_stamp == han.new_ia.timestamp) {
            continue;
        }
        auto t1 = std::chrono::steady_clock::now();

        receive_vision();
        receive_field_geometry();
        receive_config();
        mWorld.field.inside_dimensions = AreaRectangular({0, 0}, {2250, 1250});
        mWorld.field.full_dimensions = AreaRectangular({0, 0}, {2000, 1000});
        //mWorld.field.ourGoal = LineSegment(Point(500, 0), Point(500, 1200));
        mWorld.field.ourDefenseArea = AreaRectangular({0, 0}, {300, 1000});
        //mWorld.field.theirGoal = LineSegment(Point(1500, 667), Point(1500, 334));
        try {
            select_behavior();
        } catch (std::runtime_error& e) {
            std::cout << "error" << e.what() << std::endl;
        }
        check_connection();
        publish();
        std::chrono::duration<double> delta = t1 - t0;
        t0 = std::chrono::steady_clock::now();
        mDelta_time = delta.count();
    }
    std::cout << "Encerrado " << id << std::endl;
}

void RobotController::setActive(bool active) {
    this->active = active;
}

bool RobotController::isActive() {
    return this->active;
}


void RobotController::select_behavior() {
    //TODO roles
    //role reset
    if (lastRole != mTeam->roles[id]) {
        lastRole = mTeam->roles[id];
        mState = 0;
    }
    try {
        mTeam->role_map[mTeam->roles[id]]->act(*this);
    }
    catch (...) {
        mTeam->role_map[Robot::halted]->act(*this);
        //when role inst on role_map
    }
}

void RobotController::check_connection() {
    if (!isDetected()) {
        mOffline_counter++;
    } else {
        mOffline_counter = 0;
    }

    if (mOffline_counter >= mMax_offline_counter) {
        stop();
    }
}

void RobotController::receive_config() {
    if (!han.new_tartarus.ssl_vision) {
        mKP_ang = 1;
        mKD_ang = 1;
        mKI_ang = 1;
        kickDistance = 2000;
        mStatic_position_tolarance = mRadius/8;
        mDynamic_position_tolarance = mRadius/8;
        mStatic_angle_tolarance = 0.01;
        mVxy_min = 0.4;
    }
    if (han.new_tartarus.ssl_vision) {
        mKP_ang = 0.35;
        mKD_ang = 0;
        mKI_ang = 0;
        kickDistance = 500;
        mStatic_position_tolarance = mRadius/4;
        mDynamic_position_tolarance = mRadius/2;
        mStatic_angle_tolarance = 0.01;
        mVxy_min = 0.1;
        mVxy_max = 0.7;
        mVyaw_min = 0.25;
        mVyaw_max = 3;
    }
}


void RobotController::receive_vision() {
    std::unordered_set<int> allies_detected = {};
    std::unordered_set<int> enemies_detected = {};
    for (auto blue_robot : han.new_vision.robots_blue) {
        if (!blue_robot.detected) continue;
        if (mTeam->color == TeamInfo::blue) {
            int rb_id = blue_robot.robot_id;
            double new_yaw = blue_robot.orientation;
            if (new_yaw < 0) new_yaw += 2*M_PI;
            if (mDelta_time > 0) {
                //TODO logica quebrada na troca de estrutura, refazer.
                Vector2d v = {((blue_robot.position_x - mWorld.allies[rb_id].getPosition().getX())/(mDelta_time*1000)), ((blue_robot.position_y - mWorld.allies[rb_id].getPosition().getY())/(mDelta_time*1000))};
                double vyaw = (new_yaw - mWorld.allies[rb_id].getYaw())/(mDelta_time*1000);
                mWorld.allies[rb_id].setVyaw(vyaw);
                mWorld.allies[rb_id].setVelocity(v);
            }
            mWorld.allies[rb_id].setYaw(new_yaw);
            mWorld.allies[rb_id].setPosition({blue_robot.position_x, blue_robot.position_y});
            mWorld.allies[rb_id].setAlly(true);
            allies_detected.insert(rb_id);
        }
        else {
            int rb_id = blue_robot.robot_id;
            double new_yaw = blue_robot.orientation;
            if (new_yaw < 0) new_yaw += 2*M_PI;
            if (mDelta_time > 0) {
                //TODO logica quebrada na troca de estrutura, refazer.
                Vector2d v = {((blue_robot.position_x - mWorld.enemies[rb_id].getPosition().getX())/(mDelta_time*1000)), ((blue_robot.position_y - mWorld.enemies[rb_id].getPosition().getY())/(mDelta_time*1000))};
                double vyaw = (new_yaw - mWorld.enemies[rb_id].getYaw())/(mDelta_time*1000);
                mWorld.enemies[rb_id].setVyaw(vyaw);
                mWorld.enemies[rb_id].setVelocity(v);
            }
            mWorld.enemies[rb_id].setYaw(new_yaw);
            mWorld.enemies[rb_id].setPosition({blue_robot.position_x, blue_robot.position_y});
            enemies_detected.insert(rb_id);
            mWorld.enemies[rb_id].setAlly(false);
            mWorld.allies[rb_id].setRole(mTeam->enemy_roles[rb_id]);
        }
    }


    for (auto yellow_robot : han.new_vision.robots_yellow) {
        if (!yellow_robot.detected) continue;
        if (mTeam->color == TeamInfo::yellow) {
            int rb_id = yellow_robot.robot_id;
            double new_yaw = yellow_robot.orientation;
            if (new_yaw < 0) new_yaw += 2*M_PI;
            if (mDelta_time > 0) {
                //TODO logica quebrada na troca de estrutura, refazer.
                Vector2d v = {((yellow_robot.position_x - mWorld.allies[rb_id].getPosition().getX())/(mDelta_time*1000)), ((yellow_robot.position_y - mWorld.allies[rb_id].getPosition().getY())/(mDelta_time*1000))};
                double vyaw = (new_yaw - mWorld.allies[rb_id].getYaw())/(mDelta_time*1000);
                mWorld.allies[rb_id].setVyaw(vyaw);
                mWorld.allies[rb_id].setVelocity(v);
            }
            mWorld.allies[rb_id].setYaw(new_yaw);
            mWorld.allies[rb_id].setPosition({yellow_robot.position_x, yellow_robot.position_y});
            mWorld.allies[rb_id].setAlly(true);
            allies_detected.insert(rb_id);
        }
        else {
            int rb_id = yellow_robot.robot_id;

            double new_yaw = yellow_robot.orientation;
            if (new_yaw < 0) new_yaw += 2*M_PI;
            if (mDelta_time > 0) {
                //TODO logica quebrada na troca de estrutura, refazer.
                Vector2d v = {((yellow_robot.position_x - mWorld.enemies[rb_id].getPosition().getX())/(mDelta_time*1000)), ((yellow_robot.position_y - mWorld.enemies[rb_id].getPosition().getY())/(mDelta_time*1000))};
                double vyaw = (new_yaw - mWorld.enemies[rb_id].getYaw())/(mDelta_time*1000);
                mWorld.enemies[rb_id].setVyaw(vyaw);
                mWorld.enemies[rb_id].setVelocity(v);
            }
            mWorld.enemies[rb_id].setYaw(new_yaw);
            mWorld.enemies[rb_id].setPosition({yellow_robot.position_x, yellow_robot.position_y});
            mWorld.enemies[rb_id].setAlly(false);
            enemies_detected.insert(rb_id);
        }
    }

    for (int i = 0; i < size(mWorld.allies); i++) {
        if (allies_detected.find(i) != allies_detected.end()) mWorld.allies[i].setDetected(true);
        else mWorld.allies[i].setDetected(false);
        }

    for (int i = 0; i < size(mWorld.enemies); i++) {
        if (enemies_detected.find(i) != enemies_detected.end()) mWorld.enemies[i].setDetected(true);
        else mWorld.enemies[i].setDetected(false);
    }

    if (mDelta_time != 0) {
        mWorld.ball.setVelocity({(han.new_vision.balls.position_x - mWorld.ball.getPosition().getX())/(mDelta_time*1000), (han.new_vision.balls.position_y - mWorld.ball.getPosition().getY())/(mDelta_time*1000)});
    }

    mWorld.ball.setPosition({han.new_vision.balls.position_x, han.new_vision.balls.position_y});
    VisibilityGraph graph;
    for (Robot enemy : mWorld.enemies) {
        if (!enemy.isDetected()) continue;
        graph.addShadow(CircularShadow(mWorld.ball.getPosition(), {enemy.getPosition(), enemy.getRadius()}));
    }

    AreaRectangular a({0, 0}, {0, 0});
    double wall_thickness = mWorld.field.goalBarrierThickness;
    a = mWorld.field.leftFisicalBarrier;
    a.setMajorPoint({a.getMajorPoint().getX(), a.getMinorPoint().getY() + wall_thickness});
    graph.addShadow(RectangularShadow(mWorld.ball.getPosition(), a));
    a = mWorld.field.leftFisicalBarrier;
    a.setMinorPoint({a.getMinorPoint().getX(), a.getMajorPoint().getY() - wall_thickness});
    graph.addShadow(RectangularShadow(mWorld.ball.getPosition(), a));
    a = mWorld.field.leftFisicalBarrier;
    a.setMajorPoint({a.getMinorPoint().getX() + wall_thickness, a.getMajorPoint().getY()});
    graph.addShadow(RectangularShadow(mWorld.ball.getPosition(), a));

    a = mWorld.field.rightFisicalBarrier;
    a.setMajorPoint({a.getMajorPoint().getX(), a.getMinorPoint().getY() + wall_thickness});
    graph.addShadow(RectangularShadow(mWorld.ball.getPosition(), a));
    a = mWorld.field.rightFisicalBarrier;
    a.setMinorPoint({a.getMinorPoint().getX(), a.getMajorPoint().getY() - wall_thickness});
    graph.addShadow(RectangularShadow(mWorld.ball.getPosition(), a));
    a = mWorld.field.rightFisicalBarrier;
    a.setMinorPoint({a.getMajorPoint().getX() - wall_thickness, a.getMinorPoint().getY()});
    graph.addShadow(RectangularShadow(mWorld.ball.getPosition(), a));

    mWorld.ball.setVisibilityGraph(graph);

    mLast_time_stamp = han.new_vision.timestamp;

    if (mWorld.allies.size() >= id) {
        setPosition(mWorld.allies[id].getPosition());
        setYaw(mWorld.allies[id].getYaw());
        setVelocity(mWorld.allies[id].getVelocity());
        setVyaw(mWorld.allies[id].getVyaw());
        setDetected(mWorld.allies[id].isDetected());
    }
}

void RobotController::receive_field_geometry() {
    //TODO implementar urgente
    mWorld.field.inside_dimensions.setMinorPoint({static_cast<double>(-han.new_vision.field.field_length/2), static_cast<double>(-han.new_vision.field.field_width/2)});
    mWorld.field.inside_dimensions.setMajorPoint({static_cast<double>(han.new_vision.field.field_length/2), static_cast<double>(han.new_vision.field.field_width/2)});

    AreaRectangular leftDefenseArea = {{-han.new_vision.field.field_length/2, -han.new_vision.field.defense_area_width/2},{-han.new_vision.field.field_length/2 + han.new_vision.field.defense_area_height, han.new_vision.field.defense_area_width/2}};
    AreaRectangular rightDefenseArea = {{han.new_vision.field.field_length/2 - han.new_vision.field.defense_area_height, -han.new_vision.field.defense_area_width/2}, {han.new_vision.field.field_length/2, han.new_vision.field.defense_area_width/2}};

    LineSegment leftGoal = {Point(-han.new_vision.field.field_length/2, -han.new_vision.field.goal_width/2), Point(-han.new_vision.field.field_length/2 , han.new_vision.field.goal_width/2)};
    LineSegment rightGoal = {Point(han.new_vision.field.field_length/2, -han.new_vision.field.goal_width/2), Point(han.new_vision.field.field_length/2 , han.new_vision.field.goal_width/2)};

    AreaRectangular leftFisicalBarrier = {{leftGoal.getStart().getX() - han.new_vision.field.goal_height, leftGoal.getStart().getY()}, leftGoal.getEnd()};
    AreaRectangular rightFisicalBarrier = {rightGoal.getStart(), {rightGoal.getEnd().getX() + han.new_vision.field.goal_height, rightGoal.getEnd().getY()}};
    mWorld.field.leftFisicalBarrier = leftFisicalBarrier;
    mWorld.field.rightFisicalBarrier = rightFisicalBarrier;

    if (mTeam->our_side == TeamInfo::left) {
        mWorld.field.ourGoal = leftGoal;
        mWorld.field.theirGoal = rightGoal;
        mWorld.field.ourDefenseArea = leftDefenseArea;
        mWorld.field.theirDefenseArea = rightDefenseArea;
    }
    if (mTeam->our_side == TeamInfo::right) {
        mWorld.field.ourGoal = rightGoal;
        mWorld.field.theirGoal = leftGoal;
        mWorld.field.ourDefenseArea = rightDefenseArea;
        mWorld.field.theirDefenseArea = leftDefenseArea;
    }
}

void RobotController::loadCalibration() {
    //TODO
}


void RobotController::publish() {
    han.new_ia.robots[id].id = id;
    if (han.new_tartarus.ssl_vision) {
        mtarget_vel = mtarget_vel.getRotated(3.14156/2);
        han.new_ia.robots[id].vel_normal = mtarget_vel.getY();
        han.new_ia.robots[id].vel_tang = mtarget_vel.getX();
        han.new_ia.robots[id].vel_ang = static_cast<float>(-mtarget_vyaw);
    } else {
        han.new_ia.robots[id].vel_normal = mtarget_vel.getY();
        han.new_ia.robots[id].vel_tang = mtarget_vel.getX();
        han.new_ia.robots[id].vel_ang = static_cast<float>(mtarget_vyaw);
    }
    if (mkicker_x != 0) {
        han.new_ia.robots[id].kick = true;
        han.new_ia.robots[id].kick_speed_x = mkicker_x;
    } else {
        han.new_ia.robots[id].kick = false;
        han.new_ia.robots[id].kick_speed_x = mkicker_x;
    }
    han.lc->publish("IA", &han.new_ia);
}
