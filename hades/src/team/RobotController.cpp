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
    mTerminate = false;
    mOffline_counter = 0;
    loadCalibration();
    std::cout << "iniciado " << id << std::endl;
    std::thread t(&RobotController::loop, this);
    t.detach();
}

void RobotController::stop() {
    mTeam->setRobotActive(id, false);
    mTeam->setAllyRole(id, unknown);
    mTerminate = true;
    skills::SkillStop stop;
    stop.act(*this);
    publish();
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
        //mWorld.field.inside_dimensions = AreaRectangular({0, 0}, {2250, 1250});
        //mWorld.field.full_dimensions = AreaRectangular({0, 0}, {2000, 1000});
        //mWorld.field.ourGoal = LineSegment(Point(500, 0), Point(500, 1200));
        //mWorld.field.ourDefenseArea = AreaRectangular({0, 0}, {300, 1000});
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

void RobotController::doubleTouchHandler() {
    if (will_double_touch && mWorld.ball.isMoving()) {
        Vector2d diff(mWorld.ball.getVelocity().getNormalized(1).getX() - getVelocity().getNormalized(1).getX(), mWorld.ball.getVelocity().getNormalized(1).getY() - getVelocity().getNormalized(1).getY());
        if (diff.getNorm() < 0.2) double_touch_waiting = true;
        std::cout << diff.getNorm() << std::endl;
        will_double_touch = false;
    }
    if ((double_touch_waiting && mWorld.ball.isStopped())) {double_touch = true; double_touch_waiting = false;}
    if ((double_touch && mWorld.ball.isMoving()) or mTeam->getEvent() == TeamInfo::stop) {double_touch = false; double_touch_waiting = false;}
}

bool RobotController::isActive() {
    return this->active;
}


void RobotController::select_behavior() {
    //TODO roles
    //role reset
    if (lastRole != mTeam->getAllyRole(id)) {
        lastRole = mTeam->getAllyRole(id);
        mState = 0;
    }
    try {
        mTeam->role_map[mTeam->getAllyRole(id)]->act(*this);
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
        mKD_ang = 0.5;
        mKI_ang = 0;
        kickDistance = 2000;
        mStatic_position_tolarance = radius/8;
        mDynamic_position_tolarance = radius/8;
        mStatic_angle_tolarance = 0.01;
        mVxy_min = 0.4;
        kicker = true;
    }
    if (han.new_tartarus.ssl_vision) {
        mKP_ang = 0.35;
        mKD_ang = 0;
        mKI_ang = 0;
        kickDistance = 500;
        mStatic_position_tolarance = radius/4;
        mDynamic_position_tolarance = radius/2;
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
        if (mTeam->getColor() == TeamInfo::blue) {
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
            mWorld.allies[rb_id].setRole(mTeam->getEnemyRole(rb_id));
        }
    }


    for (auto yellow_robot : han.new_vision.robots_yellow) {
        if (!yellow_robot.detected) continue;
        if (mTeam->getColor() == TeamInfo::yellow) {
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

    if (mTeam->getOurSide() == TeamInfo::left) {
        mWorld.field.ourGoal = leftGoal;
        mWorld.field.theirGoal = rightGoal;
        mWorld.field.ourDefenseArea = leftDefenseArea;
        mWorld.field.theirDefenseArea = rightDefenseArea;
    }
    if (mTeam->getOurSide() == TeamInfo::right) {
        mWorld.field.ourGoal = rightGoal;
        mWorld.field.theirGoal = leftGoal;
        mWorld.field.ourDefenseArea = rightDefenseArea;
        mWorld.field.theirDefenseArea = leftDefenseArea;
    }

    if (han.new_tartarus.half_field != 0) {
            //TODO implementar
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


#include "RobotController.h"

// --- Getters ---
bool RobotController::is_active() const {
    std::lock_guard<std::mutex> lock(mtx);
    return active;
}

double RobotController::get_m_ball_avoidance_radius() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mBall_avoidance_radius;
}

double RobotController::get_mtarget_yaw() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mtarget_yaw;
}

Vector2d RobotController::get_mtarget_vel() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mtarget_vel;
}

double RobotController::get_mtarget_vyaw() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mtarget_vyaw;
}

Vector2d RobotController::get_mlast_target_vel() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mlast_target_vel;
}

double RobotController::get_mkicker_x() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mkicker_x;
}

double RobotController::get_mkicker_z() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mkicker_z;
}

double RobotController::get_mdribbler() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mdribbler;
}

TeamInfo* RobotController::get_m_team() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mTeam;
}

int RobotController::get_m_state() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mState;
}

double RobotController::get_m_delta_time() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mDelta_time;
}

double RobotController::get_m_timer() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mTimer;
}

std::vector<Point> RobotController::get_m_current_trajectory() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mCurrent_trajectory;
}

int RobotController::get_m_offline_counter() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mOffline_counter;
}

int RobotController::get_m_max_offline_counter() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mMax_offline_counter;
}

bool RobotController::is_m_terminate() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mTerminate;
}

double RobotController::get_m_vxy_max() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mVxy_max;
}

double RobotController::get_m_vxy_min() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mVxy_min;
}

double RobotController::get_m_a_xy_max() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mA_xy_max;
}

double RobotController::get_m_a_xy_brake() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mA_xy_brake;
}

double RobotController::get_m_vyaw_max() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mVyaw_max;
}

double RobotController::get_m_vyaw_min() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mVyaw_min;
}

double RobotController::get_m_a_ang_max() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mA_ang_max;
}

double RobotController::get_m_kicker_x_max() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mKicker_x_max;
}

double RobotController::get_m_kicker_x_min() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mKicker_x_min;
}

double RobotController::get_m_kicker_z_max() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mKicker_z_max;
}

double RobotController::get_m_kicker_z_min() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mKicker_z_min;
}

double RobotController::get_m_dribbler_max() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mDribbler_max;
}

double RobotController::get_m_dribbler_min() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mDribbler_min;
}

double RobotController::get_m_static_position_tolarance() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mStatic_position_tolarance;
}

double RobotController::get_m_dynamic_position_tolarance() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mDynamic_position_tolarance;
}

double RobotController::get_m_static_angle_tolarance() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mStatic_angle_tolarance;
}

double RobotController::get_m_kp_ang() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mKP_ang;
}

double RobotController::get_m_ki_ang() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mKI_ang;
}

double RobotController::get_m_kd_ang() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mKD_ang;
}

double RobotController::get_m_i_ang() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mI_ang;
}

double RobotController::get_m_i_ang_max() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mI_ang_max;
}

double RobotController::get_m_last_delta_vyaw() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mLast_delta_vyaw;
}

double RobotController::get_m_i_vx() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mI_vx;
}

double RobotController::get_m_i_vy() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mI_vy;
}

double RobotController::get_m_i_vxy_max() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mI_vxy_max;
}

double RobotController::get_m_last_delta_vx() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mLast_delta_vx;
}

double RobotController::get_m_last_delta_vy() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mLast_delta_vy;
}

double RobotController::get_m_kp_vxy() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mKP_vxy;
}

double RobotController::get_m_ki_vxy() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mKI_vxy;
}

double RobotController::get_m_kd_vxy() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mKD_vxy;
}

bool RobotController::is_will_double_touch() const {
    std::lock_guard<std::mutex> lock(mtx);
    return will_double_touch;
}

bool RobotController::is_double_touch() const {
    std::lock_guard<std::mutex> lock(mtx);
    return double_touch;
}

bool RobotController::is_double_touch_waiting() const {
    std::lock_guard<std::mutex> lock(mtx);
    return double_touch_waiting;
}

WorldModel& RobotController::get_world() {
    std::lock_guard<std::mutex> lock(mtx);
    return mWorld;
}

int64_t RobotController::get_m_last_time_stamp() const {
    std::lock_guard<std::mutex> lock(mtx);
    return mLast_time_stamp;
}

RobotController::role RobotController::get_last_role() const {
    std::lock_guard<std::mutex> lock(mtx);
    return lastRole;
}

// --- Setters ---
void RobotController::set_active(bool value) {
    std::lock_guard<std::mutex> lock(mtx);
    active = value;
}

void RobotController::set_m_ball_avoidance_radius(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mBall_avoidance_radius = value;
}

void RobotController::set_mtarget_yaw(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mtarget_yaw = value;
}

void RobotController::set_mtarget_vel(const Vector2d &value) {
    std::lock_guard<std::mutex> lock(mtx);
    mtarget_vel = value;
}

void RobotController::set_mtarget_vyaw(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mtarget_vyaw = value;
}

void RobotController::set_mlast_target_vel(const Vector2d &value) {
    std::lock_guard<std::mutex> lock(mtx);
    mlast_target_vel = value;
}

void RobotController::set_mkicker_x(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mkicker_x = value;
}

void RobotController::set_mkicker_z(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mkicker_z = value;
}

void RobotController::set_mdribbler(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mdribbler = value;
}

void RobotController::set_m_team(TeamInfo* value) {
    std::lock_guard<std::mutex> lock(mtx);
    mTeam = value;
}

void RobotController::set_m_state(int value) {
    std::lock_guard<std::mutex> lock(mtx);
    mState = value;
}

void RobotController::set_m_delta_time(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mDelta_time = value;
}

void RobotController::set_m_timer(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mTimer = value;
}

void RobotController::set_m_current_trajectory(const std::vector<Point> &value) {
    std::lock_guard<std::mutex> lock(mtx);
    mCurrent_trajectory = value;
}

void RobotController::set_m_offline_counter(int value) {
    std::lock_guard<std::mutex> lock(mtx);
    mOffline_counter = value;
}

void RobotController::set_m_max_offline_counter(int value) {
    std::lock_guard<std::mutex> lock(mtx);
    mMax_offline_counter = value;
}

void RobotController::set_m_terminate(bool value) {
    std::lock_guard<std::mutex> lock(mtx);
    mTerminate = value;
}

void RobotController::set_m_vxy_max(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mVxy_max = value;
}

void RobotController::set_m_vxy_min(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mVxy_min = value;
}

void RobotController::set_m_a_xy_max(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mA_xy_max = value;
}

void RobotController::set_m_a_xy_brake(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mA_xy_brake = value;
}

void RobotController::set_m_vyaw_max(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mVyaw_max = value;
}

void RobotController::set_m_vyaw_min(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mVyaw_min = value;
}

void RobotController::set_m_a_ang_max(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mA_ang_max = value;
}

void RobotController::set_m_kicker_x_max(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mKicker_x_max = value;
}

void RobotController::set_m_kicker_x_min(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mKicker_x_min = value;
}

void RobotController::set_m_kicker_z_max(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mKicker_z_max = value;
}

void RobotController::set_m_kicker_z_min(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mKicker_z_min = value;
}

void RobotController::set_m_dribbler_max(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mDribbler_max = value;
}

void RobotController::set_m_dribbler_min(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mDribbler_min = value;
}

void RobotController::set_m_static_position_tolarance(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mStatic_position_tolarance = value;
}

void RobotController::set_m_dynamic_position_tolarance(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mDynamic_position_tolarance = value;
}

void RobotController::set_m_static_angle_tolarance(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mStatic_angle_tolarance = value;
}

void RobotController::set_m_kp_ang(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mKP_ang = value;
}

void RobotController::set_m_ki_ang(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mKI_ang = value;
}

void RobotController::set_m_kd_ang(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mKD_ang = value;
}

void RobotController::set_m_i_ang(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mI_ang = value;
}

void RobotController::set_m_i_ang_max(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mI_ang_max = value;
}

void RobotController::set_m_last_delta_vyaw(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mLast_delta_vyaw = value;
}

void RobotController::set_m_i_vx(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mI_vx = value;
}

void RobotController::set_m_i_vy(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mI_vy = value;
}

void RobotController::set_m_i_vxy_max(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mI_vxy_max = value;
}

void RobotController::set_m_last_delta_vx(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mLast_delta_vx = value;
}

void RobotController::set_m_last_delta_vy(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mLast_delta_vy = value;
}

void RobotController::set_m_kp_vxy(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mKP_vxy = value;
}

void RobotController::set_m_ki_vxy(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mKI_vxy = value;
}

void RobotController::set_m_kd_vxy(double value) {
    std::lock_guard<std::mutex> lock(mtx);
    mKD_vxy = value;
}

void RobotController::set_will_double_touch(bool value) {
    std::lock_guard<std::mutex> lock(mtx);
    will_double_touch = value;
}

void RobotController::set_double_touch(bool value) {
    std::lock_guard<std::mutex> lock(mtx);
    double_touch = value;
}

void RobotController::set_double_touch_waiting(bool value) {
    std::lock_guard<std::mutex> lock(mtx);
    double_touch_waiting = value;
}

void RobotController::set_world(WorldModel& value) {
    std::lock_guard<std::mutex> lock(mtx);
    mWorld = value;
}

void RobotController::set_m_last_time_stamp(int64_t value) {
    std::lock_guard<std::mutex> lock(mtx);
    mLast_time_stamp = value;
}

void RobotController::set_last_role(role value) {
    std::lock_guard<std::mutex> lock(mtx);
    lastRole = value;
}