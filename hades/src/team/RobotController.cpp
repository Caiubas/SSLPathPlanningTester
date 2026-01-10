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
        if (kicker_timer > 0) {
            kicker_timer -= mDelta_time;
        }
        try {
            if (han.new_tartarus.debug_mode) debug_mode();
            else if (technical_challenge) {do_technical_challenge();}
            else select_behavior();

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

void RobotController::do_technical_challenge() {
    skills::SkillMoveTo moveTo;
    skills::SkillStop stop;
    skills::SkillTurnTo turnTo;
    if (mTeam->getEvent() == TeamInfo::theirFreeKick) {
        if (getId() == 0) {
            moveTo.act(*this, Point(2100, 300), true);
            turnTo.act(*this, Point(2100, 600));
        } else if (getId() == 1) {
            moveTo.act(*this, Point(1650, 900), true);
            turnTo.act(*this, Point(1650, 1100));
        } else if (getId() == 2) {
            moveTo.act(*this, Point(1850, 900), true);
            turnTo.act(*this, Point(1850, 1100));
        }
        else {
            stop.act(*this);
        }
    }
    else if (mTeam->getEvent() == TeamInfo::ourFreeKick) {
        if (getId() == 0) {
            moveTo.act(*this, Point(-1850, 1420), true);
            turnTo.act(*this, Point(-1850, 0));
        } else if (getId() == 1) {
            moveTo.act(*this, Point(150, 700), true);
            turnTo.act(*this, Point(0, 700));
        } else if (getId() == 2) {
            moveTo.act(*this, Point(-590, -600), true);
            turnTo.act(*this, Point(-700, -600));
        }
        else {
            stop.act(*this);
        }
    }
    else if (mTeam->getEvent() == TeamInfo::ourKickOff || mTeam->getEvent() == TeamInfo::prepareOurKickOff) {
        if (getId() == 0) {
            moveTo.act(*this, Point(250, 0), true);
            turnTo.act(*this, Point(0, 0));
        } else if (getId() == 1) {
            moveTo.act(*this, Point(250, 700), true);
            turnTo.act(*this, Point(0, 700));
        } else if (getId() == 2) {
            moveTo.act(*this, Point(2100, 0), true);
            turnTo.act(*this, Point(0, 0));
        }
        else {
            stop.act(*this);
        }
    }
    else if (mTeam->getEvent() == TeamInfo::ourPenalty || mTeam->getEvent() == TeamInfo::prepareOurPenalty) {
        if (getId() == 0) {
            moveTo.act(*this, Point(310, 800), true);
            turnTo.act(*this, Point(0, 800));
        } else if (getId() == 1) {
            moveTo.act(*this, Point(-250, 0), true);
            turnTo.act(*this, Point(-750, 0));
        } else if (getId() == 2) {
            moveTo.act(*this, Point(310, -800), true);
            turnTo.act(*this, Point(0, -800));
        }
        else {
            stop.act(*this);
        }
    }
    else {
        stop.act(*this);
    }
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

void RobotController::debug_mode() {
    if (mTeam->getEvent() == TeamInfo::halt) {
        //skills::SkillStop stop;
        //stop.act(*this);
        //return;
    }
    setRole(unknown);
    skills::SkillStop stop;
    stop.act(*this);

    enum Skill {
        UNKNOWN = 0,
        CUSHION = 1,
        KICK = 2,
        MOVE_TO = 3,
        STOP = 4,
        TURN_TO = 5
    };

    Robot::role r = static_cast<Robot::role>(han.new_tartarus.robots[getId()].role);
    switch (r) {
        case unknown:
            setRole(unknown);
        break;

        case goal_keeper:
            setRole(goal_keeper);
            mTeam->setAllyRole(getId(), goal_keeper);
        break;

        case striker:
            setRole(striker);
        break;

        case support:
            setRole(support);
        break;

        case defender:
            setRole(defender);
        break;

        case halted:
            setRole(halted);
        break;

        case kickoff_kicker:
            setRole(kickoff_kicker);
        break;

        case kickoff_goal_keeper:
            setRole(kickoff_goal_keeper);
        break;

        case kickoff_support:
            setRole(kickoff_support);
        break;

        case marker:
            setRole(marker);
        break;

        case debug_circular_trajectory:
            setRole(debug_circular_trajectory);
        break;

        case debug_squared_trajectory:
            setRole(debug_squared_trajectory);
        break;

        case retaker:
            setRole(retaker);
        break;

        case penaltier:
            setRole(penaltier);
        break;

        case freeKicker:
            setRole(freeKicker);
        break;

        case watcher:
            setRole(watcher);
        break;

        case placeHolder:
            setRole(placeHolder);
        break;

        case placer:
            setRole(placer);
        break;

        default:
            setRole(unknown);
        break;
    }
    if (r != unknown) {
        mTeam->role_map[getRole()]->act(*this);
    }

    if (r == unknown) {
        skills::SkillCushion cushion;
        skills::SkillKick kick;
        skills::SkillStop stop;
        skills::SkillMoveTo moveTo;
        skills::SkillPushBall pushBall;
        skills::SkillTurnTo turnTo;
        Skill s = static_cast<Skill>(han.new_tartarus.robots[getId()].skill);
        switch (s) {
            case UNKNOWN:
                stop.act(*this);
            break;

            case CUSHION:
                cushion.act(*this);
            break;

            case KICK:
                kick.act(*this);
            break;

            case MOVE_TO:
                moveTo.act(*this, Point(han.new_tartarus.robots[getId()].move_to_x, han.new_tartarus.robots[getId()].move_to_y), true);
            break;

            case STOP:
                stop.act(*this);
            break;

            case TURN_TO:
                turnTo.act(*this, Point(han.new_tartarus.robots[getId()].turn_to_x, han.new_tartarus.robots[getId()].turn_to_y));
            break;

            default:
                stop.act(*this);
            break;
        }

    }
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
        mKD_ang = 0;
        mKI_ang = 0;
        kickDistance = 2000;
        mStatic_position_tolarance = radius/8;
        mDynamic_position_tolarance = radius/8;
        mStatic_angle_tolarance = 0.01;
        mVyaw_min = 1;
        mVxy_min = 0.1;
    }
    if (han.new_tartarus.ssl_vision) {
        mKP_ang = 0.35;
        mKD_ang = 0;
        mKI_ang = 0;
        kickDistance = 300000;
        mStatic_position_tolarance = radius/4;
        mDynamic_position_tolarance = radius/2;
        mStatic_angle_tolarance = 0.1;
        mVxy_min = 0.1;
        mVxy_max = 0.8;
        mVyaw_min = 0.25;
        mVyaw_max = 3;
    }


    kicker = han.new_tartarus.robots[getId()].has_kicker;
<<<<<<< HEAD
    if (getId() == 2) kicker = true;
    if (getId() == 1) kicker = false;
    if (kicker) kickDistance = 4000;
    else kickDistance = 1000;
    //if (getId() == 2) std::cout << mWorld.ball.isMoving() << std::endl;

    //if (getId() == 2) std::cout << kicker << " " << kickDistance <<  " " << mWorld.field.theirGoal.getMiddle().getDistanceTo({0, 0}) << std::endl;
=======
    kicker = true;
    if (kicker) kickDistance = 2000;
    else kickDistance = 750;
>>>>>>> 9fa43e16e1cb4304d698b0bfbfc19d3511c7cccf
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
        graph.addShadow(CircularShadow(mWorld.ball.getPosition(), {enemy.getPosition(), enemy.getRadius()*1.2}));
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
    double correction = 100;
    if (!han.new_tartarus.half_field) {
        mWorld.field.full_dimensions.setMinorPoint({static_cast<double>(-han.new_vision.field.field_length/2 - han.new_vision.field.boundary_width + correction), static_cast<double>(-han.new_vision.field.field_width/2 - han.new_vision.field.boundary_width + correction)});
        mWorld.field.full_dimensions.setMajorPoint({static_cast<double>(han.new_vision.field.field_length/2 + han.new_vision.field.boundary_width - correction), static_cast<double>(han.new_vision.field.field_width/2 + han.new_vision.field.boundary_width - correction)});
    } else {
        if (han.new_tartarus.right_field) {
            mWorld.field.full_dimensions.setMinorPoint({static_cast<double>(0.0), static_cast<double>(-han.new_vision.field.field_width/2 - han.new_vision.field.boundary_width + correction)});
            mWorld.field.full_dimensions.setMajorPoint({static_cast<double>(han.new_vision.field.field_length/2 + han.new_vision.field.boundary_width - correction), static_cast<double>(han.new_vision.field.field_width/2 + han.new_vision.field.boundary_width - correction)});
        } else {
            mWorld.field.full_dimensions.setMinorPoint({static_cast<double>(-han.new_vision.field.field_length/2 - han.new_vision.field.boundary_width + correction), static_cast<double>(-han.new_vision.field.field_width/2 - han.new_vision.field.boundary_width + correction)});
            mWorld.field.full_dimensions.setMajorPoint({static_cast<double>(han.new_vision.field.field_length/2 + han.new_vision.field.boundary_width - correction), static_cast<double>(0.0)});
        }

    }

    if (!han.new_tartarus.half_field) {
        mWorld.field.inside_dimensions.setMinorPoint({static_cast<double>(-han.new_vision.field.field_length/2), static_cast<double>(-han.new_vision.field.field_width/2)});
        mWorld.field.inside_dimensions.setMajorPoint({static_cast<double>(han.new_vision.field.field_length/2), static_cast<double>(han.new_vision.field.field_width/2)});
    }
    if (han.new_tartarus.half_field) {
        if (han.new_tartarus.right_field) {
            mWorld.field.inside_dimensions.setMinorPoint({static_cast<double>(0.0), static_cast<double>(-han.new_vision.field.field_width/2)});
            mWorld.field.inside_dimensions.setMajorPoint({static_cast<double>(han.new_vision.field.field_length/2), static_cast<double>(han.new_vision.field.field_width/2)});
        } else {
            mWorld.field.inside_dimensions.setMinorPoint({static_cast<double>(-han.new_vision.field.field_length/2), static_cast<double>(-han.new_vision.field.field_width/2)});
            mWorld.field.inside_dimensions.setMajorPoint({static_cast<double>(0.0), static_cast<double>(han.new_vision.field.field_width/2)});
        }
    }   //TODO resolver isso aqui

    AreaRectangular leftDefenseArea = {{-han.new_vision.field.field_length/2, -han.new_vision.field.defense_area_width/2},{-han.new_vision.field.field_length/2 + han.new_vision.field.defense_area_height, han.new_vision.field.defense_area_width/2}};
    AreaRectangular rightDefenseArea = {{han.new_vision.field.field_length/2 - han.new_vision.field.defense_area_height, -han.new_vision.field.defense_area_width/2}, {han.new_vision.field.field_length/2, han.new_vision.field.defense_area_width/2}};

    LineSegment leftGoal = {Point(-han.new_vision.field.field_length/2, -han.new_vision.field.goal_width/2), Point(-han.new_vision.field.field_length/2 , han.new_vision.field.goal_width/2)};
    LineSegment rightGoal = {Point(han.new_vision.field.field_length/2, -han.new_vision.field.goal_width/2), Point(han.new_vision.field.field_length/2 , han.new_vision.field.goal_width/2)};

    AreaRectangular leftFisicalBarrier = {{leftGoal.getStart().getX() - han.new_vision.field.goal_height, leftGoal.getStart().getY()}, leftGoal.getEnd()};
    AreaRectangular rightFisicalBarrier = {rightGoal.getStart(), {rightGoal.getEnd().getX() + han.new_vision.field.goal_height, rightGoal.getEnd().getY()}};
    mWorld.field.leftFisicalBarrier = leftFisicalBarrier;
    mWorld.field.rightFisicalBarrier = rightFisicalBarrier;

    if (mTeam->getOurSide() == TeamInfo::left) {
        mWorld.field.ourGoal = {Point(-han.new_vision.field.field_length/2, -han.new_vision.field.goal_width/2 - 100), Point(-han.new_vision.field.field_length/2 , han.new_vision.field.goal_width/2 + 100)};;
        mWorld.field.theirGoal = rightGoal;
        mWorld.field.ourDefenseArea = leftDefenseArea;
        mWorld.field.theirDefenseArea = rightDefenseArea;
    }
    if (mTeam->getOurSide() == TeamInfo::right) {
<<<<<<< HEAD
        mWorld.field.ourGoal = {Point(han.new_vision.field.field_length/2, -han.new_vision.field.goal_width/2 - 100), Point(han.new_vision.field.field_length/2 , han.new_vision.field.goal_width/2 + 100)};;
        mWorld.field.theirGoal = leftGoal;
=======
        mWorld.field.ourGoal = rightGoal;
        mWorld.field.theirGoal = leftGoal; //TODO COMP REMOVER
>>>>>>> 9fa43e16e1cb4304d698b0bfbfc19d3511c7cccf
        mWorld.field.ourDefenseArea = rightDefenseArea;
        mWorld.field.theirDefenseArea = leftDefenseArea;
    }
    if (han.new_tartarus.half_field) {
        if (han.new_tartarus.right_field) {
            mWorld.field.ourGoal = rightGoal;
            mWorld.field.theirGoal = rightGoal;
        } else {
            mWorld.field.ourGoal = leftGoal;
            mWorld.field.theirGoal = leftGoal;
        }
    }
}

void RobotController::loadCalibration() {
    //TODO
}


void RobotController::publish() {
    han.new_ia.robots[id].id = id;
    //if (getId() == 1) std::cout << mtarget_vel.getX() << " " << mtarget_vel.getY() << std::endl;
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
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return active;
}

double RobotController::get_m_ball_avoidance_radius() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mBall_avoidance_radius;
}

double RobotController::get_mtarget_yaw() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mtarget_yaw;
}

Vector2d RobotController::get_mtarget_vel() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mtarget_vel;
}

double RobotController::get_mtarget_vyaw() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mtarget_vyaw;
}

Vector2d RobotController::get_mlast_target_vel() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mlast_target_vel;
}

double RobotController::get_mkicker_x() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mkicker_x;
}

double RobotController::get_mkicker_z() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mkicker_z;
}

double RobotController::get_mdribbler() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mdribbler;
}

TeamInfo* RobotController::get_m_team() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mTeam;
}

int RobotController::get_m_state() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mState;
}

double RobotController::get_m_delta_time() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mDelta_time;
}

double RobotController::get_m_timer() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mTimer;
}

std::vector<Point> RobotController::get_m_current_trajectory() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mCurrent_trajectory;
}
// --- Getters ---
int RobotController::get_m_offline_counter() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mOffline_counter;
}

int RobotController::get_m_max_offline_counter() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mMax_offline_counter;
}

bool RobotController::is_m_terminate() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mTerminate;
}

double RobotController::get_m_vxy_max() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mVxy_max;
}

double RobotController::get_m_vxy_min() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mVxy_min;
}

double RobotController::get_m_a_xy_max() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mA_xy_max;
}

double RobotController::get_m_a_xy_brake() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mA_xy_brake;
}

double RobotController::get_m_vyaw_max() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mVyaw_max;
}

double RobotController::get_m_vyaw_min() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mVyaw_min;
}

double RobotController::get_m_a_ang_max() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mA_ang_max;
}

double RobotController::get_m_kicker_x_max() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mKicker_x_max;
}

double RobotController::get_m_kicker_x_min() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mKicker_x_min;
}

double RobotController::get_m_kicker_z_max() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mKicker_z_max;
}

double RobotController::get_m_kicker_z_min() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mKicker_z_min;
}

double RobotController::get_m_dribbler_max() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mDribbler_max;
}

double RobotController::get_m_dribbler_min() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mDribbler_min;
}

double RobotController::get_m_static_position_tolarance() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mStatic_position_tolarance;
}

double RobotController::get_m_dynamic_position_tolarance() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mDynamic_position_tolarance;
}

double RobotController::get_m_static_angle_tolarance() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mStatic_angle_tolarance;
}

double RobotController::get_m_kp_ang() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mKP_ang;
}

double RobotController::get_m_ki_ang() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mKI_ang;
}

double RobotController::get_m_kd_ang() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mKD_ang;
}

double RobotController::get_m_i_ang() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mI_ang;
}

double RobotController::get_m_i_ang_max() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mI_ang_max;
}

double RobotController::get_m_last_delta_vyaw() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mLast_delta_vyaw;
}

double RobotController::get_m_i_vx() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mI_vx;
}

double RobotController::get_m_i_vy() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mI_vy;
}

double RobotController::get_m_i_vxy_max() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mI_vxy_max;
}

double RobotController::get_m_last_delta_vx() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mLast_delta_vx;
}

double RobotController::get_m_last_delta_vy() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mLast_delta_vy;
}

double RobotController::get_m_kp_vxy() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mKP_vxy;
}

double RobotController::get_m_ki_vxy() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mKI_vxy;
}

double RobotController::get_m_kd_vxy() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mKD_vxy;
}

bool RobotController::is_will_double_touch() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return will_double_touch;
}

bool RobotController::is_double_touch() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return double_touch;
}

bool RobotController::is_double_touch_waiting() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return double_touch_waiting;
}

WorldModel& RobotController::get_world() {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mWorld;
}

int64_t RobotController::get_m_last_time_stamp() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return mLast_time_stamp;
}

RobotController::role RobotController::get_last_role() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return lastRole;
}

// --- Setters ---
void RobotController::set_active(bool value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    active = value;
}

void RobotController::set_m_ball_avoidance_radius(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mBall_avoidance_radius = value;
}

void RobotController::set_mtarget_yaw(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mtarget_yaw = value;
}

void RobotController::set_mtarget_vel(const Vector2d &value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mtarget_vel = value;
}

void RobotController::set_mtarget_vyaw(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mtarget_vyaw = value;
}

void RobotController::set_mlast_target_vel(const Vector2d &value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mlast_target_vel = value;
}

void RobotController::set_mkicker_x(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mkicker_x = value;
}

void RobotController::set_mkicker_z(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mkicker_z = value;
}

void RobotController::set_mdribbler(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mdribbler = value;
}

void RobotController::set_m_team(TeamInfo* value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mTeam = value;
}

void RobotController::set_m_state(int value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mState = value;
}

void RobotController::set_m_delta_time(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mDelta_time = value;
}

void RobotController::set_m_timer(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mTimer = value;
}

void RobotController::set_m_current_trajectory(const std::vector<Point> &value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mCurrent_trajectory = value;
}

void RobotController::set_m_offline_counter(int value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mOffline_counter = value;
}

void RobotController::set_m_max_offline_counter(int value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mMax_offline_counter = value;
}

void RobotController::set_m_terminate(bool value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mTerminate = value;
}

void RobotController::set_m_vxy_max(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mVxy_max = value;
}

void RobotController::set_m_vxy_min(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mVxy_min = value;
}

void RobotController::set_m_a_xy_max(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mA_xy_max = value;
}

void RobotController::set_m_a_xy_brake(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mA_xy_brake = value;
}

void RobotController::set_m_vyaw_max(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mVyaw_max = value;
}

void RobotController::set_m_vyaw_min(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mVyaw_min = value;
}

void RobotController::set_m_a_ang_max(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mA_ang_max = value;
}

void RobotController::set_m_kicker_x_max(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mKicker_x_max = value;
}

void RobotController::set_m_kicker_x_min(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mKicker_x_min = value;
}

void RobotController::set_m_kicker_z_max(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mKicker_z_max = value;
}

void RobotController::set_m_kicker_z_min(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mKicker_z_min = value;
}

void RobotController::set_m_dribbler_max(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mDribbler_max = value;
}

void RobotController::set_m_dribbler_min(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mDribbler_min = value;
}

void RobotController::set_m_static_position_tolarance(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mStatic_position_tolarance = value;
}

void RobotController::set_m_dynamic_position_tolarance(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mDynamic_position_tolarance = value;
}

void RobotController::set_m_static_angle_tolarance(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mStatic_angle_tolarance = value;
}

void RobotController::set_m_kp_ang(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mKP_ang = value;
}

void RobotController::set_m_ki_ang(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mKI_ang = value;
}

void RobotController::set_m_kd_ang(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mKD_ang = value;
}

void RobotController::set_m_i_ang(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mI_ang = value;
}

void RobotController::set_m_i_ang_max(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mI_ang_max = value;
}

void RobotController::set_m_last_delta_vyaw(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mLast_delta_vyaw = value;
}

void RobotController::set_m_i_vx(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mI_vx = value;
}

void RobotController::set_m_i_vy(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mI_vy = value;
}

void RobotController::set_m_i_vxy_max(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mI_vxy_max = value;
}

void RobotController::set_m_last_delta_vx(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mLast_delta_vx = value;
}

void RobotController::set_m_last_delta_vy(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mLast_delta_vy = value;
}

void RobotController::set_m_kp_vxy(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mKP_vxy = value;
}

void RobotController::set_m_ki_vxy(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mKI_vxy = value;
}

void RobotController::set_m_kd_vxy(double value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mKD_vxy = value;
}

void RobotController::set_will_double_touch(bool value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    will_double_touch = value;
}

void RobotController::set_double_touch(bool value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    double_touch = value;
}

void RobotController::set_double_touch_waiting(bool value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    double_touch_waiting = value;
}

void RobotController::set_world(WorldModel& value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mWorld = value;
}

void RobotController::set_m_last_time_stamp(int64_t value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    mLast_time_stamp = value;
}

void RobotController::set_last_role(role value) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    lastRole = value;
}

enum Robot::role RobotController::getRole() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return this_role;
}

void RobotController::setRole(enum Robot::role r) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    this_role = r;
}

// --- Getters ---
bool RobotController::isAlly() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return ally;
}

bool RobotController::isPositioned() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return positioned;
}

bool RobotController::isOriented() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return oriented;
}

bool RobotController::isAligned() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return aligned;
}

int RobotController::getId() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return id;
}

double RobotController::getKickerColddown() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return kicker_colddown;
}

double RobotController::getKickerTimer() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return kicker_timer;
}

double RobotController::getPushTime() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return push_time;
}

Point RobotController::getOldPosition() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return old_pos;
}

Point RobotController::getPosition() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return pos;
}

double RobotController::getYaw() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return yaw;
}

Vector2d& RobotController::getVelocity() {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return velocity;
}

double RobotController::getVyaw() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return vyaw;
}

bool RobotController::isDetected() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return detected;
}

const std::deque<Vector2d>& RobotController::getStoredVelocities() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return stored_velocities;
}

double RobotController::getRadius() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return radius;
}

bool RobotController::hasKicker() {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return kicker;
}

// --- Setters ---
void RobotController::setKicker(bool kicker) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    this->kicker = kicker;
}

void RobotController::setAlly(bool is) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    this->ally = is;
}

void RobotController::setPositioned(bool is) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    this->positioned = is;
}

void RobotController::setOriented(bool is) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    this->oriented = is;
}

void RobotController::setAligned(bool is) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    this->aligned = is;
}

void RobotController::setId(int id) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    this->id = id;
}

void RobotController::setKickerColddown(double t) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    this->kicker_colddown = t;
}

void RobotController::setKickerTimer(double t) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    this->kicker_timer = t;
}

void RobotController::setPushTime(double t) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    this->push_time = t;
}


void RobotController::setPosition(const Point& p) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    old_pos = pos;
    pos = p;
}

void RobotController::setYaw(double y) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    yaw = y;
}

void RobotController::setVelocity(const Vector2d& v) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    if (stored_velocities.size() >= max_velocities_stored) {
       stored_velocities.pop_front();
    }
    stored_velocities.push_back(v);
    double average_x = 0;
    double average_y = 0;
    for (int i = 0; i < stored_velocities.size(); i++) {
       average_x += stored_velocities[i].getX()/stored_velocities.size();
       average_y += stored_velocities[i].getY()/stored_velocities.size();
    }
    velocity = Vector2d(average_x, average_y);
}

void RobotController::setVyaw(double v) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    if (stored_yaw_velocities.size() >= max_yaw_velocities_stored) {
       stored_yaw_velocities.pop_front();
    }
    stored_yaw_velocities.push_back(v);
    double average = 0;
    for (int i = 0; i < stored_yaw_velocities.size(); i++) {
       average += stored_yaw_velocities[i]/stored_yaw_velocities.size();
    }
    vyaw = average;
}

bool RobotController::isSpinning() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return (vyaw > yawVelocityThreshold);
}

void RobotController::setDetected(bool d) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    detected = d;
}

bool RobotController::isMoving() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    if (velocity.getNorm() > velocityThreshold) {
        return true;
    }
    return false;
}

bool RobotController::isStopped() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return !isMoving();  // Now safe - recursive_mutex allows nested locks
}

// --- Stored Velocities ---
void RobotController::setStoredVelocities(const std::deque<Vector2d>& vels) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    stored_velocities = vels;
<<<<<<< HEAD
=======
}

bool RobotController::isKickingOnVision() const {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return kickOnVision;
>>>>>>> 9fa43e16e1cb4304d698b0bfbfc19d3511c7cccf
}