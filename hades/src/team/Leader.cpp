//
// Created by caiom on 4/4/2025.
//

#include "Leader.h"
#include <iostream>
#include <thread>
#include <algorithm>
#include <math.h>
#include <numeric>
#include <unordered_set>

#include "../include/handlers.hpp"
#include "TeamInfo.h"
#include "plays/PlayOurKickOff.h"

Leader::Leader() {
    plays.clear();
    plays.push_back(std::make_unique<PlayAttack>());
    plays.push_back(std::make_unique<PlayDebug>());
    plays.push_back(std::make_unique<PlayHalt>());
    plays.push_back(std::make_unique<PlayOurKickOff>());
    plays.push_back(std::make_unique<PlayDefense>());
    plays.push_back(std::make_unique<PlayTheirKickOff>());
    plays.push_back(std::make_unique<PlayOurPenalty>());
    plays.push_back(std::make_unique<PlayTheirPenalty>());
    plays.push_back(std::make_unique<PlayRetake>());
    plays.push_back(std::make_unique<PlayOnTheirGoal>());
<<<<<<< HEAD
    plays.push_back(std::make_unique<PlayOnOurGoal>());
=======
>>>>>>> 0ff8d929619e556decd7d668ada64239d8c8b102
    plays.push_back(std::make_unique<PlayBallPlacement>());
}


void Leader::start() {
    loop();
}

void Leader::loop() {
    std::cout << "Leader inicializado" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    auto t0 = std::chrono::steady_clock::now();
    receive_vision();
    delta_time = 0.016;

    while (true) {
        if (last_time_stamp == han.new_ia.timestamp) {
            continue;
        }

        auto t1 = std::chrono::steady_clock::now();

        receive_config();
        receive_vision();
        receive_gamecontroller();
        receive_field_geometry();

        inspect_enemy_team();
        world_analysis();
        select_plays();
        //imprimir_ativos();

        //std::cout << team.central_line_x << std::endl;

        std::chrono::duration<double> delta = t1 - t0;
        t0 = std::chrono::steady_clock::now();
        delta_time = delta.count();
        if (delta_time == 0 or delta_time < 0) {
            std::cout << "??" << std::endl;
            delta_time = 1/60;
        }

    }

}

void Leader::receive_vision() {
    std::unordered_set<int> allies_detected = {};
    std::unordered_set<int> enemies_detected = {};

    for (auto blue_robot : han.new_vision.robots_blue) {
        if (!blue_robot.detected) continue;
        if (team.getColor() == TeamInfo::blue) {
            int rb_id = blue_robot.robot_id;
            if (team.isRobotActive(rb_id) == 0) {
                add_robot(rb_id);
                continue;
            }
            double new_yaw = blue_robot.orientation;
            if (new_yaw < 0) new_yaw += 2*M_PI;
            if (delta_time > 0) {
                //TODO logica quebrada na troca de estrutura, refazer.
                Vector2d v = {((blue_robot.position_x - world.allies[rb_id].getPosition().getX())/(delta_time*1000)), ((blue_robot.position_y - world.allies[rb_id].getPosition().getY())/(delta_time*1000))};
                auto velocities = world.allies[rb_id].getStoredVelocities();
                velocities.push_back(v);
                if (size(world.allies[rb_id].getStoredVelocities()) > 10) {
                    velocities.pop_front();
                }
                world.allies[rb_id].getVelocity().setX(v.getX()); // <<< aqui!
                world.allies[rb_id].getVelocity().setY(v.getY()); // <<< e aqui!
                world.allies[rb_id].setStoredVelocities(velocities);
            }
            world.allies[rb_id].setYaw(new_yaw);
            world.allies[rb_id].setPosition({blue_robot.position_x, blue_robot.position_y});
            world.allies[rb_id].setAlly(true);
            allies_detected.insert(rb_id);
        }
        else {
            int rb_id = blue_robot.robot_id;
            double new_yaw = blue_robot.orientation;
            if (new_yaw < 0) new_yaw += 2*M_PI;
            if (delta_time > 0) {
                //TODO
                Vector2d v = {((blue_robot.position_x - world.enemies[rb_id].getPosition().getX())/(delta_time*1000)), ((blue_robot.position_y - world.allies[rb_id].getPosition().getY())/(delta_time*1000))};
                auto velocities = world.enemies[rb_id].getStoredVelocities();
                velocities.push_back(v);
                if (size(world.enemies[rb_id].getStoredVelocities()) > 10) {
                    velocities.pop_front();
                }
                world.enemies[rb_id].getVelocity().setX(v.getX()); // <<< aqui!
                world.enemies[rb_id].getVelocity().setY(v.getY()); // <<< e aqui!
                world.enemies[rb_id].setStoredVelocities(velocities);
            }
            world.enemies[rb_id].setYaw(new_yaw);
            world.enemies[rb_id].setPosition({blue_robot.position_x, blue_robot.position_y});
            world.enemies[rb_id].setAlly(false);
            enemies_detected.insert(rb_id);
        }
    }


    for (auto yellow_robot : han.new_vision.robots_yellow) {
        if (!yellow_robot.detected) continue;
        if (team.getColor() == TeamInfo::yellow) {
            int rb_id = yellow_robot.robot_id;
            if (team.isRobotActive(rb_id) == 0) {
                add_robot(rb_id);
                continue;
            }
            double new_yaw = yellow_robot.orientation;
            if (new_yaw < 0) new_yaw += 2*M_PI;
            if (delta_time > 0) {
                //TODO logica quebrada na troca de estrutura, refazer.
                Vector2d v = {((yellow_robot.position_x - world.allies[rb_id].getPosition().getX())/(delta_time*1000)), ((yellow_robot.position_y - world.allies[rb_id].getPosition().getY())/(delta_time*1000))};
                auto velocities = world.allies[rb_id].getStoredVelocities();
                velocities.push_back(v);
                if (size(world.allies[rb_id].getStoredVelocities()) > 10) {
                    velocities.pop_front();
                }
                world.allies[rb_id].getVelocity().setX(v.getX()); // <<< aqui!
                world.allies[rb_id].getVelocity().setY(v.getY()); // <<< e aqui!
                world.allies[rb_id].setStoredVelocities(velocities);
            }
            world.allies[rb_id].setYaw(new_yaw);
            world.allies[rb_id].setPosition({yellow_robot.position_x, yellow_robot.position_y});
            world.allies[rb_id].setAlly(true);
            allies_detected.insert(rb_id);
        }
        else {
            int rb_id = yellow_robot.robot_id;

            double new_yaw = yellow_robot.orientation;
            if (new_yaw < 0) new_yaw += 2*M_PI;
            if (delta_time > 0) {
                //TODO
                Vector2d v = {((yellow_robot.position_x - world.enemies[rb_id].getPosition().getX())/(delta_time*1000)), ((yellow_robot.position_y - world.allies[rb_id].getPosition().getY())/(delta_time*1000))};
                auto velocities = world.enemies[rb_id].getStoredVelocities();
                velocities.push_back(v);
                if (size(world.enemies[rb_id].getStoredVelocities()) > 10) {
                    velocities.pop_front();
                }
                world.enemies[rb_id].getVelocity().setX(v.getX()); // <<< aqui!
                world.enemies[rb_id].getVelocity().setY(v.getY()); // <<< e aqui!
                world.enemies[rb_id].setStoredVelocities(velocities);
            }
            world.enemies[rb_id].setYaw(new_yaw);
            world.enemies[rb_id].setPosition({yellow_robot.position_x, yellow_robot.position_y});
            enemies_detected.insert(rb_id);
        }
    }

    for (int i = 0; i < size(world.allies); i++) {
        if (allies_detected.find(i) != allies_detected.end()) world.allies[i].setDetected(true);
        else world.allies[i].setDetected(false);
    }

    for (int i = 0; i < size(world.enemies); i++) {
        if (enemies_detected.find(i) != enemies_detected.end()) world.enemies[i].setDetected(true);
        else world.enemies[i].setDetected(false);
    }

    if (delta_time != 0) {
        world.ball.setVelocity({(han.new_vision.balls.position_x - world.ball.getPosition().getX())/(delta_time*1000), (han.new_vision.balls.position_y - world.ball.getPosition().getY())/(delta_time*1000)});
    }
    world.ball.setPosition({han.new_vision.balls.position_x, han.new_vision.balls.position_y});
    last_time_stamp = han.new_vision.timestamp;
}

void Leader::receive_field_geometry() {
    //TODO implementar urgente
    world.field.inside_dimensions.setMinorPoint({static_cast<double>(-han.new_vision.field.field_length/2), static_cast<double>(-han.new_vision.field.field_width/2)});
    world.field.inside_dimensions.setMajorPoint({static_cast<double>(han.new_vision.field.field_length/2), static_cast<double>(han.new_vision.field.field_width/2)});

    AreaRectangular leftDefenseArea = {{-han.new_vision.field.field_length/2 - han.new_vision.field.goal_height, -han.new_vision.field.defense_area_width/2},{-han.new_vision.field.field_length/2 + han.new_vision.field.defense_area_height, han.new_vision.field.defense_area_width/2}};
    AreaRectangular rightDefenseArea = {{han.new_vision.field.field_length/2 - han.new_vision.field.defense_area_height, -han.new_vision.field.defense_area_width/2}, {han.new_vision.field.field_length/2 + han.new_vision.field.goal_height, han.new_vision.field.defense_area_width/2}};

    LineSegment leftGoal = {Point(-han.new_vision.field.field_length/2, -han.new_vision.field.goal_width/2), Point(-han.new_vision.field.field_length/2 , han.new_vision.field.goal_width/2)};
    LineSegment rightGoal = {Point(han.new_vision.field.field_length/2, -han.new_vision.field.goal_width/2), Point(han.new_vision.field.field_length/2 , han.new_vision.field.goal_width/2)};

    AreaRectangular leftFisicalBarrier = {leftGoal.getStart(), {leftGoal.getEnd().getX() - han.new_vision.field.goal_depth, leftGoal.getEnd().getY()}};
    AreaRectangular rightFisicalBarrier = {rightGoal.getStart(), {rightGoal.getEnd().getX() + han.new_vision.field.goal_depth, rightGoal.getEnd().getY()}};

    world.field.leftFisicalBarrier = leftFisicalBarrier;
    world.field.rightFisicalBarrier = rightFisicalBarrier;
    if (team.getOurSide() == TeamInfo::left) {
        world.field.ourGoal = leftGoal;
        world.field.theirGoal = rightGoal;
        world.field.ourDefenseArea = leftDefenseArea;
        world.field.theirDefenseArea = rightDefenseArea;
    }
    if (team.getOurSide() == TeamInfo::right) {
        world.field.ourGoal = rightGoal;
        world.field.theirGoal = leftGoal;
        world.field.ourDefenseArea = rightDefenseArea;
        world.field.theirDefenseArea = leftDefenseArea;
    }
}
void Leader::event_FSM() {
    if (team.getCurrentCommand() == TeamInfo::HALT) team.setEvent(TeamInfo::halt);

    if (team.getEvent() == TeamInfo::halt) {
        GC_timer = 0;
        if (team.getCurrentCommand() == TeamInfo::STOP) {
            team.setEvent(TeamInfo::stop);
        }
    }

    if (team.getEvent() == TeamInfo::timeout) {
        GC_timer = 0;
        if (team.getCurrentCommand() == TeamInfo::STOP) team.setEvent(TeamInfo::stop);
    }

    if (team.getEvent() == TeamInfo::stop) {
        GC_timer = 0;
        if (team.getCurrentCommand() == TeamInfo::PREPARE_KICKOFF_BLUE) {
            if (team.getColor() == TeamInfo::blue) team.setEvent(TeamInfo::prepareOurKickOff);
            else team.setEvent(TeamInfo::prepareTheirKickOff);
        }
        if (team.getCurrentCommand() == TeamInfo::PREPARE_KICKOFF_YELLOW) {
            if (team.getColor() == TeamInfo::yellow) team.setEvent(TeamInfo::prepareOurKickOff);
            else team.setEvent(TeamInfo::prepareTheirKickOff);
        }

        if (team.getCurrentCommand() == TeamInfo::BALL_PLACEMENT_BLUE) {
            if (team.getColor() == TeamInfo::blue) team.setEvent(TeamInfo::ourballPlacement);
            else team.setEvent(TeamInfo::theirballPlacement);
        }
        if (team.getCurrentCommand() == TeamInfo::BALL_PLACEMENT_YELLOW) {
            if (team.getColor() == TeamInfo::yellow) team.setEvent(TeamInfo::ourballPlacement);
            else team.setEvent(TeamInfo::theirballPlacement);
        }

        if (team.getCurrentCommand() == TeamInfo::PREPARE_PENALTY_BLUE) {
            if (team.getColor() == TeamInfo::blue) team.setEvent(TeamInfo::prepareOurPenalty);
            else team.setEvent(TeamInfo::prepareTheirPenalty);
        }
        if (team.getCurrentCommand() == TeamInfo::PREPARE_PENALTY_YELLOW) {
            if (team.getColor() == TeamInfo::yellow) team.setEvent(TeamInfo::prepareOurPenalty);
            else team.setEvent(TeamInfo::prepareTheirPenalty);
        }

        if (team.getCurrentCommand() == TeamInfo::FORCE_START) team.setEvent(TeamInfo::run);

        if (team.getCurrentCommand() == TeamInfo::DIRECT_FREE_BLUE) {
            if (team.getColor() == TeamInfo::blue) team.setEvent(TeamInfo::ourFreeKick);
            else team.setEvent(TeamInfo::theirFreeKick);
        }
        if (team.getCurrentCommand() == TeamInfo::DIRECT_FREE_YELLOW) {
            if (team.getColor() == TeamInfo::yellow) team.setEvent(TeamInfo::ourFreeKick);
            else team.setEvent(TeamInfo::theirFreeKick);
        }

        if (team.getCurrentCommand() == TeamInfo::TIMEOUT_BLUE or team.getCurrentCommand() == TeamInfo::TIMEOUT_YELLOW) {
            team.setEvent(TeamInfo::timeout);
        }
    }

    if (team.getEvent() == TeamInfo::prepareOurKickOff) {
        GC_timer = 0;
        if (team.getCurrentCommand() == TeamInfo::NORMAL_START) {
            team.setEvent(TeamInfo::ourKickOff);
        }
    }

    if (team.getEvent() == TeamInfo::prepareTheirKickOff) {
        GC_timer = 0;
        if (team.getCurrentCommand() == TeamInfo::NORMAL_START) {
            team.setEvent(TeamInfo::theirKickOff);
        }
    }

    if (team.getEvent() == TeamInfo::ourballPlacement) {
        GC_timer += delta_time;
        if (team.getCurrentCommand() == TeamInfo::STOP) team.setEvent(TeamInfo::stop);
        if (delta_time > 30) team.setEvent(TeamInfo::stop);
        if (world.ball.getPosition().getDistanceTo(team.getBallPlacementSpot()) < 150 && world.ball.isStopped()) team.setEvent(TeamInfo::stop);
    }

    if (team.getEvent() == TeamInfo::theirballPlacement) {
        GC_timer += delta_time;
        if (team.getCurrentCommand() == TeamInfo::STOP) team.setEvent(TeamInfo::stop);
        if (delta_time >= 1) team.setEvent(TeamInfo::theirFreeKick);
    }

    if (team.getEvent() == TeamInfo::prepareOurPenalty) {
        GC_timer = 0;
        if (team.getCurrentCommand() == TeamInfo::NORMAL_START) team.setEvent(TeamInfo::ourPenalty);
    }

    if (team.getEvent() == TeamInfo::prepareTheirPenalty) {
        GC_timer = 0;
        if (team.getCurrentCommand() == TeamInfo::NORMAL_START) team.setEvent(TeamInfo::theirPenalty);
    }

    if (team.getEvent() == TeamInfo::ourPenalty or team.getEvent() == TeamInfo::theirPenalty) {
        GC_timer = 0;
        if (team.getCurrentCommand() == TeamInfo::NORMAL_START && team.getEvent() == TeamInfo::runningTheirPenalty) team.setEvent(TeamInfo::runningTheirPenalty);
        if (team.getCurrentCommand() == TeamInfo::NORMAL_START && team.getEvent() == TeamInfo::ourPenalty) team.setEvent(TeamInfo::runningOurPenalty);
        if (team.getCurrentCommand() == TeamInfo::STOP) team.setEvent(TeamInfo::stop);
    }

    if (team.getEvent() == TeamInfo::runningOurPenalty || team.getEvent() == TeamInfo::runningTheirPenalty) {
        GC_timer += delta_time;
        if (GC_timer >= 10 or team.getCurrentCommand() == TeamInfo::STOP) team.setEvent(TeamInfo::stop);
    }

    if (team.getEvent() == TeamInfo::ourKickOff or team.getEvent() == TeamInfo::theirKickOff) {
        GC_timer += delta_time;
        if (team.getCurrentCommand() == TeamInfo::NORMAL_START && ((world.ball.isMoving() && world.ball.getPosition().getDistanceTo(Point(0, 0)) > 100) or GC_timer > 10)) {
            team.setEvent(TeamInfo::run);
        }
    }

    if (team.getEvent() == TeamInfo::ourFreeKick or team.getEvent() == TeamInfo::theirFreeKick) {
        GC_timer = 0;
        if (world.ball.isMoving()) {
            team.setEvent(TeamInfo::run);
        }
        if (team.getCurrentCommand() == TeamInfo::NORMAL_START) {
            if (team.getEvent() == TeamInfo::ourFreeKick) team.setEvent(TeamInfo::runningOurFreeKick);
            if (team.getEvent() == TeamInfo::theirFreeKick) team.setEvent(TeamInfo::runningTheirFreeKick);
        }
    }
    if (team.getEvent() == TeamInfo::runningOurFreeKick or team.getEvent() == TeamInfo::runningTheirFreeKick) {
        GC_timer += delta_time;
        if (world.ball.isMoving() or GC_timer > 10) {
            team.setEvent(TeamInfo::run);
        }
    }

    if (team.getEvent() == TeamInfo::run) {
        GC_timer = 0;
        if (team.getCurrentCommand() == TeamInfo::STOP) team.setEvent(TeamInfo::stop);
    }
}


void Leader::receive_config() {
}

void Leader::receive_gamecontroller() {
    //TODO implementar maquina de estados dos estados do jogo
    team.setCurrentCommand(TeamInfo::Command(han.new_GC.current_command));

    team.setBallPlacementSpot({han.new_GC.designated_position_x, han.new_GC.designated_position_y});
    int is_team_blue = int(han.new_GC.team_blue);
    if (is_team_blue == 1) {
        team.setColor(TeamInfo::blue);
        team.setGoalKeeperId(han.new_GC.blue.goalkeeper_id);
    }
    else if (is_team_blue == 0) {
        team.setColor(TeamInfo::yellow);
        team.setGoalKeeperId(han.new_GC.yellow.goalkeeper_id);
    }

    if (han.new_GC.blue_team_on_positive_half) {
        if (is_team_blue) team.setOurSide(TeamInfo::right);
        else team.setOurSide(TeamInfo::left);
    } else {
        if (is_team_blue) team.setOurSide(TeamInfo::left);
        else team.setOurSide(TeamInfo::right);
    }

    event_FSM();
}

void Leader::world_analysis() {
}


void Leader::add_robot(int id) {
    if (team.isRobotActive(id) == 0) {
        team.setRobotActive(id, true);
        team.getRobotController(id).start(&team);
    }
}

void Leader::select_plays() {
    // Calcular scores
    for (auto& p : plays) {
        try {
            p->calc_score(world, team);
        } catch (...) {
            std::cout << "error acessing play" << std::endl;
        }
    }

    // Ordenar do maior para o menor score
    std::sort(plays.begin(), plays.end(),
          [](const std::unique_ptr<PlayBase>& a, const std::unique_ptr<PlayBase>& b) {
              return a->get_score() > b->get_score();
          });


    // Criar lista inicial de roles
    std::array<Robot::role, 16> roles;
    roles.fill(Robot::unknown);

    // Aplicar roles de todas as plays em ordem de score
    for (auto& p : plays) {
        roles = p->role_assign(world, team, roles);
    }

    // Copiar para o time
    for (int i = 0; i < roles.size(); i++) {
        team.setAllyRole(i, roles[i]);
    }
}





void Leader::inspect_enemy_team() {
    if (size(world.enemies) == 0) return;
    std::vector<int> active_enemies_ids = {};
    std::vector<double> distances_enemies_from_ball = {};
    if (size(world.enemies) == 0) return;
    for (int i = 0; i < size(world.enemies) ; i++) {
        if (world.enemies[i].isDetected()) {
            active_enemies_ids.push_back(world.enemies[i].getId());
            distances_enemies_from_ball.push_back(world.enemies[i].getPosition().getDistanceTo(world.ball.getPosition()));
        }
    }
    if (active_enemies_ids.size() == 0) return;
    if (team.getColor() == TeamInfo::blue) {
        team.setEnemyRole(han.new_GC.yellow.goalkeeper_id, Robot::goal_keeper);
    }
    else {
        team.setEnemyRole(han.new_GC.blue.goalkeeper_id, Robot::goal_keeper);
    }

    int closest_idx = -1;
    int second_closest_idx = -1;
    for (int idx : active_enemies_ids) {
        if (team.getEnemyRole(idx) == Robot::goal_keeper || !world.enemies[idx].isDetected()) continue;

        if (closest_idx == -1 || distances_enemies_from_ball[idx] < distances_enemies_from_ball[closest_idx]) {
            // Atualiza os dois
            second_closest_idx = closest_idx;
            closest_idx = idx;
        } else if (second_closest_idx == -1 || distances_enemies_from_ball[idx] < distances_enemies_from_ball[second_closest_idx]) {
            // Atualiza só o segundo
            second_closest_idx = idx;
        }
    }

    unsigned int id = world.enemies[closest_idx].getId();
    if (team.getEnemyRole(id) != Robot::goal_keeper && world.enemies[id].isDetected()) team.setEnemyRole(id, Robot::striker);

    id = world.enemies[second_closest_idx].getId();
    if (team.getEnemyRole(id) != Robot::goal_keeper && world.enemies[id].isDetected()) team.setEnemyRole(id, Robot::support);

}



void Leader::imprimir_ativos() {
    std::cout << std::endl << "[";
    for (int i = 0; i < 16 ; i++) {
        if (team.isRobotActive(i) == 1) {
            std::cout << i << " , ";
        }
    }
    std::cout << "]" << std::endl;
}