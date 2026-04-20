//
// Created by caiu on 15/04/25.
//

#include "PlayAttack.h"

#include <iostream>
#include <math.h>

int PlayAttack::calc_score(WorldModel world, TeamInfo& team) {
    int score = 50;
    if (team.getEvent() == TeamInfo::run && world.ball.getPosition().getX() > 0 && team.getOurSide() == TeamInfo::left) {
        score += 300;
    }
    else if (team.getEvent() == TeamInfo::run && world.ball.getPosition().getX() < 0 && team.getOurSide() == TeamInfo::right) {
        score += 300;
    }

    if (world.getBallOwner().isAlly()) {
        score += 400;
    }

    this->score = score;
    return score;
}

std::array<Robot::role, 16> PlayAttack::role_assign(WorldModel& world, TeamInfo& team, std::array<Robot::role, 16> roles) {
    std::vector<Robot*> avaiable_robots = {};
    for (int i = 0 ; i < 16 ; i++) { //TODO FAZER ISSO EM TODOS
        if (team.isRobotActive(i) == 1) {
            if (roles[i] != Robot::unknown) {
                continue;
            }
            avaiable_robots.push_back(&world.allies[i]);
        }
    }

    if (avaiable_robots.empty()) {
        return roles;
    }

    //role assign
    for (Robot::role selected_role : required_roles) {
        if (avaiable_robots.empty()) {
            return roles;
        }

        if (selected_role == Robot::goal_keeper) {
            if (!world.allies[team.getGoalKeeperId()].isDetected()) continue;
            int goal_keeper_idx = -1;
            for (int i = 0 ; i < avaiable_robots.size() ; i++) {
                if (avaiable_robots[i]->getId() == team.getGoalKeeperId()) goal_keeper_idx = i;
            }
            if (goal_keeper_idx == -1) continue;
            avaiable_robots[goal_keeper_idx]->setRole(Robot::goal_keeper);
            roles[team.getGoalKeeperId()] = Robot::goal_keeper;
            avaiable_robots.erase(avaiable_robots.begin() + goal_keeper_idx);
        }

        if (selected_role == Robot::striker) {
            int closest_idx = 0;
            for (int idx = 0; idx < avaiable_robots.size(); idx++) {
                if (avaiable_robots[idx]->getPosition().getDistanceTo(world.ball.getPosition()) < avaiable_robots[closest_idx]->getPosition().getDistanceTo(world.ball.getPosition())) {
                    closest_idx = idx;
                }
            }
            int closest_id = avaiable_robots[closest_idx]->getId();
            avaiable_robots[closest_idx]->setRole(selected_role);
            roles[closest_id] = selected_role;
            avaiable_robots.erase(avaiable_robots.begin() + closest_idx);
            continue;
        }

        if (selected_role == Robot::support) {
            int closest_idx = 0;
            if (world.ball.isStopped()) {
                for (int idx = 0; idx < avaiable_robots.size(); idx++) {
                    if (avaiable_robots[idx]->getPosition().getDistanceTo(world.ball.getPosition()) < avaiable_robots[closest_idx]->getPosition().getDistanceTo(world.ball.getPosition())) {
                        closest_idx = idx;
                    }
                }
            }
            else {
                Robot interceptor = world.getClosestAllyToPoint(world.ball.getStopPosition());
                for (int idx = 0; idx < avaiable_robots.size(); idx++) {
                    if (avaiable_robots[idx]->getId() == interceptor.getId()) {
                        closest_idx = idx;
                        break;
                    }
                }
            }
            int closest_id = avaiable_robots[closest_idx]->getId();
            avaiable_robots[closest_idx]->setRole(Robot::support);
            roles[closest_id] = Robot::support;
            avaiable_robots.erase(avaiable_robots.begin() + closest_idx);
        }
    }

    return roles;
}