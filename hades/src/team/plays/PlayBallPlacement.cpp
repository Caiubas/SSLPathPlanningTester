//
// Created by caiu on 23/09/25.
//

#include "PlayBallPlacement.h"

#include <iostream>
#include <math.h>

int PlayBallPlacement::calc_score(WorldModel world, TeamInfo team) {
    int score = 0;
    if (team.event == TeamInfo::ourballPlacement) {
        score += 999999992;
    }

    this->score = score;
    return score;
}

std::array<Robot::role, 16> PlayBallPlacement::role_assign(WorldModel& world, TeamInfo& team, std::array<Robot::role, 16> roles) {
    std::vector<Robot*> avaiable_robots = {};
    for (int i = 0 ; i < std::size(team.active_robots) ; i++) {
        if (team.active_robots[i] == 1) {
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
            if (!world.allies[team.goal_keeper_id].isDetected()) continue;
            int goal_keeper_idx = -1;
            for (int i = 0 ; i < avaiable_robots.size() ; i++) {
                if (avaiable_robots[i]->getId() == team.goal_keeper_id) goal_keeper_idx = i;
            }
            if (goal_keeper_idx == -1) continue;
            avaiable_robots[goal_keeper_idx]->setRole(Robot::goal_keeper);
            roles[team.goal_keeper_id] = Robot::goal_keeper;
            avaiable_robots.erase(avaiable_robots.begin() + goal_keeper_idx);
        }

        if (selected_role == Robot::placeHolder) {  //mais proximo da posicao desejada da bola
            int closest_idx = 0;
            for (int idx = 0; idx < avaiable_robots.size(); idx++) {
                if (avaiable_robots[idx]->getPosition().getDistanceTo(team.ball_placement_spot) < avaiable_robots[closest_idx]->getPosition().getDistanceTo(team.ball_placement_spot)) {
                    closest_idx = idx;
                }
            }
            int closest_id = avaiable_robots[closest_idx]->getId();
            avaiable_robots[closest_idx]->setRole(selected_role);
            roles[closest_id] = selected_role;
            avaiable_robots.erase(avaiable_robots.begin() + closest_idx);
        }

        if (selected_role == Robot::placer) {   //mais proximo da bola
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
        }
    }

    return roles;
}