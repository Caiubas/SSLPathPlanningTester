//
// Created by caiu on 19/09/25.
//

#include "PlayTheirPenalty.h"

#include <iostream>
#include <ostream>

#include <team_info.hpp>


#include "../TeamInfo.h"

int PlayTheirPenalty::calc_score(WorldModel world, TeamInfo& team) {
    int new_score = 0;
    if (team.getEvent() == TeamInfo::theirPenalty || team.getEvent() == TeamInfo::runningTheirPenalty) {
        new_score += 999992;
    }
    this->score = new_score;
    return score;
}

std::array<Robot::role, 16> PlayTheirPenalty::role_assign(WorldModel& world, TeamInfo& team, std::array<Robot::role, 16> roles) {
    int num_active_robots = 0;
    std::vector<int> active_allies_ids = {};
    std::vector<double> distances_allies_from_ball = {};
    for (int i = 0 ; i < team.getNumOfActiveRobots() ; i++) {
        if (team.isRobotActive(i) == 1) {
            if (roles[i] != Robot::unknown) {
                continue;
            }
            num_active_robots++;
            active_allies_ids.push_back(i);
            distances_allies_from_ball.push_back(world.allies[i].getPosition().getDistanceTo(world.ball.getPosition()));
        }
    }

    if (active_allies_ids.empty()) {
        return roles;  // ou faça algo apropriado, como um log de erro
    }

    //role assign
    for (Robot::role selected_role : required_roles) {
        if (active_allies_ids.empty()) {
            return roles;
        }
        if (selected_role == Robot::goal_keeper) {
            if (!world.allies[team.getGoalKeeperId()].isDetected()) continue;
            roles[team.getGoalKeeperId()] = Robot::goal_keeper;
            int idx = -1;
            for (int i = 0; i<active_allies_ids.size(); i++) if (i == team.getGoalKeeperId()) idx = i;
            active_allies_ids.erase(active_allies_ids.begin() + idx);
            distances_allies_from_ball.erase(distances_allies_from_ball.begin() + idx);
        }

        if (selected_role == Robot::halted) {
            if (roles[active_allies_ids[0]] == Robot::unknown) {
                roles[active_allies_ids[0]] = selected_role;
                active_allies_ids.erase(active_allies_ids.begin());
            }
        }
        if (selected_role == Robot::watcher) {
            if (roles[active_allies_ids[0]] == Robot::unknown) {
                roles[active_allies_ids[0]] = selected_role;
                active_allies_ids.erase(active_allies_ids.begin());
            }
        }
    }
    return roles;
}