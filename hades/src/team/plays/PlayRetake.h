//
// Created by caiu on 21/09/25.
//

#ifndef PLAYRETAKE_H
#define PLAYRETAKE_H
#include "../geometry/WorldModel.h"
#include "../Robot.h"
#include "PlayBase.h"



class PlayRetake : public PlayBase {
	std::array<Robot::role, 16> role_assign(WorldModel& world, TeamInfo& team, std::array<Robot::role, 16> roles) override;
	int calc_score(WorldModel world, TeamInfo& team) override;
public:
	PlayRetake() {
		name = "retake";
		required_robots = 3;
		required_roles = {Robot::goal_keeper, Robot::retaker, Robot::marker};
	}
};



#endif //PLAYRETAKE_H
