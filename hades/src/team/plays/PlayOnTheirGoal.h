//
// Created by caiu on 21/09/25.
//

#ifndef PLAYONTHEIRGOAL_H
#define PLAYONTHEIRGOAL_H

#include "../geometry/WorldModel.h"
#include "../Robot.h"
#include "PlayBase.h"


class PlayOnTheirGoal : public PlayBase {
	std::array<Robot::role, 16> role_assign(WorldModel& world, TeamInfo& team, std::array<Robot::role, 16> roles) override;
	int calc_score(WorldModel world, TeamInfo team) override;
public:
	PlayOnTheirGoal() {
		name = "onTheirGoal";
		required_robots = 3;
		required_roles = {Robot::goal_keeper, Robot::retaker, Robot::defender};
	}
};



#endif //PLAYONTHEIRGOAL_H
