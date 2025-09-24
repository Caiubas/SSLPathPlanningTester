//
// Created by caiu on 23/09/25.
//

#ifndef PLAYBALLPLACEMENT_H
#define PLAYBALLPLACEMENT_H

#include "../geometry/WorldModel.h"
#include "../Robot.h"
#include "PlayBase.h"

class PlayBallPlacement : public PlayBase {
	std::array<Robot::role, 16> role_assign(WorldModel& world, TeamInfo& team, std::array<Robot::role, 16> roles) override;
	int calc_score(WorldModel world, TeamInfo team) override;
public:
	PlayBallPlacement() {
		name = "ball_placement";
		required_robots = 3;
		required_roles = {Robot::goal_keeper, Robot::placeHolder, Robot::placer};
	}
};



#endif //PLAYBALLPLACEMENT_H
