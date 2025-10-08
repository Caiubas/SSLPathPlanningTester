//
// Created by caiu on 06/10/25.
//

#include "SkillPushBall.h"
#include <math.h>
#include "../RobotController.h"
#include "../TeamInfo.h"

namespace skills {
	double SkillPushBall::find_angle_error(RobotController robot, Point goal) {
		double theta_final = atan2(goal.getY() - robot.getPosition().getY(), goal.getX() - robot.getPosition().getX());
		double orientation = robot.getYaw();
		double delta = theta_final - orientation;
		if (delta > M_PI) delta -= 2 * M_PI;
		if (delta < -M_PI) delta += 2 * M_PI;
		//return 0; //TODO remover
		return delta;
	}

	void SkillPushBall::act(RobotController& robot) {
		if (robot.getPosition().getDistanceTo(robot.mWorld.ball.getPosition()) > distancethreshold + robot.getRadius() || robot.mWorld.ball.getVelocity().getNorm() >= robot.mVxy_max/2) {
			robot.mkicker_x = 0;
			robot.positioned = false;
			robot.mTeam->positioned[robot.getId()] = false;
			robot.mtarget_vel = {0, 0};
			if (robot.mTeam->event == TeamInfo::ourFreeKick or robot.mTeam->event == TeamInfo::theirFreeKick or robot.mTeam->event == TeamInfo::ourKickOff or robot.mTeam->event == TeamInfo::theirKickOff) {
				robot.double_touch = true;
			}
		}
		Vector2d v_vet = {robot.mWorld.ball.getPosition(), robot.getPosition()};
		v_vet = v_vet.getNormalized(robot.mVxy_max);
		robot.mtarget_vel = v_vet.getRotated(-robot.getYaw());

		double angle_error = find_angle_error(robot, robot.mWorld.ball.getPosition());	//TODO TESTAR ISSO AQUI
		if (fabs(angle_error) > 2*robot.mStatic_angle_tolarance) {
			robot.mtarget_vyaw = angle_error*robot.mVyaw_min/(2*fabs(angle_error));
		}
	}
} // skills