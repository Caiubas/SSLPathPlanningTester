//
// Created by caiu on 18/09/25.
//

#include "RoleRetaker.h"
#include "../TeamInfo.h"

namespace roles {
	void RoleRetaker::act(RobotController& robot) {
		bool has_goal = false;
		Point goal = robot.mWorld.field.theirGoal.getMiddle();
		bool has_enemies = false;
		Robot closest_enemy_to_ball(-1);
		bool has_support = false;
		Robot support(-1);
		try {
			closest_enemy_to_ball = robot.mWorld.getClosestEnemyToPoint(robot.mWorld.ball.getPosition());
			has_enemies = true;
		} catch (...) {std::cout << "no enemies" << std::endl;}
		try {
			goal = robot.mWorld.getGoalPosition();
			has_goal = true;
		} catch (...) {
		}
		try {
			support = robot.mTeam->getRobotofRole(Robot::marker);
			has_support = true;
		} catch (...) {
			try {
				support = robot.mTeam->getRobotofRole(Robot::support);
				has_support = true;
			} catch (...) {}
		}
		if (robot.mWorld.ball.isMoving()) {
			intercept.act(robot);
		} else if (robot.mWorld.ball.isStopped() && (
			closest_enemy_to_ball.getPosition().getDistanceTo(robot.mWorld.ball.getPosition())
			 < robot.getPosition().getDistanceTo(robot.mWorld.ball.getPosition())) && has_goal) {
			positionAndKick.act(robot, goal);
		} else if (robot.mWorld.ball.isStopped() && (
			closest_enemy_to_ball.getPosition().getDistanceTo(robot.mWorld.ball.getPosition())
			< robot.getPosition().getDistanceTo(robot.mWorld.ball.getPosition())) && has_support) {
			positionAndKick.act(robot, support);
		}
		else if (has_enemies) {
			blockBall.act(robot, closest_enemy_to_ball, 200);
		} else {
			stop.act(robot); //Nao deve ocorrer
		}
    }
} // roles