//
// Created by caiu on 18/09/25.
//

#include "RoleRetaker.h"
#include "../TeamInfo.h"

namespace roles {
	void RoleRetaker::act(RobotController& robot) {
		bool has_goal = false;
		Point goal = robot.get_world().field.theirGoal.getMiddle();
		bool has_enemies = false;
		Robot closest_enemy_to_ball(-1);
		bool has_support = false;
		Robot support(-1);
		try {
			closest_enemy_to_ball = robot.get_world().getClosestEnemyToPoint(robot.get_world().ball.getPosition());
			has_enemies = true;
		} catch (...) {std::cout << "no enemies" << std::endl;}
		try {
			goal = robot.get_world().getGoalPosition();
			has_goal = true;
		} catch (...) {
		}
		try {
			support = robot.get_m_team()->getRobotofRole(Robot::marker);
			has_support = true;
		} catch (...) {
			try {
				support = robot.get_m_team()->getRobotofRole(Robot::support);
				has_support = true;
			} catch (...) {}
		}
		if (robot.get_world().ball.isMoving()) {
			intercept.act(robot);
		} else if (robot.get_world().ball.isStopped() && (
			closest_enemy_to_ball.getPosition().getDistanceTo(robot.get_world().ball.getPosition())
			 < robot.getPosition().getDistanceTo(robot.get_world().ball.getPosition())) && has_goal) {
			positionAndKick.act(robot, goal);
		} else if (robot.get_world().ball.isStopped() && (
			closest_enemy_to_ball.getPosition().getDistanceTo(robot.get_world().ball.getPosition())
			< robot.getPosition().getDistanceTo(robot.get_world().ball.getPosition())) && has_support) {
			positionAndKick.act(robot, support);
		}
		else if (has_enemies) {
			blockBall.act(robot, closest_enemy_to_ball, 200);
		} else {
			stop.act(robot); //Nao deve ocorrer
		}
    }
} // roles