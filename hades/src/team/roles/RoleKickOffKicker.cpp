//
// Created by caiu on 18/08/25.
//

#include "RoleKickOffKicker.h"
#include "../RobotController.h"
#include "../TeamInfo.h"

namespace roles {
    void RoleKickOffKicker::act(RobotController& robot) {
        bool others_positioned = true;
        for (Robot r : robot.get_world().allies) if (!r.isPositioned() && !r.getId() != robot.getId()) others_positioned = false;
		if (others_positioned && robot.get_world().isAllAlliesOnOurSideorOnCenterCircle() && robot.get_m_team()->getEvent() == TeamInfo::ourKickOff && robot.get_world().ball.getPosition().getDistanceTo({0, 0}) < 100) {
    		try {	//TODO ALGO MUITO ERRADO
    			Robot support(-1);
    			try {
    				support = robot.get_m_team()->getRobotofRole(Robot::kickoff_support);
    			} catch (...) {
    				support = robot.get_m_team()->getRobotofRole(Robot::support);
    			}
    			positionAndKick.act(robot, support);
    		} catch (...) { //NO SUPPORT
    			Point p = robot.get_world().getGoalPosition();
    			positionAndKick.act(robot, p);
    		}
    	} else {
	        Point their_goal = robot.get_world().field.theirGoal.getMiddle();
    		Point center = {0, 0};
        	auto kickposition = robot.get_world().getKickingPosition(center, their_goal, robot.get_m_ball_avoidance_radius() + robot.getRadius() + 10); //Nao sei, nao me pergunte
    		moveTo.act(robot, kickposition, true);
    		turnTo.act(robot, robot.get_world().ball.getPosition());
        }
    }
} // roles