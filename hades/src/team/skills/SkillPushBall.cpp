//
// Created by caiu on 06/10/25.
//

#include "SkillPushBall.h"
#include <math.h>
#include "../RobotController.h"
#include "../TeamInfo.h"

namespace skills {
	double SkillPushBall::find_angle_error(RobotController& robot, Point goal) {
		double theta_final = atan2(goal.getY() - robot.getPosition().getY(), goal.getX() - robot.getPosition().getX());
		double orientation = robot.getYaw();
		double delta = theta_final - orientation;
		if (delta > M_PI) delta -= 2 * M_PI;
		if (delta < -M_PI) delta += 2 * M_PI;
		//return 0; //TODO remover
		return delta;
	}

	void SkillPushBall::act(RobotController& robot) {
		robot.push_time += robot.get_m_delta_time();
		std::cout << "push time: " << robot.push_time << std::endl;
		if (robot.push_time >= 3 || (robot.push_time >= 1.5 && (robot.getPosition().getDistanceTo(robot.get_world().ball.getPosition()) > distancethreshold + robot.getRadius() || robot.get_world().ball.getVelocity().getNorm() >= robot.get_m_vxy_min()))) {
			robot.push_time = 0;
			robot.set_mkicker_x(0);
			robot.setPositioned(false);
			robot.get_m_team()->setPositioned(robot.getId(), false);
			robot.set_mtarget_vel({0, 0});
		}
		if (robot.get_m_team()->getEvent() == TeamInfo::ourFreeKick or robot.get_m_team()->getEvent() == TeamInfo::runningOurFreeKick or robot.get_m_team()->getEvent() == TeamInfo::theirFreeKick or robot.get_m_team()->getEvent() == TeamInfo::runningTheirFreeKick or robot.get_m_team()->getEvent() == TeamInfo::ourKickOff or robot.get_m_team()->getEvent() == TeamInfo::theirKickOff) {
			robot.set_will_double_touch(true);
		}
		robot.set_mkicker_x(0);
		Vector2d v_vet = {robot.get_world().ball.getPosition(), robot.getPosition()};
		v_vet = v_vet.getNormalized(robot.get_m_vxy_max()/2);
		robot.set_mtarget_vel(v_vet.getRotated(-robot.getYaw()));

		LineSegment robot_goal(robot.getPosition(), robot.get_world().ball.getPosition());
		if (robot_goal.getResized(10).intersects(robot.get_world().field.ourGoal)) {
			//NAO FAZER GOL CONTRA
			//robot.set_mtarget_vel({0, 0}); //TODO REMOVER ARRUMAR COMP
		}

		double angle_error = find_angle_error(robot, robot.get_world().ball.getPosition());	//TODO TESTAR ISSO AQUI
		if (fabs(angle_error) > 2*robot.get_m_static_angle_tolarance()) {
			robot.set_mtarget_vyaw(angle_error*robot.get_m_vyaw_min()/(2*fabs(angle_error)));
		}
	}
} // skills