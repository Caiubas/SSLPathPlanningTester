//
// Created by caiu on 25/08/25.
//

#include "SkillKick.h"

#include <iostream>

#include "../RobotController.h"
#include "../TeamInfo.h"

namespace skills {
	double SkillKick::find_angle_error(RobotController& robot, Point goal) {
		double theta_final = atan2(goal.getY() - robot.getPosition().getY(), goal.getX() - robot.getPosition().getX());
		double orientation = robot.getYaw();
		double delta = theta_final - orientation;
		if (delta > M_PI) delta -= 2 * M_PI;
		if (delta < -M_PI) delta += 2 * M_PI;
		//return 0; //TODO remover
		return delta;
	}

	void SkillKick::act(RobotController& robot) {
		if (robot.hasKicker()) {
			if (robot.getPosition().getDistanceTo(robot.get_world().ball.getPosition()) > distancethreshold + robot.getRadius() || robot.get_world().ball.isMoving()) {
				robot.set_mkicker_x(0);
				robot.setPositioned(false);
				robot.get_m_team()->setPositioned(robot.getId(), false);
				robot.set_mtarget_vel({0, 0});
				//TODO adicionar return?
			}
			if (robot.get_m_team()->getEvent() == TeamInfo::ourFreeKick or robot.get_m_team()->getEvent() == TeamInfo::runningOurFreeKick or robot.get_m_team()->getEvent() == TeamInfo::theirFreeKick or robot.get_m_team()->getEvent() == TeamInfo::runningTheirFreeKick or robot.get_m_team()->getEvent() == TeamInfo::ourKickOff or robot.get_m_team()->getEvent() == TeamInfo::theirKickOff) {
				robot.set_will_double_touch(true);
			}
			Vector2d v_vet = {robot.get_world().ball.getPosition(), robot.getPosition()};
			v_vet = v_vet.getNormalized(robot.get_m_vxy_min());
			robot.set_mtarget_vel(v_vet.getRotated(-robot.getYaw()));
			robot.set_mkicker_x(3.5);

		} else {
			if (robot.getPosition().getDistanceTo(robot.get_world().ball.getPosition()) > distancethreshold + robot.getRadius() || robot.get_world().ball.getVelocity().getNorm() >= robot.get_m_vxy_max()/2) {
				robot.set_mkicker_x(0);
				robot.setPositioned(false);
				robot.get_m_team()->setPositioned(robot.getId(), false);
				robot.set_mtarget_vel({0, 0});
				//TODO adicionar return?
			}
			if (robot.get_m_team()->getEvent() == TeamInfo::ourFreeKick or robot.get_m_team()->getEvent() == TeamInfo::runningOurFreeKick or robot.get_m_team()->getEvent() == TeamInfo::theirFreeKick or robot.get_m_team()->getEvent() == TeamInfo::runningTheirFreeKick or robot.get_m_team()->getEvent() == TeamInfo::ourKickOff or robot.get_m_team()->getEvent() == TeamInfo::theirKickOff) {
				robot.set_will_double_touch(true);
			}
			Vector2d v_vet = {robot.get_world().ball.getPosition(), robot.getPosition()};
			v_vet = v_vet.getNormalized(robot.get_m_vxy_max());
			robot.set_mtarget_vel(v_vet.getRotated(-robot.getYaw()));
			//sleep(1); // Gambiarra braba mas NAO funciona
		}

		double angle_error = find_angle_error(robot, robot.get_world().ball.getPosition());	//TODO TESTAR ISSO AQUI
		if (fabs(angle_error) > 2*robot.get_m_static_angle_tolarance()) {
			robot.set_mtarget_vyaw(angle_error*robot.get_m_vyaw_min()/(2*fabs(angle_error)));
		}
	}
} // skills