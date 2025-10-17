//
// Created by caiu on 25/08/25.
//

#include "SkillTurnTo.h"
#include "../RobotController.h"
#include "../TeamInfo.h"
#include "../c_trajectory/C_trajectory.h"


namespace skills {
	double SkillTurnTo::find_angle_error(RobotController& robot, Point goal) {
		double theta_final = atan2(goal.getY() - robot.getPosition().getY(), goal.getX() - robot.getPosition().getX());
		double orientation = robot.getYaw();
		double delta = theta_final - orientation;
		if (delta > M_PI) delta -= 2 * M_PI;
		if (delta < -M_PI) delta += 2 * M_PI;
		return delta;
	}

	double SkillTurnTo::turn_control(RobotController& robot, double delta) {
		double P = robot.get_m_kp_ang() * delta;
		if (!robot.isOriented()) {	//anti-windup
			robot.set_m_i_ang(std::clamp(robot.get_m_i_ang() + delta*robot.get_m_delta_time()*robot.get_m_ki_ang(), -robot.get_m_i_ang_max(), robot.get_m_i_ang_max()));
		}
		double D = ((delta-robot.get_m_last_delta_vyaw())/robot.get_m_delta_time())*robot.get_m_kd_ang();	//TODO fitro no derivativo (sofre mto de ruido)
		double PID_vyaw = P + robot.get_m_ki_ang() + D;

		robot.set_m_last_delta_vyaw(delta);
		if (fabs(PID_vyaw) > robot.get_m_vyaw_max()) {
			PID_vyaw = robot.get_m_vyaw_max()*PID_vyaw/fabs(PID_vyaw);
		};
		if (fabs(PID_vyaw) < robot.get_m_vyaw_min() && fabs(PID_vyaw) != 0) {
			PID_vyaw = robot.get_m_vyaw_min()*PID_vyaw/fabs(PID_vyaw);
		}

		return PID_vyaw;
	}

void SkillTurnTo::act(RobotController& robot, Point goal) {
	double delta = find_angle_error(robot, goal);
	if (!robot.isOriented() && fabs(delta) < robot.get_m_static_angle_tolarance()) {	//quando alinhando
		robot.set_mtarget_vyaw(0);
		if (!robot.isSpinning()) {
			robot.setOriented(true);
		}
		return;
	}
	if (robot.isOriented() && fabs(delta) < robot.get_m_static_angle_tolarance()*2) { //quando já alinhado
		robot.set_mtarget_vyaw(0);
		if (!robot.isSpinning()) {
			robot.setOriented(true);
		}
		return;
	}

	robot.setOriented(false);
	if (robot.get_mlast_target_vel().getNorm() != 0) {
		//robot.set_mtarget_vyaw(0); //TODO REMOVER COMP
		//return;
	}
	double new_vyaw = turn_control(robot, delta);
	if (new_vyaw > robot.get_mtarget_vyaw() + robot.get_m_delta_time()*robot.get_m_a_ang_max()) {
		new_vyaw = robot.get_mtarget_vyaw() + robot.get_m_delta_time()*robot.get_m_a_ang_max();
	}
	if (new_vyaw < robot.get_mtarget_vyaw() - robot.get_m_delta_time()*robot.get_m_a_ang_max()) {
		new_vyaw = robot.get_mtarget_vyaw() - robot.get_m_delta_time()*robot.get_m_a_ang_max();
	}
	robot.set_mtarget_vyaw(new_vyaw);
}
} // skills