//
// Created by caiu on 25/08/25.
//

#include "RoleCircularTrajectory.h"
#include "../RobotController.h"

namespace roles {
	void RoleCircularTrajectory::act(RobotController& robot) {
		if (size(robot.get_m_current_trajectory()) == 0) {
			for (int i = 0; i < resolution; i++) {
				robot.get_m_current_trajectory().emplace_back(radius*sin(2*i*M_PI/resolution + robot.getId()*2*M_PI/size(robot.get_world().allies)) + center.getX(), radius*cos(2*i*M_PI/resolution + robot.getId()*2*M_PI/size(robot.get_world().allies)) + center.getY());
			}
		}
		robot.set_m_vxy_min(robot.get_m_vxy_min());
		followTrajectory.act(robot, robot.get_m_current_trajectory());
    }
}