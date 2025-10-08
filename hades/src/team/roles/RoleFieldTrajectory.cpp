//
// Created by caiu on 25/08/25.
//

#include "RoleFieldTrajectory.h"

#include "../RobotController.h"

namespace roles {
	void RoleFieldTrajectory::act(RobotController& robot){
		if (size(robot.get_m_current_trajectory()) == 0) {
			robot.get_m_current_trajectory().push_back({robot.get_world().field.inside_dimensions.getMajorPoint().getX(), robot.get_world().field.inside_dimensions.getMajorPoint().getY()});
			robot.get_m_current_trajectory().push_back({robot.get_world().field.inside_dimensions.getMajorPoint().getX(), robot.get_world().field.inside_dimensions.getMinorPoint().getY()});
			robot.get_m_current_trajectory().push_back({robot.get_world().field.inside_dimensions.getMinorPoint().getX(), robot.get_world().field.inside_dimensions.getMinorPoint().getY()});
			robot.get_m_current_trajectory().push_back({robot.get_world().field.inside_dimensions.getMinorPoint().getX(), robot.get_world().field.inside_dimensions.getMajorPoint().getY()});
		}
		followTrajectory.act(robot, robot.get_m_current_trajectory());
    }

} // roles