//
// Created by caiu on 25/08/25.
//

#include "SkillStop.h"
#include "../RobotController.h"

namespace skills {
	void SkillStop::act(RobotController& robot) {
		robot.set_mtarget_vel({0, 0});
		robot.set_mtarget_vyaw(0);
	}
} // skills