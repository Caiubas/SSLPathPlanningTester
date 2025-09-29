//
// Created by caiu on 25/08/25.
//

#ifndef ROLECIRCULARTRAJECTORY_H
#define ROLECIRCULARTRAJECTORY_H
#include "RoleBase.h"

namespace roles {
	class RoleCircularTrajectory : public roles::RoleBase {
	private:
		int resolution = 120;
		double radius = 500;
	public:
		void act(RobotController& robot);
	};
}


#endif //ROLECIRCULARTRAJECTORY_H
