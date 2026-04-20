//
// Created by caiu on 25/08/25.
//

#ifndef ROLESQUAREDTRAJECTORY_H
#define ROLESQUAREDTRAJECTORY_H
#include "RoleBase.h"

namespace roles {

class RoleSquaredTrajectory : public RoleBase {
private:
	std::vector<Point> default_trajectory = {{1000, -1000}, {200, -1000}, {200, 1000}, {1000, 1000}};
	//std::vector<Point> default_trajectory = {{-1000, -1000}, {1000, 1000}, {-1000, -1000}, {1000, 1000}};
public:
	void act(RobotController& robot);
};

} // roles

#endif //ROLESQUAREDTRAJECTORY_H
