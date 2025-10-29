//
// Created by caiu on 25/08/25.
//

#ifndef ROLESQUAREDTRAJECTORY_H
#define ROLESQUAREDTRAJECTORY_H
#include "RoleBase.h"

namespace roles {

class RoleSquaredTrajectory : public RoleBase {
private:
	std::vector<Point> default_trajectory = {{1500, -200}, {1500, 200}, {200, 200}, {200, -200}};
	//std::vector<Point> default_trajectory = {{-1000, -1000}, {1000, 1000}, {-1000, -1000}, {1000, 1000}};
public:
	void act(RobotController& robot);
};

} // roles

#endif //ROLESQUAREDTRAJECTORY_H
