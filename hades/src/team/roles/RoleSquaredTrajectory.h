//
// Created by caiu on 25/08/25.
//

#ifndef ROLESQUAREDTRAJECTORY_H
#define ROLESQUAREDTRAJECTORY_H
#include "RoleBase.h"

namespace roles {

class RoleSquaredTrajectory : public RoleBase {
private:
<<<<<<< HEAD
	std::vector<Point> default_trajectory = {{1500, -200}, {1500, 200}, {200, 200}, {200, -200}};
=======
	std::vector<Point> default_trajectory = {{1000, -1000}, {200, -1000}, {200, 1000}, {1000, 1000}};
>>>>>>> 9fa43e16e1cb4304d698b0bfbfc19d3511c7cccf
	//std::vector<Point> default_trajectory = {{-1000, -1000}, {1000, 1000}, {-1000, -1000}, {1000, 1000}};
public:
	void act(RobotController& robot);
};

} // roles

#endif //ROLESQUAREDTRAJECTORY_H
