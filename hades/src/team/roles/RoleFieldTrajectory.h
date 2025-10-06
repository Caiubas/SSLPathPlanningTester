//
// Created by caiu on 25/08/25.
//

#ifndef ROLEFIELDTRAJECTORY_H
#define ROLEFIELDTRAJECTORY_H
#include "RoleBase.h"

namespace roles {

class RoleFieldTrajectory : public RoleBase {
public:
	void act(RobotController& robot);
};

} // roles

#endif //ROLEFIELDTRAJECTORY_H
