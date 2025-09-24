//
// Created by caiu on 23/09/25.
//

#ifndef ROLEPLACER_H
#define ROLEPLACER_H
#include "RoleBase.h"

namespace roles {

class RolePlacer : public RoleBase {
public:
	void act(RobotController& robot);
};

} // roles

#endif //ROLEPLACER_H
