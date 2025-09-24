//
// Created by caiu on 23/09/25.
//

#ifndef ROLEPLACEHOLDER_H
#define ROLEPLACEHOLDER_H
#include "RoleBase.h"

namespace roles {

class RolePlaceHolder : public RoleBase {
public:
	void act(RobotController& robot);
};

} // roles

#endif //ROLEPLACEHOLDER_H
