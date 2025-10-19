//
// Created by caiu on 18/08/25.
//

#ifndef ROLESTRIKER_H
#define ROLESTRIKER_H
#include "RoleBase.h"

namespace roles {

class RoleStriker : public RoleBase {
public:
    Point getSupportPosition(RobotController& robot);
    Point find_ball_trajectory(RobotController& robot, Point goal);
    void act(RobotController& robot);
};

} // roles

#endif //ROLESTRIKER_H
