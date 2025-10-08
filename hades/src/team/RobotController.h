//
// Created by caiom on 4/4/2025.
//

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H
#include <vector>
#include <lcm/lcm-cpp.hpp>

#include "Robot.h"
#include "geometry/WorldModel.h"
#include "geometry/geometry.h"
#include "skills/skills.h"
#include "tactics/tactics.h"
#include "roles/roles.h"

class TeamInfo;

class RobotController : public Robot{

public:
    RobotController(int new_id): Robot(new_id), mWorld() {
        id = new_id;
        setPosition({0, 0});
    }

    void start(TeamInfo* team_ads);
    void stop();
    void loop();
    bool isActive();

    bool active = false;

    double mBall_avoidance_radius = 100;

    //target movimentation
    double mtarget_yaw = 0;
    Vector2d mtarget_vel = {0, 0};
    double mtarget_vyaw = 0;
    Vector2d mlast_target_vel = {0, 0};

    //actuators activationstop
    double mkicker_x = 0;
    double mkicker_z = 0;
    double mdribbler = 0;

    //strategy status
    TeamInfo* mTeam;  //role; -1 unsigned

    int mState = 0;  //estado

    double mDelta_time = 0;
    double mTimer = 0;
    std::vector<Point> mCurrent_trajectory = {};



    //on-field detection
    int mOffline_counter = 0;
    int mMax_offline_counter = 10;
    bool mTerminate = false;


    //extreme params
    double mVxy_max = 0.7;
    double mVxy_min = 0.1;
    double mA_xy_max = 2;
    double mA_xy_brake = 500;
    double mVyaw_max = 5;
    double mVyaw_min = 0.3;
    double mA_ang_max = 150;
    double mKicker_x_max = 3;
    double mKicker_x_min = 0.5;
    double mKicker_z_max = 1;
    double mKicker_z_min = 0.5;
    double mDribbler_max = 1;
    double mDribbler_min = 0.5;

    //angle and position tolerance
    double mStatic_position_tolarance = radius/2;
    double mDynamic_position_tolarance = radius/2;
    double mStatic_angle_tolarance = 0.1;

    //PID control
    double mKP_ang = 0.05;
    double mKI_ang = 0;
    double mKD_ang = 0;
    double mI_ang = 0;
    double mI_ang_max = 1;
    double mLast_delta_vyaw = 0;

    double mI_vx = 0;
    double mI_vy = 0;
    double mI_vxy_max = 0;
    double mLast_delta_vx = 0;
    double mLast_delta_vy = 0;

    double mKP_vxy = 0;
    double mKI_vxy = 0;
    double mKD_vxy = 0;

    //field info
    bool will_double_touch = false;
    bool double_touch = false;
    bool double_touch_waiting = false;
    WorldModel mWorld;
    int64_t mLast_time_stamp = 0;

    enum role lastRole = unknown;

private:

    void check_connection();
    void dynamic_calculations();
    void setActive(bool active);

    void receive_vision();
    void receive_field_geometry();
    void receive_config();
    void publish();
    void loadCalibration();
    void doubleTouchHandler();
    void select_behavior();

};



#endif //ROBOT_H
