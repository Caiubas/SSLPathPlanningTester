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

    [[nodiscard]] bool is_active() const;
    [[nodiscard]] double get_m_ball_avoidance_radius() const;
    [[nodiscard]] double get_mtarget_yaw() const;
    [[nodiscard]] Vector2d get_mtarget_vel() const;
    [[nodiscard]] double get_mtarget_vyaw() const;
    [[nodiscard]] Vector2d get_mlast_target_vel() const;
    [[nodiscard]] double get_mkicker_x() const;
    [[nodiscard]] double get_mkicker_z() const;
    [[nodiscard]] double get_mdribbler() const;
    [[nodiscard]] TeamInfo* get_m_team() const;
    [[nodiscard]] int get_m_state() const;
    [[nodiscard]] double get_m_delta_time() const;
    [[nodiscard]] double get_m_timer() const;
    [[nodiscard]] std::vector<Point> get_m_current_trajectory() const;
    [[nodiscard]] int get_m_offline_counter() const;
    [[nodiscard]] int get_m_max_offline_counter() const;
    [[nodiscard]] bool is_m_terminate() const;
    [[nodiscard]] double get_m_vxy_max() const;
    [[nodiscard]] double get_m_vxy_min() const;
    [[nodiscard]] double get_m_a_xy_max() const;
    [[nodiscard]] double get_m_a_xy_brake() const;
    [[nodiscard]] double get_m_vyaw_max() const;
    [[nodiscard]] double get_m_vyaw_min() const;
    [[nodiscard]] double get_m_a_ang_max() const;
    [[nodiscard]] double get_m_kicker_x_max() const;
    [[nodiscard]] double get_m_kicker_x_min() const;
    [[nodiscard]] double get_m_kicker_z_max() const;
    [[nodiscard]] double get_m_kicker_z_min() const;
    [[nodiscard]] double get_m_dribbler_max() const;
    [[nodiscard]] double get_m_dribbler_min() const;
    [[nodiscard]] double get_m_static_position_tolarance() const;
    [[nodiscard]] double get_m_dynamic_position_tolarance() const;
    [[nodiscard]] double get_m_static_angle_tolarance() const;
    [[nodiscard]] double get_m_kp_ang() const;
    [[nodiscard]] double get_m_ki_ang() const;
    [[nodiscard]] double get_m_kd_ang() const;
    [[nodiscard]] double get_m_i_ang() const;
    [[nodiscard]] double get_m_i_ang_max() const;
    [[nodiscard]] double get_m_last_delta_vyaw() const;
    [[nodiscard]] double get_m_i_vx() const;
    [[nodiscard]] double get_m_i_vy() const;
    [[nodiscard]] double get_m_i_vxy_max() const;
    [[nodiscard]] double get_m_last_delta_vx() const;
    [[nodiscard]] double get_m_last_delta_vy() const;
    [[nodiscard]] double get_m_kp_vxy() const;
    [[nodiscard]] double get_m_ki_vxy() const;
    [[nodiscard]] double get_m_kd_vxy() const;
    [[nodiscard]] bool is_will_double_touch() const;
    [[nodiscard]] bool is_double_touch() const;
    [[nodiscard]] bool is_double_touch_waiting() const;
    [[nodiscard]] WorldModel& get_world();
    [[nodiscard]] int64_t get_m_last_time_stamp() const;
    [[nodiscard]] enum role get_last_role() const;

    // --- Setters ---
    void set_active(bool active);
    void set_m_ball_avoidance_radius(double m_ball_avoidance_radius);
    void set_mtarget_yaw(double mtarget_yaw);
    void set_mtarget_vel(const Vector2d &mtarget_vel);
    void set_mtarget_vyaw(double mtarget_vyaw);
    void set_mlast_target_vel(const Vector2d &mlast_target_vel);
    void set_mkicker_x(double mkicker_x);
    void set_mkicker_z(double mkicker_z);
    void set_mdribbler(double mdribbler);
    void set_m_team(TeamInfo *m_team);
    void set_m_state(int m_state);
    void set_m_delta_time(double m_delta_time);
    void set_m_timer(double m_timer);
    void set_m_current_trajectory(const std::vector<Point> &m_current_trajectory);
    void set_m_offline_counter(int m_offline_counter);
    void set_m_max_offline_counter(int m_max_offline_counter);
    void set_m_terminate(bool m_terminate);
    void set_m_vxy_max(double m_vxy_max);
    void set_m_vxy_min(double m_vxy_min);
    void set_m_a_xy_max(double m_a_xy_max);
    void set_m_a_xy_brake(double m_a_xy_brake);
    void set_m_vyaw_max(double m_vyaw_max);
    void set_m_vyaw_min(double m_vyaw_min);
    void set_m_a_ang_max(double m_a_ang_max);
    void set_m_kicker_x_max(double m_kicker_x_max);
    void set_m_kicker_x_min(double m_kicker_x_min);
    void set_m_kicker_z_max(double m_kicker_z_max);
    void set_m_kicker_z_min(double m_kicker_z_min);
    void set_m_dribbler_max(double m_dribbler_max);
    void set_m_dribbler_min(double m_dribbler_min);
    void set_m_static_position_tolarance(double m_static_position_tolarance);
    void set_m_dynamic_position_tolarance(double m_dynamic_position_tolarance);
    void set_m_static_angle_tolarance(double m_static_angle_tolarance);
    void set_m_kp_ang(double m_kp_ang);
    void set_m_ki_ang(double m_ki_ang);
    void set_m_kd_ang(double m_kd_ang);
    void set_m_i_ang(double m_i_ang);
    void set_m_i_ang_max(double m_i_ang_max);
    void set_m_last_delta_vyaw(double m_last_delta_vyaw);
    void set_m_i_vx(double m_i_vx);
    void set_m_i_vy(double m_i_vy);
    void set_m_i_vxy_max(double m_i_vxy_max);
    void set_m_last_delta_vx(double m_last_delta_vx);
    void set_m_last_delta_vy(double m_last_delta_vy);
    void set_m_kp_vxy(double m_kp_vxy);
    void set_m_ki_vxy(double m_ki_vxy);
    void set_m_kd_vxy(double m_kd_vxy);
    void set_will_double_touch(bool will_double_touch);
    void set_double_touch(bool double_touch);
    void set_double_touch_waiting(bool double_touch_waiting);
    void set_world(WorldModel& m_world);
    void set_m_last_time_stamp(int64_t m_last_time_stamp);
    void set_last_role(enum role last_role);


    void setRole(enum role r);
    enum role getRole() const;

    bool isAlly() const;
    bool isPositioned() const;
    bool isOriented() const;
    bool isAligned() const;
    int getId() const;
    Point getOldPosition() const;
    Point getPosition() const;
    double getYaw() const;
    Vector2d& getVelocity();
    double getVyaw() const;
    bool isDetected() const;
    bool hasKicker();
    bool isStopped() const;
    bool isMoving() const;
    bool isSpinning() const;
    const std::deque<Vector2d>& getStoredVelocities() const;
    double getRadius() const;
    double getKickDistance() const {return kickDistance;};
    bool isKickingOnVision() const;

    // --- Setters ---
    void setAlly(bool is);
    void setPositioned(bool is);
    void setOriented(bool is);
    void setAligned(bool is);
    void setKicker(bool kicker);
    void setId(int id);
    void setPosition(const Point& p);
    void setYaw(double y);
    void setVelocity(const Vector2d& v);
    void setVyaw(double v);
    void setDetected(bool d);
    void setStoredVelocities(const std::deque<Vector2d>& vels);

private:
    mutable std::recursive_mutex mtx;
    bool active = false;
    double mBall_avoidance_radius = 50;

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

    bool technical_challenge = false;

    enum role lastRole = unknown;
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
    void debug_mode();
    void do_technical_challenge();

};



#endif //ROBOT_H
