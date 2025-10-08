//
// Created by caiom on 4/7/2025.
//

#ifndef ROBOT_H
#define ROBOT_H

#include <deque>
#include <mutex>

#include "geometry/Point.h"
#include "geometry/Vector2d.h"

class Robot {
public:
    enum role {
        unknown = -1,
        goal_keeper,
        striker,
        support,
        defender,
        halted,
        kickoff_kicker,
        kickoff_goal_keeper,
        kickoff_support,
        marker,
        debug_circular_trajectory,
        debug_squared_trajectory,
        retaker,
        penaltier,
        freeKicker,
        watcher,
        placeHolder,
        placer
    };

protected:
    bool ally = true;
    int id = -1;
    Point old_pos = {0, 0};
    Point pos = {0, 0};
    double yaw = 0.0;
    Vector2d velocity = Vector2d(0,0);
	int max_velocities_stored = 30;
	int max_yaw_velocities_stored = 30;
    std::deque<Vector2d> stored_velocities = {};
    std::deque<double> stored_yaw_velocities = {};
    double velocityThreshold = 0.1;
    double yawVelocityThreshold = 0.001;
    double vyaw = 0.0;
    double radius = 160;
    bool detected = false;
    bool kicker = false;
    double kickDistance = 500;
    enum role this_role = unknown;
    bool positioned = true;
    bool aligned = true;
    bool oriented = true;
public:
    // Construtor
    Robot(int id) : id(id) {this_role = unknown;}

    //flags

    // --- Getters ---
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
    enum role getRole() const;
    const std::deque<Vector2d>& getStoredVelocities() const;
    double getRadius() const;
    double getKickDistance() const {return kickDistance;};

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
    void setRole(enum role r);
    void setStoredVelocities(const std::deque<Vector2d>& vels);

};

#endif //ROBOT_H
