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
	int max_velocities_stored = 60;
	int max_yaw_velocities_stored = 60;
    std::deque<Vector2d> stored_velocities = {};
    std::deque<double> stored_yaw_velocities = {};
    double velocityThreshold = 0.1;
    double yawVelocityThreshold = 0.001;
    double vyaw = 0.0;
    double radius = 100;
    bool detected = false;
    bool kicker = true;
    double kickDistance = 2000;
    enum role this_role = unknown;
    bool positioned = true;
    bool aligned = true;
    bool oriented = true;
public:
    // Construtor
    Robot(int id) : id(id) {this_role = unknown;}

    //flags

    // --- Getters ---
    virtual bool isAlly() const;
    virtual bool isPositioned() const;
    virtual bool isOriented() const;
    virtual bool isAligned() const;
    virtual int getId() const;
    virtual Point getOldPosition() const;
    virtual Point getPosition() const;
    virtual double getYaw() const;
    virtual Vector2d& getVelocity();
    virtual double getVyaw() const;
    virtual bool isDetected() const;
    virtual bool hasKicker();
    virtual bool isStopped() const;
    virtual bool isMoving() const;
    virtual bool isSpinning() const;
    virtual enum role getRole() const;
    virtual const std::deque<Vector2d>& getStoredVelocities() const;
    virtual double getRadius() const;
    virtual double getKickDistance() const {return kickDistance;};

    // --- Setters ---
    virtual void setAlly(bool is);
    virtual void setPositioned(bool is);
    virtual void setOriented(bool is);
    virtual void setAligned(bool is);
    virtual void setKicker(bool kicker);
    virtual void setId(int id);
    virtual void setPosition(const Point& p);
    virtual void setYaw(double y);
    virtual void setVelocity(const Vector2d& v);
    virtual void setVyaw(double v);
    virtual void setDetected(bool d);
    virtual void setRole(enum role r);
    virtual void setStoredVelocities(const std::deque<Vector2d>& vels);

};

#endif //ROBOT_H
