#include "TeamInfo.h"
#include <stdexcept>

Robot TeamInfo::getRobotofRole(enum Robot::role role) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    for (Robot robot : robots) {
        if (!isRobotActive(robot.getId())) continue;
        if (robot.getRole() == role) return robot;
    }
    throw std::runtime_error("No robot of desired role");
}

Robot TeamInfo::getEnemyofRole(enum Robot::role role, std::array<Robot, 16> enemies) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    for (Robot robot : enemies) {
        if (enemy_roles[robot.getId()] == role) return robot;
    }
    throw std::runtime_error("No robot of desired role");
}

Robot TeamInfo::getRobotToKickTo(RobotController& robot) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    for (Robot r : robots) {
        if (r.getId() != robot.getId() && r.isDetected() && r.getRole() != Robot::goal_keeper)
            return robot;
    }
    throw std::runtime_error("No robot to kick to");
}

double TeamInfo::getStopDistanceToBall() {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return stop_distance_to_ball;
}

double TeamInfo::getStopMaxSpeed() {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return stop_max_speed;
}

int TeamInfo::isRobotActive(int id) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return active_robots[id];
}

void TeamInfo::setRobotActive(int id, bool active) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    active_robots[id] = active;
}

unsigned int TeamInfo::getNumOfActiveRobots() {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    int num = 0;
    for (bool b : active_robots) if (b) num++;
    return num;
}

bool TeamInfo::isPositioned(int id) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return positioned[id];
}

void TeamInfo::setPositioned(int id, bool positioned) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    this->positioned[id] = positioned;
}

RobotController& TeamInfo::getRobotController(int id) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return robots[id];
}

Robot::role TeamInfo::getAllyRole(int id) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return roles[id];
}

void TeamInfo::setAllyRole(int id, Robot::role role) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    roles[id] = role;
    robots[id].setRole(role);
}

Robot::role TeamInfo::getEnemyRole(int id) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return enemy_roles[id];
}

void TeamInfo::setEnemyRole(int id, Robot::role role) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    enemy_roles[id] = role;
}

void TeamInfo::setBallPlacementSpot(Point p) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    ball_placement_spot = p;
}

Point TeamInfo::getBallPlacementSpot() {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return ball_placement_spot;
}

int TeamInfo::getGoalKeeperId() {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return goal_keeper_id;
}

void TeamInfo::setGoalKeeperId(int id) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    goal_keeper_id = id;
}

TeamInfo::sides TeamInfo::getOurSide() {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return our_side;
}

void TeamInfo::setOurSide(sides side) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    our_side = side;
}

bool TeamInfo::isDebug() {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return debug;
}

void TeamInfo::setDebug(bool debug) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    this->debug = debug;
}

TeamInfo::Command TeamInfo::getCurrentCommand() {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return current_command;
}

void TeamInfo::setCurrentCommand(Command command) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    current_command = command;
}

TeamInfo::events TeamInfo::getEvent() {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return event;
}

void TeamInfo::setEvent(events event) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    this->event = event;
}

TeamInfo::colors TeamInfo::getColor() {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return color;
}

void TeamInfo::setColor(colors c) {
    std::lock_guard<std::recursive_mutex> lock(mtx);
    this->color = c;
}
