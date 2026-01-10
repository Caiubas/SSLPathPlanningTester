//
// Created by caiom on 4/7/2025.
//

#include "Robot.h"

// --- Getters ---
bool Robot::isAlly() const {
	return ally;
}

bool Robot::isPositioned() const {
	return positioned;
}

bool Robot::isOriented() const {
	return oriented;
}

bool Robot::isAligned() const {
	return aligned;
}

int Robot::getId() const {
	return id;
}

double Robot::getKickerColddown() const {
	return kicker_colddown;
}

double Robot::getKickerTimer() const {
	return kicker_timer;
}

double Robot::getPushTime() const {
	return push_time;
}

Point Robot::getOldPosition() const {
	return old_pos;
}

Point Robot::getPosition() const {
	return pos;
}

double Robot::getYaw() const {
	return yaw;
}

Vector2d& Robot::getVelocity() {
	return velocity;
}

double Robot::getVyaw() const {
	return vyaw;
}

bool Robot::isDetected() const {
	return detected;
}

enum Robot::role Robot::getRole() const {
	return this_role;
}

const std::deque<Vector2d>& Robot::getStoredVelocities() const {
	return stored_velocities;
}

double Robot::getRadius() const {
	return radius;
}

bool Robot::hasKicker() {
	return kicker;
}

// --- Setters ---

void Robot::setKicker(bool kicker) {
	this->kicker = kicker;
}

void Robot::setAlly(bool is) {
	this->ally = is;
}

void Robot::setPositioned(bool is) {
	this->positioned = is;
}

void Robot::setOriented(bool is) {
	this->oriented = is;
}

void Robot::setAligned(bool is) {
	this->aligned = is;
}

void Robot::setId(int id) {
	this->id = id;
}

void Robot::setKickerColddown(double t) {
	this->kicker_colddown = t;
}

void Robot::setKickerTimer(double t) {
	this->kicker_timer = t;
}

void Robot::setPushTime(double t) {
	this->push_time = t;
}

void Robot::setPosition(const Point& p) {
	old_pos = pos;
	pos = p;
}

void Robot::setYaw(double y) {
	yaw = y;
}

void Robot::setVelocity(const Vector2d& v) {
	if (stored_velocities.size() >= max_velocities_stored) {
		stored_velocities.pop_front();
	}
	stored_velocities.push_back(v);
	double average_x = 0;
	double average_y = 0;
	for (int i = 0; i < stored_velocities.size(); i++) {
		average_x += stored_velocities[i].getX()/stored_velocities.size();
		average_y += stored_velocities[i].getY()/stored_velocities.size();
	}
	velocity = Vector2d(average_x, average_y);
}

void Robot::setVyaw(double v) {
	if (stored_yaw_velocities.size() >= max_yaw_velocities_stored) {
		stored_yaw_velocities.pop_front();
	}
	stored_yaw_velocities.push_back(v);
	double average = 0;
	for (int i = 0; i < stored_yaw_velocities.size(); i++) {
		average += stored_yaw_velocities[i]/stored_yaw_velocities.size();
	}
	vyaw = average;
}

bool Robot::isSpinning() const {
	return (vyaw > yawVelocityThreshold);
}

void Robot::setDetected(bool d) {
	detected = d;
}

void Robot::setRole(enum Robot::role r) {
	this_role = r;
}

bool Robot::isMoving() const {
	if (velocity.getNorm() > velocityThreshold) {
	return true;
	}
	return false;
}

bool Robot::isStopped() const {
	return !isMoving();
}

// --- Stored Velocities ---
void Robot::setStoredVelocities(const std::deque<Vector2d>& vels) {
	stored_velocities = vels;
<<<<<<< HEAD
=======
}


bool Robot::isKickingOnVision() const {
	return kickOnVision;
>>>>>>> 9fa43e16e1cb4304d698b0bfbfc19d3511c7cccf
}