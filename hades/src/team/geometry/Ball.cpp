//
// Created by caiu on 21/08/25.
//

#include "Ball.h"
#include <cmath>

// --- Setters e Getters ---
void Ball::setVelocity(const Vector2d& v) {
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
    stopPosition = getStopPosition();
}

Vector2d Ball::getVelocity() const {
    return velocity;
}

void Ball::setDetected(bool d) {
    detected = d;
}

bool Ball::getDetected() {
    return detected;
}

void Ball::setPosition(Point p) {
    position = p;
    stopPosition = getStopPosition();
}

Point Ball::getPosition() {
    return position;
}

Point Ball::getStopPosition() const {
    //TODO melhorar precisao

    if (velocity.getX() == 0 && velocity.getY() == 0) {
        return position;
    }

    // Módulo da velocidade inicial
    double v0 = 1000*std::sqrt(velocity.getX() * velocity.getX() +
                          velocity.getY() * velocity.getY());

    // Distância até parar (Torricelli)
    double distance = (v0 * v0) / (2 * deceleration);

    // Direção da velocidade (vetor normalizado)
    double dirX = velocity.getX() / v0;
    double dirY = velocity.getY() / v0;

    // Posição final = posição atual + deslocamento
    return Point(position.getX() + dirX * distance,
                 position.getY() + dirY * distance);
}

bool Ball::isStopped() {
    return getVelocity().getNorm() < velocityThreshold;
}

bool Ball::isMoving() {
    return !isStopped();
}

LineSegment Ball::getMovementLine() const {
    return LineSegment(position, getStopPosition());
}