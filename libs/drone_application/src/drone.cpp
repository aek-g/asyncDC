#include "drone.h"

#include <cmath>
// Namespace for use protection in larger scopes
namespace drone {
    // Arm function
    RCode DroneController::arm() {
        // Check if drone is already armed
        if (state != DroneState::DISARMED) {
            return RCode::ERR_INVALID;
        }
        // Check if drone is outside of geofence (no arming)
        if (position.X > geofence.maxX || position.X  < geofence.minX || position.Y > geofence.maxY || position.Y < geofence.minY) {
            return RCode::ERR_GEOFENCE;
        }
        // State to ARMING and target alt to 20m
        state = DroneState::ARMING;
        target.alt = 20.0f;
        // Return OK for logging
        return RCode::OK;
    }
    // Land function
    RCode DroneController::land() {
        // Check if drone is disarmed or already landing
        if (state == DroneState::DISARMED || state == DroneState::LANDING) {
            return RCode::ERR_INVALID;
        }
        // State to LANDING and target alt to 0m
        state = DroneState::LANDING;
        target.alt = 0.0f;
        // Return OK for logging
        return RCode::OK;
    }
    // Goto function
    RCode DroneController::goTo(float x, float y) {
        // Check if drone is landing/arming/disarmed
        if (state != DroneState::IDLE && state != DroneState::GOTO) {
            return RCode::ERR_INVALID;
        }
        // Check if target is outside of geofence
        if (x > geofence.maxX || x < geofence.minX || y > geofence.maxY || y < geofence.minY) {
            return RCode::ERR_GEOFENCE;
        }
        // Change target x/y and state
        target.X = x;
        target.Y = y;
        state = DroneState::GOTO;
        // Return OK for logging
        return RCode::OK;
    }

    // Getters for position and state
    [[nodiscard]] Position DroneController::getPosition() const {
        return position;
    }
    [[nodiscard]] DroneState DroneController::getState() const {
        return state;
    }

    // Update function
    void DroneController::update(float deltaSeconds) {

        switch (state) {
            // IDLE or DISARMED - nothing to do
            case DroneState::IDLE:case DroneState::DISARMED: break;
            // ARMING - Increase elevation to target alt
            case DroneState::ARMING: {
                // Distance travelled for deltaseconds
                float distanceT = 10.0f * deltaSeconds;
                // Overshoot guard
                if (position.alt + distanceT < target.alt) {
                    position.alt += distanceT;
                } else {
                    // In case of overshoot or 0.0m remaining
                    position.alt = target.alt;
                    state = DroneState::IDLE;
                }
                break;
            }
            // LANDING - Decrease elevation to target alt
            case DroneState::LANDING: {
                // Distance travelled for deltaseconds
                float distanceT = 10.0f * deltaSeconds;
                // Overshoot guard
                if (position.alt - distanceT > target.alt) {
                    position.alt -= distanceT;
                } else {
                    // In case of overshoot or 0.0m remaining
                    position.alt = target.alt;
                    state = DroneState::DISARMED;
                }
                break;
            }
            case DroneState::GOTO: {
                // Distance travelled for deltaseconds
                float distanceT = 10.0f * deltaSeconds;
                // Calculating individual differences for X and Y
                float dx = target.X - position.X;
                float dy = target.Y - position.Y;
                // Remaining 2D distance
                float remaining = sqrtf(dx * dx + dy * dy);

                //Overshoot guard
                if (distanceT >= remaining) {
                    // In case of overshoot or 0.0m remaining
                    position.X = target.X;
                    position.Y = target.Y;
                    state = DroneState::IDLE;
                    break;
                } else {
                    // Next movement
                    float dirX = dx / remaining;
                    float dirY = dy / remaining;

                    position.X += dirX*distanceT;
                    position.Y += dirY*distanceT;
                    break;
                }
            }
        }
    }

    // Constructor for Drone class
    DroneController::DroneController(Geofence geofence) : geofence(geofence) {}
}
