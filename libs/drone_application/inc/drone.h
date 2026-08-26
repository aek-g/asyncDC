#ifndef ASYNCDC_DRONE_H
#define ASYNCDC_DRONE_H
#include <cstdint>

// Namespace for use protection in larger scopes
namespace drone {
    // Enum for current drone state
    enum class DroneState : uint8_t {
        DISARMED, // Disarmed, not in air
        ARMING, // Takeoff
        IDLE, // Armed, but idle
        GOTO, // Moving to GoTo
        LANDING, // Landing
    };

    // Enum for Dronecontroller function responses, for debugging via LOG
    enum class RCode : uint8_t {
        OK,
        ERR_INVALID,
        ERR_GEOFENCE,
    };

    // Position struct
    struct Position {
        float X = 0.0f;
        float Y = 0.0f;
        float alt = 0.0f;
    };

    // Simple rectangular geofence
    struct Geofence {
        float minX, maxX, minY, maxY;
    };

    // Dronecontroller class
    class DroneController {
        DroneState state = DroneState::DISARMED;
        Position position;
        Position target;
        Geofence geofence;
    public:
        // Arm and land functions
        RCode arm();
        RCode land();
        // Goto function
        RCode goTo(float x, float y);

        // Getters for position and state
        [[nodiscard]] Position getPosition() const;
        [[nodiscard]] DroneState getState() const;

        // Update function
        void update(float deltaSeconds);

        // Constructor with default geofence
        explicit DroneController(Geofence geofence = Geofence{-100, 100, -100, 100});
    };

}

#endif //ASYNCDC_DRONE_H
