#include <gtest/gtest.h>
#include "drone.h"
#include <cmath>

using namespace drone;
static Geofence small(-10.0f, 10.0f, -10.0f, 10.0f);


// Constructor tests

// Default Constructor
TEST(ConstructorTest, DefaultConstructor) {
    DroneController drone;
    EXPECT_EQ(drone.getState(), DroneState::DISARMED);
    Position position = drone.getPosition();
    EXPECT_FLOAT_EQ(position.X, 0.0f);
    EXPECT_FLOAT_EQ(position.Y, 0.0f);
    EXPECT_FLOAT_EQ(position.alt, 0.0f);
}

// Custom Geofence constructor
TEST(ConstructorTest, GeofenceConstructor) {
    DroneController drone(small);
    EXPECT_EQ(drone.getState(), DroneState::DISARMED);
    drone.arm();
    EXPECT_EQ(drone.getState(), DroneState::ARMING);
    drone.update(10.0f);
    EXPECT_EQ(drone.getState(), DroneState::IDLE);
    RCode resultGeo = drone.goTo(20,5);
    EXPECT_EQ(resultGeo, RCode::ERR_GEOFENCE);
    RCode resultOK = drone.goTo(9,5);
    EXPECT_EQ(resultOK, RCode::OK);
}

// Arm tests

// Successful arming from disarmed
TEST(ArmTest, SucceedsFromDisarmed) {
    DroneController drone;
    EXPECT_EQ(drone.arm(), RCode::OK);
    EXPECT_EQ(drone.getState(), DroneState::ARMING);
}

// While arming, reject additional arm command
TEST(ArmTest, RejectedWhileArming) {
    DroneController drone;
    drone.arm();
    EXPECT_EQ(drone.arm(), RCode::ERR_INVALID);
}
// While idle, reject arm command
TEST(ArmTest, RejectedWhileIdle) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    ASSERT_EQ(drone.getState(), DroneState::IDLE);
    EXPECT_EQ(drone.arm(), RCode::ERR_INVALID);
}

// While GoTo, reject arm command
TEST(ArmTest, RejectedWhileGoTo) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.goTo(50.0f, 50.0f);
    ASSERT_EQ(drone.getState(), DroneState::GOTO);
    EXPECT_EQ(drone.arm(), RCode::ERR_INVALID);
}

// While landing, reject arm command
TEST(ArmTest, RejectedWhileLanding) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.land();
    ASSERT_EQ(drone.getState(), DroneState::LANDING);
    EXPECT_EQ(drone.arm(), RCode::ERR_INVALID);
}

// When arming, drone starts to climb
TEST(ArmTest, SetsClimbInMotion) {
    DroneController drone;
    drone.arm();
    drone.update(0.1f);
    EXPECT_GT(drone.getPosition().alt, 0.0f);
}

// When drone is outside of geofence, arm command rejected
TEST(ArmTest, RejectedGeofence) {
    Geofence outside{50.0f, 100.0f, 50.0f, 100.0f};
    DroneController drone(outside);

    EXPECT_EQ(drone.arm(), RCode::ERR_GEOFENCE);
    EXPECT_EQ(drone.getState(), DroneState::DISARMED);
}

// Land commands

// Land rejected when disarmed
TEST(LandTest, RejectedWhileDisarmed) {
    DroneController drone;
    EXPECT_EQ(drone.land(), RCode::ERR_INVALID);
}

// While drone is already landing, reject land command
TEST(LandTest, RejectedWhileLanding) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.land();
    ASSERT_EQ(drone.getState(), DroneState::LANDING);
    EXPECT_EQ(drone.land(), RCode::ERR_INVALID);
}

// While arming, land command accepted
TEST(LandTest, SucceedsFromArming) {
    DroneController drone;
    drone.arm();
    EXPECT_EQ(drone.getState(), DroneState::ARMING);
    EXPECT_EQ(drone.land(), RCode::OK);
    EXPECT_EQ(drone.getState(), DroneState::LANDING);
}

// While idle, land command accepted
TEST(LandTest, SucceedsFromIdle) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    ASSERT_EQ(drone.getState(), DroneState::IDLE);
    EXPECT_EQ(drone.land(), RCode::OK);
    EXPECT_EQ(drone.getState(), DroneState::LANDING);
}

// While GoTo, land command accepted
TEST(LandTest, SucceedsFromGoTo) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.goTo(50.0f, 50.0f);
    ASSERT_EQ(drone.getState(), DroneState::GOTO);
    EXPECT_EQ(drone.land(), RCode::OK);
    EXPECT_EQ(drone.getState(), DroneState::LANDING);
}

// GoTo tests

// Rejected when disarmed
TEST(GoToTest, RejectedWhileDisarmed) {
    DroneController drone;
    EXPECT_EQ(drone.goTo(5.0f, 5.0f), RCode::ERR_INVALID);
}

// Rejected while arming
TEST(GoToTest, RejectedWhileArming) {
    DroneController drone;
    drone.arm();
    EXPECT_EQ(drone.goTo(5.0f, 5.0f), RCode::ERR_INVALID);
}

// Rejected while landing
TEST(GoToTest, RejectedWhileLanding) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.land();
    EXPECT_EQ(drone.goTo(5.0f, 5.0f), RCode::ERR_INVALID);
}

// Accepted when idle
TEST(GoToTest, SucceedsWhileIdle) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    ASSERT_EQ(drone.getState(), DroneState::IDLE);
    EXPECT_EQ(drone.goTo(5.0f, 5.0f), RCode::OK);
    EXPECT_EQ(drone.getState(), DroneState::GOTO);
}

// While on GoTo, additional GoTo command is accepted
TEST(GoToTest, Retarget) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.goTo(5.0f, 5.0f);
    ASSERT_EQ(drone.getState(), DroneState::GOTO);
    EXPECT_EQ(drone.goTo(50.0f, 50.0f), RCode::OK);
    EXPECT_EQ(drone.getState(), DroneState::GOTO);
}

// Retarget outside geofence maintains current target
TEST(GoToTest, RejectRetarget) {
    DroneController drone(small);
    drone.arm();
    drone.update(10.0f);
    drone.goTo(5.0f, 0.0f);
    ASSERT_EQ(drone.getState(), DroneState::GOTO);
    EXPECT_EQ(drone.goTo(50.0f, 50.0f), RCode::ERR_GEOFENCE);
    drone.update(5.0f);
    Position pos = drone.getPosition();
    EXPECT_NEAR(pos.X, 5.0f, 0.001f);
    EXPECT_NEAR(pos.Y, 0.0f, 0.001f);
    EXPECT_EQ(drone.getState(), DroneState::IDLE);
}

// GoTo target at current position ok, not NaN
TEST(GoToTest, CurrentPosition) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    EXPECT_EQ(drone.goTo(0.0f, 0.0f), RCode::OK);
    drone.update(0.1f);
    Position pos = drone.getPosition();
    EXPECT_FALSE(std::isnan(pos.X));
    EXPECT_FALSE(std::isnan(pos.Y));
    EXPECT_EQ(drone.getState(), DroneState::IDLE);
}

// GoTo accepted to geofence boundary values
TEST(GoToTest, AcceptedOnBoundary) {
    DroneController drone(small);
    drone.arm();
    drone.update(10.0f);
    EXPECT_EQ(drone.goTo(small.minX, 0.0f), RCode::OK);
    EXPECT_EQ(drone.goTo(small.maxX, 0.0f), RCode::OK);
    EXPECT_EQ(drone.goTo(0.0f, small.minY), RCode::OK);
    EXPECT_EQ(drone.goTo(0.0f, small.maxY), RCode::OK);
}

// Rejected just outside of geofence minX
TEST(GoToTest, RejectedOutsideMinX) {
    DroneController drone(small);
    drone.arm();
    drone.update(10.0f);
    EXPECT_EQ(drone.goTo(small.minX - 1.0f, 0.0f), RCode::ERR_GEOFENCE);
}

// Rejected just outside of geofence maxX
TEST(GoToTest, RejectedOutsideMaxX) {
    DroneController drone(small);
    drone.arm();
    drone.update(10.0f);
    EXPECT_EQ(drone.goTo(small.maxX + 1.0f, 0.0f), RCode::ERR_GEOFENCE);
}

// Rejected just outside of geofence minY
TEST(GoToTest, RejectedOutsideMinY) {
    DroneController drone(small);
    drone.arm();
    drone.update(10.0f);
    EXPECT_EQ(drone.goTo(0.0f, small.minY - 1.0f), RCode::ERR_GEOFENCE);
}

// Rejected just outside of geofence maxY
TEST(GoToTest, RejectedOutsideMaxY) {
    DroneController drone(small);
    drone.arm();
    drone.update(10.0f);
    EXPECT_EQ(drone.goTo(0.0f, small.maxY + 1.0f), RCode::ERR_GEOFENCE);
}

// Update loop

// When updated, position stays at default when disarmed
TEST(UpdateTest, PosWhileDisarmed) {
    DroneController drone;
    drone.update(5.0f);
    EXPECT_EQ(drone.getState(), DroneState::DISARMED);
    Position pos = drone.getPosition();
    EXPECT_FLOAT_EQ(pos.X, 0.0f);
    EXPECT_FLOAT_EQ(pos.Y, 0.0f);
    EXPECT_FLOAT_EQ(pos.alt, 0.0f);
}

// When updated, position stays fixed when idle
TEST(UpdateTest, PosWhileIdle) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f); // reach IDLE
    Position before = drone.getPosition();
    drone.update(5.0f); // should not change anything further
    Position after = drone.getPosition();
    EXPECT_FLOAT_EQ(before.alt, after.alt);
    EXPECT_EQ(drone.getState(), DroneState::IDLE);
}

// Update with 0 tick delta
TEST(UpdateTest, ZeroDelta) {
    DroneController drone;
    drone.arm();
    drone.update(5.0f);
    Position before = drone.getPosition();
    DroneState stateBefore = drone.getState();
    drone.update(0.0f);
    EXPECT_FLOAT_EQ(drone.getPosition().alt, before.alt);
    EXPECT_EQ(drone.getState(), stateBefore);
}

// Update while arming

// Single update tick while arming
TEST(UpdateArmingTest, SingleTick) {
    DroneController drone;
    drone.arm();
    drone.update(0.1f); // 1m
    EXPECT_FLOAT_EQ(drone.getPosition().alt, 1.0f);
    EXPECT_EQ(drone.getState(), DroneState::ARMING);
}

// Multiple update ticks while arming result in correct values
TEST(UpdateArmingTest, MultipleTicks) {
    DroneController drone;
    drone.arm();
    for (int i = 0; i < 10; ++i) {
        drone.update(0.1f);
    }
    EXPECT_FLOAT_EQ(drone.getPosition().alt, 10.0f);
    EXPECT_EQ(drone.getState(), DroneState::ARMING);
}

// Update for exactly 20m (20 ticks)
TEST(UpdateArmingTest, ExactArmingComplete) {
    DroneController drone;
    drone.arm();
    for (int i = 0; i < 20; ++i) {
        drone.update(0.1f);
    }
    EXPECT_FLOAT_EQ(drone.getPosition().alt, 20.0f);
    EXPECT_EQ(drone.getState(), DroneState::IDLE);
}

// Longer update loop results in correct values
TEST(UpdateArmingTest, LongUpdate) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    EXPECT_FLOAT_EQ(drone.getPosition().alt, 20.0f);
    EXPECT_EQ(drone.getState(), DroneState::IDLE);
}

// Update while landing

// Landing accumulates to correct alt
TEST(UpdateLandingTest, LandAccumulates) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.land();
    for (int i = 0; i < 10; ++i) {
        drone.update(0.1f);
    }
    EXPECT_FLOAT_EQ(drone.getPosition().alt, 14.0f);
    EXPECT_EQ(drone.getState(), DroneState::LANDING);
}

// Exact arrival time snaps to target alt and disarms
TEST(UpdateLandingTest, ExactArrivalSnaps) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.land();
    drone.update(17.0f / 6.0f);
    drone.update(3.0f);
    EXPECT_FLOAT_EQ(drone.getPosition().alt, 0.0f);
    EXPECT_EQ(drone.getState(), DroneState::DISARMED);
}

// Descending starts at 6m/s
TEST(UpdateLandingTest, DescentStartSix) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.land();
    drone.update(0.1f);
    EXPECT_FLOAT_EQ(drone.getPosition().alt, 19.4f);
    EXPECT_EQ(drone.getState(), DroneState::LANDING);
}

// Descent to exactly 3m
TEST(UpdateLandingTest, ThreeMeters) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.land();
    drone.update(17.0f / 6.0f);
    EXPECT_NEAR(drone.getPosition().alt, 3.0f, 0.0001f);
    EXPECT_EQ(drone.getState(), DroneState::LANDING);
}

// Slows to 1m/s when below 3m
TEST(UpdateLandingTest, BelowThreeMeters) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.land();
    drone.update(17.0f / 6.0f);
    drone.update(0.1f);
    EXPECT_NEAR(drone.getPosition().alt, 2.9f, 0.0001f);
    EXPECT_EQ(drone.getState(), DroneState::LANDING);
}

// Lands correctly from mid climb during arming
TEST(UpdateLandingTest, WhileArming) {
    DroneController drone;
    drone.arm();
    drone.update(0.5f);
    ASSERT_EQ(drone.getState(), DroneState::ARMING);
    ASSERT_FLOAT_EQ(drone.getPosition().alt, 5.0f);

    drone.land();
    ASSERT_EQ(drone.getState(), DroneState::LANDING);
    drone.update(2.0f / 6.0f);
    EXPECT_NEAR(drone.getPosition().alt, 3.0f, 0.0001f);
    EXPECT_EQ(drone.getState(), DroneState::LANDING);

    drone.update(3.0f);
    EXPECT_FLOAT_EQ(drone.getPosition().alt, 0.0f);
    EXPECT_EQ(drone.getState(), DroneState::DISARMED);
}

// Long update tick results in correct values
TEST(UpdateLandingTest, LongUpdate) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.land();
    drone.update(10.0f);
    EXPECT_FLOAT_EQ(drone.getPosition().alt, 0.0f);
    EXPECT_EQ(drone.getState(), DroneState::DISARMED);
}

// Update while GoTo

// Correct position on route to diagonal target
TEST(UpdateGoToTest, DiagonalMovement) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.goTo(30.0f, 40.0f);
    drone.update(1.0f);

    Position pos = drone.getPosition();
    EXPECT_NEAR(pos.X, 6.0f, 0.001f);
    EXPECT_NEAR(pos.Y, 8.0f, 0.001f);
    EXPECT_EQ(drone.getState(), DroneState::GOTO);
}

// Exact arrival at diagonal target
TEST(UpdateGoToTest, ExactArrival) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.goTo(30.0f, 40.0f);
    drone.update(5.0f);

    Position pos = drone.getPosition();
    EXPECT_FLOAT_EQ(pos.X, 30.0f);
    EXPECT_FLOAT_EQ(pos.Y, 40.0f);
    EXPECT_EQ(drone.getState(), DroneState::IDLE);
}

// Long update time results in correct position and status
TEST(UpdateGoToTest, LongUpdate) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.goTo(5.0f, 5.0f);
    drone.update(100.0f);

    Position pos = drone.getPosition();
    EXPECT_FLOAT_EQ(pos.X, 5.0f);
    EXPECT_FLOAT_EQ(pos.Y, 5.0f);
    EXPECT_EQ(drone.getState(), DroneState::IDLE);
}

// Handles negative coordinates correctly
TEST(UpdateGoToTest, NegativeCoordinates) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.goTo(-30.0f, -40.0f);
    drone.update(5.0f);

    Position pos = drone.getPosition();
    EXPECT_FLOAT_EQ(pos.X, -30.0f);
    EXPECT_FLOAT_EQ(pos.Y, -40.0f);
    EXPECT_EQ(drone.getState(), DroneState::IDLE);
}

// Correct values when crossing zero
TEST(UpdateGoToTest, CrossingZero) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.goTo(-10.0f, -10.0f);
    drone.update(2.0f);
    ASSERT_EQ(drone.getState(), DroneState::IDLE);

    drone.goTo(10.0f, 10.0f);
    drone.update(10.0f);
    Position pos = drone.getPosition();
    EXPECT_NEAR(pos.X, 10.0f, 0.01f);
    EXPECT_NEAR(pos.Y, 10.0f, 0.01f);
    EXPECT_EQ(drone.getState(), DroneState::IDLE);
}

// Target very close to position does not result in NaN
TEST(UpdateGoToTest, VeryCloseTarget) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.goTo(0.0001f, 0.0001f);
    drone.update(0.1f);

    Position pos = drone.getPosition();
    EXPECT_FALSE(std::isnan(pos.X));
    EXPECT_FALSE(std::isnan(pos.Y));
    EXPECT_EQ(drone.getState(), DroneState::IDLE);
}

// Getters

// Get position returns copy, not a refrence
TEST(GetterTest, GetPositionReturnsCopy) {
    DroneController drone;
    drone.arm();
    drone.update(0.5f);

    Position pos = drone.getPosition();
    pos.alt = 999.0f;

    Position posAgain = drone.getPosition();
    EXPECT_NE(posAgain.alt, 999.0f);
}

// Getstate returns correct state while transitioning between states
TEST(GetterTest, GetStateTransition) {
    DroneController drone;
    EXPECT_EQ(drone.getState(), DroneState::DISARMED);

    drone.arm();
    EXPECT_EQ(drone.getState(), DroneState::ARMING);

    drone.update(10.0f);
    EXPECT_EQ(drone.getState(), DroneState::IDLE);

    drone.goTo(5.0f, 5.0f);
    EXPECT_EQ(drone.getState(), DroneState::GOTO);

    drone.update(10.0f);
    EXPECT_EQ(drone.getState(), DroneState::IDLE);

    drone.land();
    EXPECT_EQ(drone.getState(), DroneState::LANDING);

    drone.update(10.0f);
    EXPECT_EQ(drone.getState(), DroneState::DISARMED);
}

// Full lifecycle

TEST(LifecycleTest, FullMissionFromArmToLand) {
    DroneController drone;

    // Arm and climb
    ASSERT_EQ(drone.arm(), RCode::OK);
    ASSERT_EQ(drone.getState(), DroneState::ARMING);
    for (int i = 0; i < 20; ++i) drone.update(0.1f);
    ASSERT_EQ(drone.getState(), DroneState::IDLE);
    ASSERT_FLOAT_EQ(drone.getPosition().alt, 20.0f);

    // GoTo target
    ASSERT_EQ(drone.goTo(30.0f, 40.0f), RCode::OK);
    ASSERT_EQ(drone.getState(), DroneState::GOTO);
    for (int i = 0; i < 51; ++i) drone.update(0.1f);
    ASSERT_EQ(drone.getState(), DroneState::IDLE);
    Position afterGoTo = drone.getPosition();
    ASSERT_FLOAT_EQ(afterGoTo.X, 30.0f);
    ASSERT_FLOAT_EQ(afterGoTo.Y, 40.0f);
    ASSERT_FLOAT_EQ(afterGoTo.alt, 20.0f);

    // Land
    ASSERT_EQ(drone.land(), RCode::OK);
    ASSERT_EQ(drone.getState(), DroneState::LANDING);
    drone.update(17.0f / 6.0f);
    ASSERT_EQ(drone.getState(), DroneState::LANDING);
    drone.update(3.0f);
    ASSERT_EQ(drone.getState(), DroneState::DISARMED);
    Position final = drone.getPosition();
    ASSERT_FLOAT_EQ(final.alt, 0.0f);
    ASSERT_FLOAT_EQ(final.X, 30.0f);
    ASSERT_FLOAT_EQ(final.Y, 40.0f);
}