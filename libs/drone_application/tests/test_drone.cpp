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

// arm()


TEST(ArmTest, SucceedsFromDisarmed) {
    DroneController drone;
    EXPECT_EQ(drone.arm(), RCode::OK);
    EXPECT_EQ(drone.getState(), DroneState::ARMING);
}

TEST(ArmTest, RejectedWhileArming) {
    DroneController drone;
    drone.arm();
    EXPECT_EQ(drone.arm(), RCode::ERR_INVALID);
}

TEST(ArmTest, RejectedWhileIdle) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    ASSERT_EQ(drone.getState(), DroneState::IDLE);
    EXPECT_EQ(drone.arm(), RCode::ERR_INVALID);
}

TEST(ArmTest, RejectedWhileGoTo) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.goTo(50.0f, 50.0f);
    ASSERT_EQ(drone.getState(), DroneState::GOTO);
    EXPECT_EQ(drone.arm(), RCode::ERR_INVALID);
}

TEST(ArmTest, RejectedWhileLanding) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.land();
    ASSERT_EQ(drone.getState(), DroneState::LANDING);
    EXPECT_EQ(drone.arm(), RCode::ERR_INVALID);
}

TEST(ArmTest, SetsClimbInMotion) {
    DroneController drone;
    drone.arm();
    drone.update(0.1f);
    EXPECT_GT(drone.getPosition().alt, 0.0f);
}

TEST(ArmTest, RejectedGeofence) {
    // Geofence that is outside the default drone coords
    Geofence outside{50.0f, 100.0f, 50.0f, 100.0f};
    DroneController drone(outside);

    EXPECT_EQ(drone.arm(), RCode::ERR_GEOFENCE);
    EXPECT_EQ(drone.getState(), DroneState::DISARMED);
}

// land()

TEST(LandTest, RejectedWhileDisarmed) {
    DroneController drone;
    EXPECT_EQ(drone.land(), RCode::ERR_INVALID);
}

TEST(LandTest, RejectedWhileAlreadyLanding) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.land();
    ASSERT_EQ(drone.getState(), DroneState::LANDING);
    EXPECT_EQ(drone.land(), RCode::ERR_INVALID);
}

TEST(LandTest, SucceedsFromArming) {
    DroneController drone;
    drone.arm();
    EXPECT_EQ(drone.getState(), DroneState::ARMING);
    EXPECT_EQ(drone.land(), RCode::OK);
    EXPECT_EQ(drone.getState(), DroneState::LANDING);
}

TEST(LandTest, SucceedsFromIdle) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    ASSERT_EQ(drone.getState(), DroneState::IDLE);
    EXPECT_EQ(drone.land(), RCode::OK);
    EXPECT_EQ(drone.getState(), DroneState::LANDING);
}

TEST(LandTest, SucceedsFromGoTo) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.goTo(50.0f, 50.0f);
    ASSERT_EQ(drone.getState(), DroneState::GOTO);
    EXPECT_EQ(drone.land(), RCode::OK);
    EXPECT_EQ(drone.getState(), DroneState::LANDING);
}

// goTo()

TEST(GoToTest, RejectedWhileDisarmed) {
    DroneController drone;
    EXPECT_EQ(drone.goTo(5.0f, 5.0f), RCode::ERR_INVALID);
}

TEST(GoToTest, RejectedWhileArming) {
    DroneController drone;
    drone.arm();
    EXPECT_EQ(drone.goTo(5.0f, 5.0f), RCode::ERR_INVALID);
}

TEST(GoToTest, RejectedWhileLanding) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.land();
    EXPECT_EQ(drone.goTo(5.0f, 5.0f), RCode::ERR_INVALID);
}

TEST(GoToTest, SucceedsWhileIdle) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    ASSERT_EQ(drone.getState(), DroneState::IDLE);
    EXPECT_EQ(drone.goTo(5.0f, 5.0f), RCode::OK);
    EXPECT_EQ(drone.getState(), DroneState::GOTO);
}

TEST(GoToTest, Retarget) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.goTo(5.0f, 5.0f);
    ASSERT_EQ(drone.getState(), DroneState::GOTO);
    EXPECT_EQ(drone.goTo(50.0f, 50.0f), RCode::OK);
    EXPECT_EQ(drone.getState(), DroneState::GOTO);
}

TEST(GoToTest, AcceptedExactlyOnEachBoundary) {
    DroneController drone(small);
    drone.arm();
    drone.update(10.0f);
    EXPECT_EQ(drone.goTo(small.minX, 0.0f), RCode::OK);
    EXPECT_EQ(drone.goTo(small.maxX, 0.0f), RCode::OK);
    EXPECT_EQ(drone.goTo(0.0f, small.minY), RCode::OK);
    EXPECT_EQ(drone.goTo(0.0f, small.maxY), RCode::OK);
}

TEST(GoToTest, RejectedJustOutsideMinX) {
    DroneController drone(small);
    drone.arm();
    drone.update(10.0f);
    EXPECT_EQ(drone.goTo(small.minX - 1.0f, 0.0f), RCode::ERR_GEOFENCE);
}

TEST(GoToTest, RejectedJustOutsideMaxX) {
    DroneController drone(small);
    drone.arm();
    drone.update(10.0f);
    EXPECT_EQ(drone.goTo(small.maxX + 1.0f, 0.0f), RCode::ERR_GEOFENCE);
}

TEST(GoToTest, RejectedJustOutsideMinY) {
    DroneController drone(small);
    drone.arm();
    drone.update(10.0f);
    EXPECT_EQ(drone.goTo(0.0f, small.minY - 1.0f), RCode::ERR_GEOFENCE);
}

TEST(GoToTest, RejectedJustOutsideMaxY) {
    DroneController drone(small);
    drone.arm();
    drone.update(10.0f);
    EXPECT_EQ(drone.goTo(0.0f, small.maxY + 1.0f), RCode::ERR_GEOFENCE);
}

// update()

TEST(UpdateTest, NoOpWhileDisarmed) {
    DroneController drone;
    drone.update(5.0f);
    EXPECT_EQ(drone.getState(), DroneState::DISARMED);
    Position pos = drone.getPosition();
    EXPECT_FLOAT_EQ(pos.X, 0.0f);
    EXPECT_FLOAT_EQ(pos.Y, 0.0f);
    EXPECT_FLOAT_EQ(pos.alt, 0.0f);
}

TEST(UpdateTest, NoOpWhileIdle) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f); // reach IDLE
    Position before = drone.getPosition();
    drone.update(5.0f); // should not change anything further
    Position after = drone.getPosition();
    EXPECT_FLOAT_EQ(before.alt, after.alt);
    EXPECT_EQ(drone.getState(), DroneState::IDLE);
}

// update() while arming

TEST(UpdateArmingTest, SingleTickIncreasesAltitudeByExactAmount) {
    DroneController drone;
    drone.arm();
    drone.update(0.1f); // 1m
    EXPECT_FLOAT_EQ(drone.getPosition().alt, 1.0f);
    EXPECT_EQ(drone.getState(), DroneState::ARMING);
}

TEST(UpdateArmingTest, MultipleTicksAccumulateCorrectly) {
    DroneController drone;
    drone.arm();
    for (int i = 0; i < 10; ++i) {
        drone.update(0.1f); // 1.0m per tick, 10 ticks = 10.0m
    }
    EXPECT_FLOAT_EQ(drone.getPosition().alt, 10.0f);
    EXPECT_EQ(drone.getState(), DroneState::ARMING);
}

TEST(UpdateArmingTest, ExactArrivalSnapsAndTransitionsToIdle) {
    DroneController drone;
    drone.arm();
    for (int i = 0; i < 20; ++i) {
        drone.update(0.1f); // 20 ticks * 1.0m = exactly 20.0m
    }
    EXPECT_FLOAT_EQ(drone.getPosition().alt, 20.0f);
    EXPECT_EQ(drone.getState(), DroneState::IDLE);
}

TEST(UpdateArmingTest, LargeDeltaOvershootSnapsInOneTick) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f); // 100m worth of climb capacity in one tick
    EXPECT_FLOAT_EQ(drone.getPosition().alt, 20.0f);
    EXPECT_EQ(drone.getState(), DroneState::IDLE);
}

// update() while landing

TEST(UpdateLandingTest, GradualDescentAccumulatesCorrectly) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f); // reach 20m, IDLE
    drone.land();
    for (int i = 0; i < 10; ++i) {
        drone.update(0.1f); // 1.0m per tick, 10 ticks = 10.0m descended
    }
    EXPECT_FLOAT_EQ(drone.getPosition().alt, 10.0f);
    EXPECT_EQ(drone.getState(), DroneState::LANDING);
}

TEST(UpdateLandingTest, ExactArrivalSnapsAndTransitionsToDisarmed) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f); // 20m
    drone.land();
    for (int i = 0; i < 20; ++i) {
        drone.update(0.1f); // 20 ticks * 1.0m = exactly 20.0m descent
    }
    EXPECT_FLOAT_EQ(drone.getPosition().alt, 0.0f);
    EXPECT_EQ(drone.getState(), DroneState::DISARMED);
}

TEST(UpdateLandingTest, LargeDeltaOvershootSnapsInOneTick) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f); // 20m
    drone.land();
    drone.update(10.0f); // plenty to reach 0m in one tick
    EXPECT_FLOAT_EQ(drone.getPosition().alt, 0.0f);
    EXPECT_EQ(drone.getState(), DroneState::DISARMED);
}

// update() while goTo()

TEST(UpdateGoToTest, GradualDiagonalMovementIsProportional) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.goTo(30.0f, 40.0f); // 3-4-5 triangle scaled: distance = 50
    drone.update(1.0f); // 10m of travel, 1/5 of the way there

    Position pos = drone.getPosition();
    // Direction is (30/50, 40/50) = (0.6, 0.8); 10m along that direction:
    EXPECT_NEAR(pos.X, 6.0f, 0.001f);
    EXPECT_NEAR(pos.Y, 8.0f, 0.001f);
    EXPECT_EQ(drone.getState(), DroneState::GOTO);
}

TEST(UpdateGoToTest, ExactArrivalSnapsAndTransitionsToIdle) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.goTo(30.0f, 40.0f); // distance = 50
    drone.update(5.0f); // exactly 50m of travel in one tick

    Position pos = drone.getPosition();
    EXPECT_FLOAT_EQ(pos.X, 30.0f);
    EXPECT_FLOAT_EQ(pos.Y, 40.0f);
    EXPECT_EQ(drone.getState(), DroneState::IDLE);
}

TEST(UpdateGoToTest, LargeDeltaOvershootSnapsInOneTick) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.goTo(5.0f, 5.0f);
    drone.update(100.0f); // wildly more than enough

    Position pos = drone.getPosition();
    EXPECT_FLOAT_EQ(pos.X, 5.0f);
    EXPECT_FLOAT_EQ(pos.Y, 5.0f);
    EXPECT_EQ(drone.getState(), DroneState::IDLE);
}

TEST(UpdateGoToTest, HandlesNegativeTargetCoordinates) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.goTo(-30.0f, -40.0f); // distance = 50
    drone.update(5.0f); // exact arrival

    Position pos = drone.getPosition();
    EXPECT_FLOAT_EQ(pos.X, -30.0f);
    EXPECT_FLOAT_EQ(pos.Y, -40.0f);
    EXPECT_EQ(drone.getState(), DroneState::IDLE);
}

TEST(UpdateGoToTest, HandlesCrossingZeroOnBothAxes) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    // Manually walk the drone to a negative starting position first via goTo
    drone.goTo(-10.0f, -10.0f);
    drone.update(2.0f); // dist = sqrt(200) ~= 14.14, well within 20m travel -> arrives
    ASSERT_EQ(drone.getState(), DroneState::IDLE);

    drone.goTo(10.0f, 10.0f); // now cross zero to a positive target
    drone.update(10.0f); // plenty to reach it
    Position pos = drone.getPosition();
    EXPECT_NEAR(pos.X, 10.0f, 0.01f);
    EXPECT_NEAR(pos.Y, 10.0f, 0.01f);
    EXPECT_EQ(drone.getState(), DroneState::IDLE);
}

TEST(UpdateGoToTest, VeryCloseTargetDoesNotProduceNaN) {
    DroneController drone;
    drone.arm();
    drone.update(10.0f);
    drone.goTo(0.0001f, 0.0001f); // extremely close to current position (0,0)
    drone.update(0.1f);

    Position pos = drone.getPosition();
    EXPECT_FALSE(std::isnan(pos.X));
    EXPECT_FALSE(std::isnan(pos.Y));
    EXPECT_EQ(drone.getState(), DroneState::IDLE);
}

// Getters

TEST(GetterTest, GetPositionReturnsACopyNotAReference) {
    DroneController drone;
    drone.arm();
    drone.update(0.5f);

    Position pos = drone.getPosition();
    pos.alt = 999.0f; // mutate the returned copy

    Position posAgain = drone.getPosition();
    EXPECT_NE(posAgain.alt, 999.0f) << "getPosition() must return a copy, not internal state";
}

TEST(GetterTest, GetStateReflectsEachTransitionImmediately) {
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

    // 1. Arm and climb to 20m
    ASSERT_EQ(drone.arm(), RCode::OK);
    ASSERT_EQ(drone.getState(), DroneState::ARMING);
    for (int i = 0; i < 20; ++i) drone.update(0.1f);
    ASSERT_EQ(drone.getState(), DroneState::IDLE);
    ASSERT_FLOAT_EQ(drone.getPosition().alt, 20.0f);

    // 2. GoTo a target
    ASSERT_EQ(drone.goTo(30.0f, 40.0f), RCode::OK); // distance 50
    ASSERT_EQ(drone.getState(), DroneState::GOTO);
    for (int i = 0; i < 51; ++i) drone.update(0.1f); // 50 ticks * 1.0m = 50m
    ASSERT_EQ(drone.getState(), DroneState::IDLE);
    Position afterGoTo = drone.getPosition();
    ASSERT_FLOAT_EQ(afterGoTo.X, 30.0f);
    ASSERT_FLOAT_EQ(afterGoTo.Y, 40.0f);
    ASSERT_FLOAT_EQ(afterGoTo.alt, 20.0f); // altitude unaffected by horizontal movement

    // 3. Land
    ASSERT_EQ(drone.land(), RCode::OK);
    ASSERT_EQ(drone.getState(), DroneState::LANDING);
    for (int i = 0; i < 20; ++i) drone.update(0.1f);
    ASSERT_EQ(drone.getState(), DroneState::DISARMED);
    Position final_ = drone.getPosition();
    ASSERT_FLOAT_EQ(final_.alt, 0.0f);
    // X/Y should remain wherever the drone was when it landed
    ASSERT_FLOAT_EQ(final_.X, 30.0f);
    ASSERT_FLOAT_EQ(final_.Y, 40.0f);
}