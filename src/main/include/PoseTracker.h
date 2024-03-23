// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>

#include <deque>
#include <functional>

class PoseTracker {
   public:
    PoseTracker(int bufferSize = 30);
    units::meters_per_second_squared_t getAverageAcceleration();
    units::meters_per_second_t getAverageVelocity();
    units::degrees_per_second_t getAverageAngularVelocity();
    void addReading(frc::Pose2d pose, units::second_t timestamp);

   private:
    void resetBuffers(uint count);
    std::deque<std::pair<frc::Pose2d, units::second_t>> poseBuffer;
    std::deque<std::pair<units::meters_per_second_t, units::second_t>> velBuffer;
    std::deque<std::pair<units::meters_per_second_squared_t, units::second_t>> accelBuffer;
    std::deque<std::pair<units::degrees_per_second_t, units::second_t>> angVelBuffer;
    double bufferSize;
};
