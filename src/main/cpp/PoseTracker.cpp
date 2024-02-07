#include "PoseTracker.h"

PoseTracker::PoseTracker(int _bufferSize) : 
bufferSize(_bufferSize)
{
    
}

void PoseTracker::addReading(frc::Pose2d pose, units::second_t timestamp) {
    if (poseBuffer.size() != 0) {
        if ((poseBuffer.back().first - pose).Translation().Norm() > 1_m){
            resetBuffers(0);
        } 
        else {
            units::meters_per_second_t vel = (pose - poseBuffer.back().first).Translation().Norm() / (timestamp - poseBuffer.back().second);
            if (velBuffer.size() != 0){
                accelBuffer.push_back({
                    (vel - velBuffer.back().first) / (timestamp - velBuffer.back().second),
                    timestamp
                });
            }
            velBuffer.push_back({vel, timestamp});
        }    
    }
    poseBuffer.push_back({pose, timestamp});
    resetBuffers(bufferSize);
}

void PoseTracker::resetBuffers(int count){
    while (poseBuffer.size() > count) poseBuffer.pop_front(); 
    while (velBuffer.size() > count) velBuffer.pop_front(); 
    while (accelBuffer.size() > count) accelBuffer.pop_front(); 
}

units::meters_per_second_squared_t PoseTracker::getAverageAcceleration() {
    units::meters_per_second_squared_t t = 0_mps_sq;
    for (auto e = accelBuffer.begin(); e != accelBuffer.end(); e++) {
        t += (*e).first;
    }
    return t / accelBuffer.size();
}

units::meters_per_second_t PoseTracker::getAverageVelocity() {
    units::meters_per_second_t t = 0_mps;
    for (auto e = velBuffer.begin(); e != velBuffer.end(); e++) {
        t += (*e).first;
    }
    return t / velBuffer.size();
}