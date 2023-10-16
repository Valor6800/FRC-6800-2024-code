#pragma once

#include "valkyrie/sensors/BaseSensor.h"
#include <frc/TimedRobot.h>
#include <functional>

namespace valor {

/**
 * @brief Sensor - detect anomalies in a system compared to a pre-set benchmark
 * 
 * This sensor is centered around detecting anomalies in a system. 
 * A target mean and range will be set in the class. Every 20ms, the sensor
 * will poll the getter function and save the results into an array. The average
 * and standard deviation will calculated from the array, and if the calculated mean
 * is outside of the range (target_mean-target_range, target_mean+target_range), then
 * the sensor will report an anomaly.
 * 
 * This should be used to validate in pit and sys checks that everything is fully working
 * as intended without mechanical failures.
*/
class MonitorSensor : public BaseSensor<double>
{
public:

    /**
     * @brief Construct a new Valor Monitor Sensor object
     * 
     * @param _robot Pass in the Robot reference so the calculate method can be auto-scheduled
     * @param _name The name of the specific current sensor for logging and reporting
    */
    MonitorSensor(frc::TimedRobot *_robot, const char *_name);

    void reset() override;

    void InitSendable(wpi::SendableBuilder& builder) override;

    /**
     * @brief Set the target mean for the calculated average value of the sensor during recording
     * 
     * @param _target_mean Target mean of the sensor
    */
    void setTargetMean(double _target_mean);

    /**
     * @brief Set the target range that the calculated average value should be in
     * 
     * @param _target_mean Target mean of the sensor
    */
    void setTargetRange(double _target_range);

    /**
     * @brief Identify if the calculated mean is within the specified bounds
     * 
     * The calculated mean should be between the lower bound of target_mean - target_range and the
     * upper bound of the target_mean + target_range
     * 
     * @return bool If the calculated mean is within the specified range
    */
    bool getInRange();

    /**
     * @brief Start the automatic calculations
    */
    void startRecording();
    
    /**
     * @brief Stop the automatic calculations
    */
    void stopRecording();

private:
    void calculate() override;
    
    /**
     * @brief Helper function to calculate the mean from the history array
     * 
     * @return double The mean of the history array
    */
    double mean();

    /**
     * @brief Holds the target mean that the calculations should be at
    */
    double target_mean;

    /**
     * @brief Holds the target range that the calculations should be betweeen
    */
    double target_range;

    /**
     * @brief If the calculations should be calculated at this time
    */
    bool recording;

    /**
     * @brief The history of the sensor
    */
    std::vector<double> history;
};
}
