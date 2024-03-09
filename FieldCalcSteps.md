# Pre-Comp
* Check what units each tunable parameter is
    * Shooter distance is meters away from center of robot
    * Shooter Angle is degrees
    * Odom is in meters
    * Robot rotation is in degrees
# Pre-Field Calibration
* Tools:
    * Measuring tapes (2x)
    * Driver Station with advantage scope pulled up
    * Spooled Ethernet Cable
# Odom Calibration
* Make sure robot is tracking AprilTags and poses are being set
* Graph robots position purely from drives, separate limelight positions, and calculated position
    * Move robot x distance away and check if readings are measuring x +- .05 m
    * Check if calculated direction is the same as actual direction
# Pivot & LOBF Calibration
* Measure distances for Line of Best Fit with corresponding angles
* Move robot to the actual distance based of the table below
    * Use measuring tapes
* Input measured value into the Line of Best Fit calculations

| Actual Distance (feet) | Angle (deg) |
|-|-|
|8|39|
|10|33|
|12|30|
|14|27.75|
|15|26.5|
|17|25.25|
|Subwoofer|55|
