#include <iostream>
#include <boost>

/*  Test Low level methods
Navigation module
- Motor controller and serial driver
    - Connect to Serial (arduino)
    - Set motor speeds @ 10Hz
    - Get encoders (velocity, position) @ 10Hz   
- Kinematics
    - Forward kinematics - calculate motor speeds
    - Inverse kinematics - calculate odometry
    - Publish Odometry with timestamp
    - Subscribe to Motorspeeds

- Path planning (low level)
    - Calculate Potential field in robot frame
    - Subscribe to KF Obstacles
    - Subscribe to KF Targets
    - Publish Command Velocity with timestamp

Kalman Filter Module
- Navigation KF
    - Subscribe Heading in Frame:World
    - Subscribe Odometry in Frame:Vehicle
    - Subscribe Vision Model Odometry in Frame:Vehicle
    - Publish KF_State with timestamp

- Target KF
    - Subscribe Vision Model Target
    - Subscribe Vision Model Obstacles
    - Convert to Frame:Vehicle
    - Manage list of Targets and obstacles
        - Add to list
        - Prune list

- IMU driver and comms
    - Get IMU messages @ 50Hz (I/O thread)
    - Publish Attitude in Frame:Vehicle
    - Publish Heading in Frame:World

- Camera module
    - Get video frames 
    - Publish to Camera (GStreamer src)

- Vision model
    - Subscribe to Camera (Gstreamer sink)
    - Calculate visual odometry estimate
    - Find Targets in frame
    - Find obtacles in frame
    - Publish target list in Frame:Camera
    - Publish obstacle list in Frame:Camera
    - Publish Odometry in Frame:Vehicle
    
EXTERNAL PROCESSES
- SLAM (P3)

-Telemetry viewer 
(P1) - Gstreamer sink / Console output
(P3) - formatted application window
*/ 