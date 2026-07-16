# Rfc
RocketFlight is an open-source flight computer firmware designed for model rockets. This project focuses exclusively on the software stack required to capture sensor data, determine flight state transitions (launch, apogee, landing), and trigger deployment events (parachutes).

Features
Real-time Sensor Processing: Integrates high-frequency data from IMUs (accelerometer/gyroscope) and barometric pressure sensors.

# State #  Machine Logic: Robust flight state detection (Idle, Boost, Coast, Descent, Recovery).

# Data # Logging: Records flight telemetry to onboard storage for post-flight analysis.

# Event # Triggering: Configurable timing and altitude triggers for dual-deployment parachute systems.

# Architecture

The system operates on a cyclic loop, processing input signals and updating the flight state machine to ensure timely deployment of recovery systems.

# Getting Started 
Prerequisites
Hardware: This code is optimized for Arduino Mega  

 This software is provided for experimental use only. The user assumes all responsibility for the reliability and safety of the flight hardware and recovery systems. Always conduct ground testing before any flight attempt.Contributing Contributions are welcome! Please open an issue to discuss proposed features or submit a pull request for bug fixes

.LicenseDistributed under the MIT License. See LICENSE for more information
