# RTK_Project

This personal research project is my attempt at an IMU-based state estimator. The objective is to combine acceleration data, GPS position, and (possibly) anemometer speed or optical flow velocity to get our second-order states.

### Suggested Readings:
* Chapter 7 of S.O.H. Madgwick's PhD Thesis (2014); explains the first attempts at the ECF with block diagram.
* Extended Complementary Filter (2020) by Madgwick et al.; looks to be a slightly improved version of the algorithm.

This project requires the following components:
* Teensy 4.1 by pjrc
* 4 x MPU-9250 modules
* MAX-M10 GNSS module by ublox
* Wind Sensor Rev C by Modern Devices
* Optical Flow LiDAR Sensor by Matek Systems
