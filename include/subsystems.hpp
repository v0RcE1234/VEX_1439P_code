#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

inline ez::Piston mogo_clamp('B');
inline pros::Gps gps1(8, 0, 0.1397, 0);
inline pros::Distance distance_sensor(3);
inline pros::Motor intake_left(-2);
inline pros::Motor intake_right(12);
inline pros::MotorGroup intake({-2, 12});
inline pros::Motor conveyor_top(4);
inline pros::Motor conveyor_bottom(7);
inline pros::MotorGroup conveyor({4, 7});
inline ez::Piston mid_goal_piston('A');