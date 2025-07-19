#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

inline ez::Piston mogo_clamp('A');
inline pros::Gps gps1(8, 0, 0.1397, 0);
inline pros::Distance distance_sensor(3);