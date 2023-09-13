// MIT License

// Copyright (c) 2022 Mateus Menezes

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef OMNIDIRECTIONAL_CONTROLLERS__TYPES_HPP_
#define OMNIDIRECTIONAL_CONTROLLERS__TYPES_HPP_

#include <cmath>

#define DEG2RAD(deg) (deg * M_PI / 180.0)

namespace omnidirectional_controllers {

struct RobotParams {
  double wheel_radius;
  double robot_radius;
  double gamma;
};

struct RobotVelocity {
  double vx;    // [m/s]
  double vy;    // [m/s]
  double omega;    // [rad]
};

struct RobotPose {
  double x;        //  [m]
  double y;        //  [m]
  double theta;    // [rad]
};

// Proportional controller gains for 2D
struct PGains{
  double x; //
  double y; // 
  double w; //
};
// clamp rotation between (-PI, PI)
template<typename T>
static T clampRotation(T rotation)
{
    while (rotation > static_cast<T>(M_PI))
        rotation -= 2*static_cast<T>(M_PI);
    while (rotation < -static_cast<T>(M_PI))
        rotation += 2*static_cast<T>(M_PI);
    return rotation;
}

}  // namespace omnidirectional_controllers

#endif  // OMNIDIRECTIONAL_CONTROLLERS__TYPES_HPP_
