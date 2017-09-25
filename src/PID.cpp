#include <iostream>
#include <cmath>
#include "PID.h"

using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

PID::PID() :
  p_error(0), i_error(0), d_error(0),
  maxSteering(25) {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  i_error += cte;
  p_error = cte;
}

double PID::TotalError() {
  return rad2deg(-Kp*p_error - Kd*d_error - Ki*i_error)/maxSteering;
}
