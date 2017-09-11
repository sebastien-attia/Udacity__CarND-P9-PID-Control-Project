#include <iostream>
#include <cmath>
#include "PID.h"

using namespace std;

PID::PID() :
  p_error(0), i_error(0), d_error(0),
  maxSteering(25),
  isFirdtIter(true) {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::UpdateError(double cte) {
  if (isFirdtIter) {
    d_error = 0;
    isFirdtIter = false;
  } else {
    d_error = cte - p_error;
  }
  i_error += cte;
  p_error = cte;
}

double PID::Output() {
  const double output = (-Kp*p_error - Kd*d_error - Ki*i_error)/maxSteering;

  if (output > maxSteering) {
    return 1;
  } else if (output < -maxSteering) {
    return -1;
  }

  return output;
}

double PID::TotalError() {
  return abs(p_error+i_error+d_error);
}
