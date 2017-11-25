#include "PID.h"
#include <iostream>


PID::PID() {
  this->cte_initialized = false;
  this->i_error = 0;
}
PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::UpdateError(double cte) {
  if (!this->cte_initialized) {
    this->p_error = cte;
    this->cte_initialized = true;
    this->last_timestamp = clock();
  }

  this->i_error += cte;

  clock_t now = clock();
  clock_t delta_time = now - this->last_timestamp;
  this->last_timestamp = now;
  if (delta_time > 0) {
    this->d_error = (cte - this->p_error) / delta_time;
  } else {
    this->d_error = 0;
  }

  this->p_error = cte;
}

double PID::TotalError() {
  std::cout << -Kp * p_error <<  " " << -Kd * d_error << " " << -Ki * i_error << std::endl;
  return(-Kp * p_error - Kd * d_error - Ki * i_error);
}
