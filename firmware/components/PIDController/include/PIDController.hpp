#ifndef PIDCONTROLLER_HPP
#define PIDCONTROLLER_HPP

#include "esp_timer.h"

class PIDController{
  public:
      PIDController(double KP,double KI, double KD);
     void control();
     void setValues(double setpoint,double Actual);
     double output;

   private:
      double kp;
      double ki;
      double kd;
      double actual;
      double setPoint;
      double error;
      double previous;
      double integral;
      double last_time;
};

#endif // PIDCONTROLLER_HPP
