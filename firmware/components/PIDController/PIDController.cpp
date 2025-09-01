#include "PIDController.hpp"


PIDController::PIDController(double KP,double KI, double KD){
    kp = KP;
    ki = KI;
    kd = KD;
    previous = 0.0f;
    integral = 0.0f;
    last_time = esp_timer_get_time() / 1000000.0; // Convert microseconds to seconds
}
void PIDController::control(){
    double current_time = esp_timer_get_time() / 1000000.0; // Convert microseconds to seconds
    double dt = current_time - last_time;
    last_time = current_time;
    
    double proportional = error;
    integral += error * dt;
    double derivative = (error - previous) / dt;
    previous = error;
    output = (kp * proportional) + (ki * integral) + (kd * derivative);
    
}
void PIDController::setValues(double setpoint,double Actual){
    setPoint = setpoint;
    actual = Actual;
    error = setPoint - actual;
}

void PIDController::setTunings(double KP, double KI, double KD) {
    kp = KP;
    ki = KI;
    kd = KD;
}
