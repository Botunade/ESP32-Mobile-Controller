#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PIDController {
public:
    // Constructor with optional limits (default 0-100 for percentage)
    PIDController(float kp, float ki, float kd, float minLimit = 0.0f, float maxLimit = 100.0f)
        : kp(kp), ki(ki), kd(kd), minOut(minLimit), maxOut(maxLimit), integral(0), prevError(0) {}

    float compute(float setpoint, float measured, float dtSeconds) {
        float error = setpoint - measured;
        
        // Proportional term
        float P = kp * error;

        // Integral term
        integral += error * dtSeconds;

        // Anti-windup: Clamp integral contribution indirectly via output clamping logic below
        // or prevent integral accumulation if saturated.
        // Here we use a standard clamping on the calculated I-term for stability
        // relative to output limits.

        float I = ki * integral;

        // Derivative term
        float derivative = (error - prevError) / dtSeconds;
        float D = kd * derivative;

        prevError = error;

        // Calculate total output
        float output = P + I + D;

        // Output Clamping and Anti-Windup
        if (output > maxOut) {
            output = maxOut;
            // Prevent integral from growing in the same direction
            if (error > 0) integral -= error * dtSeconds;
        } else if (output < minOut) {
            output = minOut;
            // Prevent integral from growing in the same direction
            if (error < 0) integral -= error * dtSeconds;
        }

        return output;
    }

    void reset() {
        integral = 0;
        prevError = 0;
    }

    void setTunings(float kp, float ki, float kd) {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }

    void setOutputLimits(float min, float max) {
        minOut = min;
        maxOut = max;
    }

private:
    float kp;
    float ki;
    float kd;
    float minOut;
    float maxOut;
    float integral;
    float prevError;
};

#endif // PID_H
