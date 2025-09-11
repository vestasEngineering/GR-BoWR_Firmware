#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
public:
    float Kp, Ki, Kd;
    float setpoint;
    float outputMin, outputMax;
    float sampleTime; // in seconds

    PID(float kp, float ki, float kd, float setpoint = 0.0f)
        : Kp(kp), Ki(ki), Kd(kd), setpoint(setpoint),
          outputMin(-1.0f), outputMax(1.0f),
          sampleTime(0.05f), lastError(0.0f), integral(0.0f) {}

    void setOutputLimits(float min, float max) {
        outputMin = min;
        outputMax = max;
    }

    void setSampleTime(float timeSec) {
        sampleTime = timeSec;
    }

    void setSetpoint(float sp) {
        setpoint = sp;
    }

    float compute(float input) {
        float error = setpoint - input;
        integral += error * sampleTime;
        float derivative = (error - lastError) / sampleTime;
        lastError = error;

        float output = Kp * error + Ki * integral + Kd * derivative;
        if (output > outputMax) output = outputMax;
        else if (output < outputMin) output = outputMin;

        return output;
    }

private:
    float lastError;
    float integral;
};

#endif