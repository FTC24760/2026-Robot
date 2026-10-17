package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    double Kp, Ki, Kd, Kf;
    double previousError = 0;
    double integral = 0;
    ElapsedTime timer;
    double previousTime;
    public PID(double Kp, double Ki, double Kd, double Kf) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
        timer = new ElapsedTime();
        previousTime = timer.time();
    }
    double update(double target, double state) {
        double error = target - state;
        integral += error;
        double derivative = error / (timer.time() - previousTime);

        previousTime = timer.time();
        previousError = error;


        double P = Kp * error;
        double I = Ki * integral;
        double D = Kd * derivative;
        double F = Kf * target;

        return P + I + D + F;
    }
}
