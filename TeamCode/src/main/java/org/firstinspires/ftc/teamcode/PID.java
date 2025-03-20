package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    static ElapsedTime timer = new ElapsedTime();
    static double error = 0;
    static double integral = 0;
    static double derivative = 0;
    static double lastError = 0;
    public static double calculatePID(double setpoint, double currentValue){
        error = setpoint - currentValue;
        timer.reset();
        integral += error * timer.milliseconds();
        derivative = (error - lastError) / timer.milliseconds();
        lastError = error;

        return (PIDConstants.P * error) + (PIDConstants.I * integral) + (PIDConstants.D * derivative);
    }
    public static double calculateArmPID(double setpoint, double currentValue) {
        error = setpoint - currentValue;
        timer.reset();
        integral += error * timer.milliseconds();
        derivative = (error - lastError) / timer.milliseconds();
        lastError = error;

        return (PIDConstants.ArmPID.P * error) + (PIDConstants.ArmPID.I * integral) + (PIDConstants.ArmPID.D * derivative);
    }
    public static double calculateCorrectionPID(double setpoint, double currentValue) {
        error = setpoint - currentValue;
        timer.reset();
        integral += error * timer.milliseconds();
        derivative = (error - lastError) / timer.milliseconds();
        lastError = error;

        return (PIDConstants.ChassisCorrectionPID.P * error) + (PIDConstants.ChassisCorrectionPID.I * integral) + (PIDConstants.ChassisCorrectionPID.D * derivative);
    }

    static class PIDConstants {
        public static final double P = 0.48;
        public static final double I = 0.0;
        public static final double D = 0.0;
        static class ArmPID{
            public static final double P = 0.0;
            public static final double I = 0.0;
            public static final double D = 0.0;
        }

        static class ChassisCorrectionPID {
            public static final double P = 0.007;
            public static final double I = 0.0;
            public static final double D = 0.0;
        }
    }
}
