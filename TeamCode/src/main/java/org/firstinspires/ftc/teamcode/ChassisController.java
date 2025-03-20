package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ChassisController {
    DcMotor topLeft;
    DcMotor topRight;
    DcMotor rearLeft;
    DcMotor rearRight;
    IMU imu;
    TelemetryPacket packet = new TelemetryPacket();

    public ChassisController(DcMotor[] motors, IMU imu) {
        topLeft = motors[0];
        topRight = motors[1];
        rearLeft = motors[2];
        rearRight = motors[3];

        this.imu = imu;

        setUpMotors();
        setUpIMU();
        imu.resetYaw();
    }
    // Basic movement methods ----------------------------------------------------------------
    public void moveForward(double s){
        topLeft.setPower(s);
        topRight.setPower(s);
        rearLeft.setPower(s);
        rearRight.setPower(s);
    }
    public void moveBackward(double s){
        topLeft.setPower(-s);
        topRight.setPower(-s);
        rearLeft.setPower(-s);
        rearRight.setPower(-s);
    }
    public void moveRight(double s){
        topLeft.setPower(s);
        topRight.setPower(-s);
        rearLeft.setPower(-s);
        rearRight.setPower(s);
    }
    public void moveLeft(double s){
        topLeft.setPower(-s);
        topRight.setPower(s);
        rearLeft.setPower(s);
        rearRight.setPower(-s);
    }
    public void spin(double s){
        topLeft.setPower(s);
        topRight.setPower(-s);
        rearLeft.setPower(s);
        rearRight.setPower(-s);
    }

    public void slowSpin(double s){
        s *= .3;
        topLeft.setPower(-s);
        topRight.setPower(s);
        rearLeft.setPower(-s);
        rearRight.setPower(s);
    }
    public void spinRight(double s){
        topLeft.setPower(s);
        topRight.setPower(-s);
        rearLeft.setPower(s);
        rearRight.setPower(-s);
    }
    public void spinLeft(double s){
        topLeft.setPower(-s);
        topRight.setPower(s);
        rearLeft.setPower(-s);
        rearRight.setPower(s);
    }
    public void moveForwardSlow(double s){
        s *= .2;
        topLeft.setPower(s);
        topRight.setPower(s);
        rearLeft.setPower(s);
        rearRight.setPower(s);
    }

    public void moveBackwardSlow(double s){
        s *= .2;
        topLeft.setPower(-s);
        topRight.setPower(-s);
        rearLeft.setPower(-s);
        rearRight.setPower(-s);
    }

    public void diagonalMovementFrontRight(double s) {
        topLeft.setPower(s);
        topRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(s);
    }

    public void diagonalMovementFrontLeft(double s) {
        topLeft.setPower(0);
        topRight.setPower(s);
        rearLeft.setPower(s);
        rearRight.setPower(0);
    }

    public void diagonalMovementBackRight(double s) {
        topLeft.setPower(0);
        topRight.setPower(-s);
        rearLeft.setPower(-s);
        rearRight.setPower(0);
    }
    public void diagonalMovementBackLeft(double s) {
        topLeft.setPower(-s);
        topRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(-s);
    }

    // Autonomous-meant methods ----------------------------------------------------------------
    public void moveToDistancePID(double distance) { //Distance in cm
        double distanceInRevs = distance / Constants.DistanceConstants.OMNI_CIRC_CM; // TODO: Change this to mecanum wheels
        resetEncoders();

        double currentRevs;
        currentRevs = topLeft.getCurrentPosition() / Constants.EncoderConstants.YELLOW_JKT_TPR;

        while ((distanceInRevs - currentRevs) > .05 || (distanceInRevs-currentRevs) < -.05) { // Error never reaches 0, so a threshold is applied to ensure the loop stops
            double powerPID = PID.calculatePID(distanceInRevs, currentRevs);
            topLeft.setPower(powerPID);
            topRight.setPower(powerPID);
            rearLeft.setPower(powerPID);
            rearRight.setPower(powerPID);
            currentRevs = topLeft.getCurrentPosition() / Constants.EncoderConstants.YELLOW_JKT_TPR;
        }
    }
    public void spinToAngle(int setpointAngle){ // Angle from 0 to 360
        double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = setpointAngle - currentAngle;

        //if(currentAngle < 0) {currentAngle += 360;} // Alter angle to always exist within the 0 to 360 range

        while(error > 5|| error < -5){ // Error never reaches 0, so a threshold is applied to ensure the loop stops
            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = setpointAngle - currentAngle;

            spinLeft(PID.calculateCorrectionPID(setpointAngle,currentAngle));
        }
        spinLeft(0);
    }
    public void spinToAngleByTime(double v, double time){
        ElapsedTime timer = new ElapsedTime();

        while(timer.seconds() < time){
            spin(v);
        }
    }
    // Misc methods ----------------------------------------------------------------
    public void brake(){
        topLeft.setPower(0);
        topRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }
    private void setUpMotors(){
        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        topRight.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    private void setUpIMU(){
        IMU.Parameters imuParameters = new IMU.Parameters(
              new RevHubOrientationOnRobot(
                      RevHubOrientationOnRobot.LogoFacingDirection.UP,
                      RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
              )
        );

        imu.initialize(imuParameters);
        imu.resetYaw();
    }

    public void resetEncoders(){
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
