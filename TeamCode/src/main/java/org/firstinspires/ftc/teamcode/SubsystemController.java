package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class  SubsystemController {
    DcMotor elevatorMotor;
    DcMotor elbowMotor;
    DcMotor clawMotor;
    DcMotor chainMotor;
    Servo chainClawDeployServoL;
    Servo chainClawWristServo;
    Servo chainClawDeployServoR;
    Servo chainClawServo;
    TouchSensor touchSensor;
    Telemetry telemetry;
    double currentRevs = 0;
    double lastStickReading = 0;

    public SubsystemController(DcMotor[] motors, Servo[] servos, TouchSensor touchSensor) {
        elevatorMotor = motors[0];
        elbowMotor = motors[1];
        clawMotor = motors[2];
        chainMotor = motors[3];

        this.touchSensor = touchSensor;

        chainClawDeployServoL = servos[0];
        chainClawDeployServoR = servos[1];
        chainClawWristServo = servos[2];
        chainClawServo = servos[3];

        setupMotorsAndServos();
    }
    public void rotateClawWristRight(){
        chainClawWristServo.setPosition(chainClawWristServo.getPosition() + .02);
    }
    public void rotateClawWristLeft(){
        chainClawWristServo.setPosition(chainClawWristServo.getPosition() - .02);
    }
    public void unfoldClaw(){
        chainClawDeployServoL.setPosition(chainClawDeployServoL.getPosition() + .02);
        chainClawDeployServoR.setPosition(chainClawDeployServoL.getPosition() + .02);
    }
    public void foldClaw(){
        chainClawDeployServoL.setPosition(chainClawDeployServoL.getPosition() - .02);
        chainClawDeployServoR.setPosition(chainClawDeployServoL.getPosition() - .02);
    }
    public void foldLowerClawToPosition(double foldPos){
        chainClawDeployServoL.setPosition(foldPos);
        chainClawDeployServoR.setPosition(foldPos);
    }
    public void closeLowerClaw(){
        chainClawServo.setPosition(chainClawServo.getPosition() + .03);
    }
    public void openLowerClaw(){
        chainClawServo.setPosition(chainClawServo.getPosition() - .03);
    }
    public void moveLowerClawToPosition(double clawPos){
        chainClawServo.setPosition(clawPos);
    }
    public void extrudeChain(){
        chainMotor.setPower(Constants.SpeedConstants.CHAIN_EXTRUDE_SPEED);
    }
    public void retractChain(){
        chainMotor.setPower(Constants.SpeedConstants.CHAIN_RETRACT_SPEED);
    }
    public void moveChain(double v){
        chainMotor.setPower(v);
    }

    public void moveChainByTime(double v, double time){
        ElapsedTime timer = new ElapsedTime();

        while(timer.seconds() < time){
            chainMotor.setPower(v);
        }

        chainMotor.setPower(0);
    }
    public void moveChainToDistance(double distance){
        double distanceInRevs = distance / .55; // TODO: Change to REV Sprocket Circumference
        double revsInTicks = distanceInRevs * Constants.EncoderConstants.CORE_HEX_TPR;
    }
    public void extendArm(double v) {
        elevatorMotor.setPower(v);
    }
    public void retractArm(double v) {
        elevatorMotor.setPower(-v);
    }
    public void rotateElbow(double v){
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowMotor.setPower(v);
    }
    public void rotateElbowToPosition(int elbowPosition){
        elbowMotor.setTargetPosition(elbowPosition);
        elbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elbowMotor.setPower(.8);
    }
    public void triggerClaw(double v){
        clawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawMotor.setPower(v);
    }
    public void closeClaw(){
        clawMotor.setTargetPosition(Constants.EncoderConstants.CLOSED_CLAW_TICKS);
        clawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clawMotor.setPower(1);
    }
    public void openClaw(){
        clawMotor.setTargetPosition(Constants.EncoderConstants.OPEN_CLAW_TICKS);
        clawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clawMotor.setPower(1);
    }
    public void extendArmToDistancePID(double distance){
        double distanceInRevs = distance / Constants.DistanceConstants.PULLEY_CIRC_CM;

        resetSubsystemEncoders();

        double error = distanceInRevs - currentRevs;
        while(error > .6 || error < -.6){ // Error never reaches 0, so a threshold is applied to ensure the loop stops
            elevatorMotor.setPower(PID.calculatePID(distanceInRevs, currentRevs));
            currentRevs = elevatorMotor.getCurrentPosition() / Constants.EncoderConstants.TRQNADO_TPR;
            error = distanceInRevs - currentRevs;
        }
    }

    public void extendArmToPosition(int setPointTicks){
        elevatorMotor.setTargetPosition(setPointTicks);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorMotor.setPower(-.8);
    }
    public void extendArmByTime(double v, double time){
        ElapsedTime timer = new ElapsedTime();

        timer.reset();

        while(timer.seconds() < time) {
            elevatorMotor.setPower(v);
        }

        elevatorMotor.setPower(0);
    }
    // Misc ----------------------------------------------------------------
    public void brake() {
        elevatorMotor.setPower(0);
        elbowMotor.setPower(0);
    }
    public void resetSubsystemEncoders() {
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void homeServos(){

    }
    private void setupMotorsAndServos(){
        elevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        elbowMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        clawMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        chainClawDeployServoL.setDirection(Servo.Direction.REVERSE);
    }
}
