package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PickUpAndDeliver extends LinearOpMode {

    DcMotor chassisLeft;
    DcMotor chassisRight;
    IMU imu;
    DcMotor armAxisMotorL;
    DcMotor armAxisMotorR;
    DcMotor elevatorMotor;
    DcMotor elbowMotor;
    DcMotor clawMotor;
    DcMotor scissorMotor;
    Servo scissorClawServo;
    Servo scissorClawWristServoL;
    Servo scissorClawWristServoR;
    TouchSensor touchSensor;
    ChassisController chassis;
    SubsystemController subsystems;

    @Override
    public void runOpMode() throws InterruptedException {
        chassisLeft = hardwareMap.get(DcMotor.class, "topLeft");
        chassisRight = hardwareMap.get(DcMotor.class, "topRight");

        DcMotor[] chassisMotors = {chassisLeft, chassisRight};

        imu = hardwareMap.get(IMU.class,"imu");

        // Subsystem

        armAxisMotorL = hardwareMap.get(DcMotor.class,"armAxisMotorL");
        armAxisMotorR = hardwareMap.get(DcMotor.class, "armAxisMotorR");
        elevatorMotor = hardwareMap.get(DcMotor.class,"elevatorMotor");
        elbowMotor = hardwareMap.get(DcMotor.class,"elbowMotor");
        clawMotor = hardwareMap.get(DcMotor.class,"clawMotor");
        scissorMotor = hardwareMap.get(DcMotor.class,"chainMotor");

        scissorClawServo = hardwareMap.get(Servo.class,"chainClawServo");
        scissorClawWristServoL = hardwareMap.get(Servo.class,"chainClawDeployServoL");
        scissorClawWristServoR = hardwareMap.get(Servo.class,"chainClawDeployServoL");

        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        DcMotor[] subsystemMotors = {armAxisMotorL, armAxisMotorR, elevatorMotor, elbowMotor, clawMotor, scissorMotor};
        Servo[] subsystemServos = {scissorClawWristServoL, scissorClawWristServoR, scissorClawServo};

        chassis = new ChassisController(chassisMotors, imu);
        subsystems = new SubsystemController(subsystemMotors, subsystemServos, touchSensor);

        chassis.resetEncoders();
        subsystems.resetSubsystemEncoders();

        subsystems.foldLowerClawToPosition(Constants.ActuatorConstants.SCISSOR_CLAW_FOLDED);

        waitForStart();

        chassis.moveToDistancePID(7);
        chassis.spinToAngle(-55);

        // Move Toward first sample
        chassis.moveToDistancePID(80);
        chassis.spinToAngle(0);

        subsystems.moveChainByTime(.3, .2);
        subsystems.moveLowerClawToPosition(Constants.ActuatorConstants.SCISSOR_CLAW_OPEN);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while(timer.seconds() <  1.4){
            subsystems.foldLowerClawToPosition(Constants.ActuatorConstants.SCISSOR_CLAW_UNFOLDED);
        }

        timer.reset();

        while(timer.seconds() <  2){
            subsystems.moveLowerClawToPosition(Constants.ActuatorConstants.SCISSOR_CLAW_CLOSED);
        }

        timer.reset();

        while(timer.seconds() <  1.4){
            subsystems.foldLowerClawToPosition(Constants.ActuatorConstants.SCISSOR_CLAW_FOLDED);
        }

        chassis.spinToAngle(-180);

        timer.reset();

        while(timer.seconds() <  1.4){
            subsystems.foldLowerClawToPosition(Constants.ActuatorConstants.SCISSOR_CLAW_UNFOLDED);
        }

        subsystems.moveLowerClawToPosition(Constants.ActuatorConstants.SCISSOR_CLAW_OPEN);
        sleep(1000);

        chassis.spinToAngle(-90);
        chassis.moveToDistancePID(10);

    }
}
