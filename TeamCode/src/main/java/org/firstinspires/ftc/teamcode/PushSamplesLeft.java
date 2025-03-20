package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name="Just Push Left Samples", group = "Robot Autos")
public class PushSamplesLeft extends LinearOpMode {
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
        subsystems.foldLowerClawToPosition(.5);

        waitForStart();

        chassis.moveToDistancePID(7);
        chassis.spinToAngle(55);

        chassis.moveToDistancePID(80);

        chassis.spinToAngle(0);
        chassis.moveToDistancePID(45);

        chassis.spinToAngle(-180);
        chassis.moveToDistancePID(70);

        chassis.moveToDistancePID(-35);
        chassis.spinToAngle(90);
        chassis.moveToDistancePID(25);

        chassis.spinToAngle(0);
        chassis.moveToDistancePID(62);

        chassis.spinToAngle(-180);
        chassis.moveToDistancePID(91);
    }
}
