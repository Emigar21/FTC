package org.firstinspires.ftc.teamcode;
import  com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Specimen + push samples", group="Robot Autos")

public class SpecimenAndPush extends LinearOpMode {
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
            // Chassis

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

            waitForStart(); // --------- Auto Start Right Side --------

            chassis.moveToDistancePID(7);
            chassis.spinToAngle(45);

            chassis.moveToDistancePID(85);

            chassis.spinToAngle(0);
            chassis.brake();

            chassis.moveToDistancePID(-8);
            chassis.brake();

            // Leave Specimen

            subsystems.rotateElbowToPosition(Constants.ActuatorConstants.ELBOW_HIGH_TICKS);

            subsystems.extendArmByTime(1, .9);

            chassis.moveToDistancePID(12);

            subsystems.triggerClaw(.1);
            subsystems.extendArmByTime(-.3, .3);
            subsystems.extendArmByTime(-1, .6);

            subsystems.openClaw();

            subsystems.brake();

            // Move Toward Samples

            chassis.moveToDistancePID(-10); // Back away from submersible
            chassis.spinToAngle(-90);

            chassis.moveToDistancePID(92); // Move to samples
 }
}
