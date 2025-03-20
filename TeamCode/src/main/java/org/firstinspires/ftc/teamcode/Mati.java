 package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PID.derivative;
import static org.firstinspires.ftc.teamcode.PID.error;
import static org.firstinspires.ftc.teamcode.PID.integral;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

 @TeleOp(name="Mati", group="Robot")

public class Mati extends OpMode {
    // Chassis
    DcMotor topLeft;
    DcMotor topRight;
    DcMotor rearLeft;
     DcMotor rearRight;
    IMU imu;

    // Subsystems
    DcMotor elevatorMotor;
    DcMotor elbowMotor;
    DcMotor clawMotor;
    DcMotor chainMotor;
    Servo lowerClawDeployServoL;
    Servo lowerClawDeployServoR;
    Servo lowerClawWristServo;
    Servo lowerClawServo;
    TouchSensor touchSensor;
    ColorSensor colorSensor;

    // Chassis and Subsystem controllers
    ChassisController chassis;
    SubsystemController subsystems;

    // Controller Input

    // Gamepad 1
    double RT1;
    double LT1;
    double LSx1;
    double LSy1;
    double RSx1;
    double RSy1;
    boolean LB1;
    boolean RB1;
    boolean Y1;
    boolean B1;
    boolean X1;
    boolean A1;
    boolean dPadUp1;
     boolean dPadDown1;

    // Gamepad 2
    float RT2;
    float LT2;
    double LSy2;
    double LSx2;
    double RSy2;
    boolean LB2;
    boolean RB2;
    double RSx2;
    boolean dPadUp2;
    boolean dPadDown2;
    boolean dPadRight2;
    boolean dPadLeft2;
    boolean A2;
    boolean B2;
    boolean X2;
    boolean Y2;



    @Override
    public void init() {
        // Chassis
        topLeft = hardwareMap.get(DcMotor.class, "topLeft");
        topRight = hardwareMap.get(DcMotor.class, "topRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        DcMotor[] chassisMotors = {topLeft, topRight, rearLeft, rearRight};

        imu = hardwareMap.get(IMU.class,"imu");

        // Subsystem

        elevatorMotor = hardwareMap.get(DcMotor.class,"elevatorMotor");
        elbowMotor = hardwareMap.get(DcMotor.class,"elbowMotor");
        clawMotor = hardwareMap.get(DcMotor.class,"clawMotor");
        chainMotor = hardwareMap.get(DcMotor.class, "chainMotor");

        lowerClawDeployServoL = hardwareMap.get(Servo.class,"lowerClawDeployServoL");
        lowerClawDeployServoR = hardwareMap.get(Servo.class,"lowerClawDeployServoR");
        lowerClawServo = hardwareMap.get(Servo.class,"lowerClawServo");
        lowerClawWristServo = hardwareMap.get(Servo.class,"lowerClawWristServo");

        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");


        DcMotor[] subsystemMotors = {elevatorMotor, elbowMotor, clawMotor, chainMotor};
        Servo[] subsystemServos = {lowerClawDeployServoL, lowerClawDeployServoR, lowerClawWristServo, lowerClawServo};

        chassis = new ChassisController(chassisMotors, imu);
        subsystems = new SubsystemController(subsystemMotors, subsystemServos, touchSensor);

        chassis.resetEncoders();
        subsystems.resetSubsystemEncoders();
    }

    @Override

    public void loop() {
        updateControllerInput();
        updateTelemetry();

       if (LSy1 > 0) {
           chassis.moveForward(LSy1 * -0.6);
        } else if (LSy1 < 0){
            chassis.moveBackward(LSy1 * 0.6);
        } else if (LSx1 > 0) {
            chassis.moveRight(LSx1 * 0.6);
        } else if (LSx1 < 0) {
            chassis.moveLeft(LSx1 * -0.6);


//        if (RT1 > 0) {
//            chassis.moveForward(RT1);
//        }  else if (LT1 > 0) {
//            chassis.moveBackward(-LT1);
//        } else if(RB1) {
//            chassis.moveRight(.8);
//        } else if(LB1) {
//            chassis.moveLeft(.8);

        }else if (RSx1 != 0) {
            chassis.spin(RSx1);
        } else {
            chassis.brake();
        }

        if(dPadUp2){
            subsystems.unfoldClaw();
        } else if(dPadDown2){
            subsystems.foldClaw();
        } else if(dPadRight2){
            subsystems.rotateClawWristRight();
        } else if(dPadLeft2){
            subsystems.rotateClawWristLeft();
        }

        if(RT2 > 0){
            subsystems.openLowerClaw();
        } else if(LT2 > 0){
            subsystems.closeLowerClaw();
        }

        if (dPadUp1) {
            subsystems.extrudeChain();
        } else if (dPadDown1) {
            subsystems.retractChain();
        }

        if(Y2) {
            subsystems.rotateElbow(1);
        } else if(A2) {
            subsystems.rotateElbow(-1);
        } else if(X2){
            subsystems.rotateElbowToPosition(Constants.ActuatorConstants.ELBOW_MID_TICKS); // Optimized position for specimen pickup
        }else {
            subsystems.rotateElbow(0);
        }

        subsystems.extendArm(RSy2);
        subsystems.triggerClaw(RB2 ? .6 : (LB2 ? -.6 : 0));
    }

    public void updateControllerInput(){
        RT1 = gamepad1.right_trigger;
        LT1 = gamepad1.left_trigger;
        LSx1 = gamepad1.left_stick_x;
        LSy1 = gamepad1.left_stick_y;
        RSx1 = gamepad1.right_stick_x;
        RSy1 = gamepad1.right_stick_y;
        LB1 = gamepad1.left_bumper;
        RB1 = gamepad1.right_bumper;
        A1 = gamepad1.a;
        B1 = gamepad1.b;
        X1 = gamepad1.x;
        Y1 = gamepad1.y;
        dPadUp1 = gamepad1.dpad_up;
        dPadDown1 = gamepad1.dpad_down;

        RT2 = gamepad2.right_trigger;
        LT2 = gamepad2.left_trigger;
        LSx2 = gamepad2.left_stick_x;
        LSy2 = -gamepad2.left_stick_y;
        RSy2 = -gamepad2.right_stick_y;
        LB2 = gamepad2.left_bumper;
        RB2 = gamepad2.right_bumper;
        RSx2 = gamepad2.right_stick_x;
        dPadUp2 = gamepad2.dpad_up;
        dPadDown2 = gamepad2.dpad_down;
        dPadLeft2 = gamepad2.dpad_left;
        dPadRight2 = gamepad2.dpad_right;
        A2 = gamepad2.a;
        B2 = gamepad2.b;
        X2 = gamepad2.x;
        Y2 = gamepad2.y;
    }
    public void updateTelemetry(){
        telemetry.addData("P", PID.PIDConstants.P * error);
        telemetry.addData("I", PID.PIDConstants.I * integral);
        telemetry.addData("D", PID.PIDConstants.D * derivative);
        telemetry.addData("Output", (PID.PIDConstants.P * error) + (PID.PIDConstants.I * integral) + (PID.PIDConstants.D * derivative));
        telemetry.addData("Angular velocity", imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);

        telemetry.addData("Current Angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        telemetry.addData("ColorRed", colorSensor.red());
        telemetry.addData("ColorBlue", colorSensor.blue());
        telemetry.addData("ColorGreen", colorSensor.green());

        int colorW = colorSensor.blue() + colorSensor.red() + colorSensor.green();
        telemetry.addData("Color", colorW);


        telemetry.addLine();

        telemetry.update();
    }
}