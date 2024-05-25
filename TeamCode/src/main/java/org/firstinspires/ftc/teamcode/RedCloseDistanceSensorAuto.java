package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Autonomous(name = "RedCloseDistanceSensorAuto")
public class RedCloseDistanceSensorAuto extends LinearOpMode {

    private DcMotor FrontRight;
    private DcMotor BackRight;
    private DcMotor FrontLeft;
    private DcMotor BackLeft;
    private DcMotor leftarm;
    private DcMotor rightarm;
    private DcMotor LeftPivot;
    private DcMotor RightPivot;
    private DcMotor Lift;
    //servos
    private Servo plane;
    private Servo LeftClaw;
    private Servo RightClaw;
    private Servo Wrist;
    //sensors
    private DistanceSensor RedDistance;
    private DistanceSensor BlueDistance;

    int FrontRightPos;
    int LeftArmPos;
    int BackRightPos;
    int RightArmPos;
    int FrontLeftPos;
    int BackLeftPos;

    /**
     * Describe this function...
     */
    private void drive(double FrontRightTarget, double BackRightTarget, double FrontLeftTarget, double BackLeftTarget, double Speed) {
        FrontRightPos += FrontRightTarget;
        BackRightPos += BackRightTarget;
        FrontLeftPos += FrontLeftTarget;
        BackLeftPos += BackLeftTarget;
        FrontRight.setTargetPosition(FrontRightPos);
        BackRight.setTargetPosition(BackRightPos);
        FrontLeft.setTargetPosition(FrontLeftPos);
        BackLeft.setTargetPosition(BackLeftPos);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setPower(Speed);
        BackRight.setPower(Speed);
        FrontLeft.setPower(Speed);
        BackLeft.setPower(Speed);
        while (opModeIsActive() && FrontRight.isBusy() && BackRight.isBusy() && FrontLeft.isBusy() && BackLeft.isBusy()) {
            // Do nothing
        }
        FrontRight.setPower(0);
        BackRight.setPower(0);
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int hex_motor_ticks;

        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        LeftPivot = hardwareMap.get(DcMotor.class, "LeftPivot");
        RightPivot = hardwareMap.get(DcMotor.class, "RightPivot");
        Lift = hardwareMap.get(DcMotor.class, "Lift");
        plane = hardwareMap.get(Servo.class, "plane");
        LeftClaw = hardwareMap.get(Servo.class, "LeftClaw");
        RightClaw = hardwareMap.get(Servo.class, "RightClaw");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        RedDistance = hardwareMap.get(DistanceSensor.class, "RedDistance");
        BlueDistance = hardwareMap.get(DistanceSensor.class, "BlueDistance");


        // Put initialization blocks here.
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        LeftPivot.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LeftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RightPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LeftPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RightPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            double distance = RedDistance.getDistance(DistanceUnit.CM);
            sleep(500);
            hex_motor_ticks = 288;
            FrontRightPos = 0;
            BackRightPos = 0;
            FrontLeftPos = 0;
            BackLeftPos = 0;
            LeftArmPos = 0;
            RightArmPos = 0;
            sleep(500);
            //arm(hex_motor_ticks * 0.1, hex_motor_ticks * 0.1, 0.5);
            drive(hex_motor_ticks * 6.5, -hex_motor_ticks * 6.5, -hex_motor_ticks * 6.5, hex_motor_ticks * 6.5, 0.5);
            sleep(500);
            if (distance > 43) {
                telemetry.addData("Distance", RedDistance.getDistance(DistanceUnit.CM));
                telemetry.update();
                drive(-hex_motor_ticks * 1, -hex_motor_ticks * 1, -hex_motor_ticks * 1, -hex_motor_ticks * 1, 0.5);
                drive(hex_motor_ticks * 5, hex_motor_ticks * 5, hex_motor_ticks * 5, hex_motor_ticks * 5, 0.5);
                drive(-hex_motor_ticks * 4.25, -hex_motor_ticks * 4.25, -hex_motor_ticks * 4.25, -hex_motor_ticks * 4.25, 0.5);
                drive(-hex_motor_ticks * 1.5, -hex_motor_ticks * 1.5, -hex_motor_ticks * 1.5, -hex_motor_ticks * 1.5, 0.5);
                arm(hex_motor_ticks * 1, hex_motor_ticks * 1, 0.5);
                sleep(100);
                arm(hex_motor_ticks * 2.25, hex_motor_ticks * 2.25, 0.55);
                sleep(500);
                sleep(50);
                FrontLeft.setPower(0);
                BackLeft.setPower(0);
                FrontRight.setPower(0);
                BackRight.setPower(0);
                LeftPivot.setPower(0);
                RightPivot.setPower(0);
                sleep(30000);
            } else {
                drive(hex_motor_ticks * 1, -hex_motor_ticks * 1, -hex_motor_ticks * 1, hex_motor_ticks * 1, 0.5);
                drive(-hex_motor_ticks * 2, -hex_motor_ticks * 2, -hex_motor_ticks * 2, -hex_motor_ticks * 2, 0.5);
            }
            if (distance < 39) {
                telemetry.addData("Distance", RedDistance.getDistance(DistanceUnit.CM));
                telemetry.update();
                drive(-hex_motor_ticks * 1, -hex_motor_ticks * 1, -hex_motor_ticks * 1, -hex_motor_ticks * 1, 0.5);
                drive(hex_motor_ticks * 5, hex_motor_ticks * 5, hex_motor_ticks * 5, hex_motor_ticks * 5, 0.5);
                drive(-hex_motor_ticks * 3, -hex_motor_ticks * 3, -hex_motor_ticks * 3, -hex_motor_ticks * 3, 0.5);
                drive(-hex_motor_ticks * 1.5, -hex_motor_ticks * 1.5, -hex_motor_ticks * 1.5, -hex_motor_ticks * 1.5, 0.5);
                arm(hex_motor_ticks * 1, hex_motor_ticks * 1, 0.5);
                sleep(100);
                arm(hex_motor_ticks * 2.25, hex_motor_ticks * 2.25, 0.55);
                sleep(500);
                sleep(50);
                FrontLeft.setPower(0);
                BackLeft.setPower(0);
                FrontRight.setPower(0);
                BackRight.setPower(0);
                LeftPivot.setPower(0);
                RightPivot.setPower(0);
                sleep(30000);
            } else {
                telemetry.addData("Distance", RedDistance.getDistance(DistanceUnit.CM));
                telemetry.update();
                drive(-hex_motor_ticks * 1, -hex_motor_ticks * 1, -hex_motor_ticks * 1, -hex_motor_ticks * 1, 0.5);
                drive(hex_motor_ticks * 5, hex_motor_ticks * 5, hex_motor_ticks * 5, hex_motor_ticks * 5, 0.5);
                drive(-hex_motor_ticks * 5, -hex_motor_ticks * 5, -hex_motor_ticks * 5, -hex_motor_ticks * 5, 0.5);
                drive(-hex_motor_ticks * 1.5, -hex_motor_ticks * 1.5, -hex_motor_ticks * 1.5, -hex_motor_ticks * 1.5, 0.5);
                arm(hex_motor_ticks * 1, hex_motor_ticks * 1, 0.5);
                sleep(100);
                arm(hex_motor_ticks * 2.25, hex_motor_ticks * 2.25, 0.55);
                sleep(500);
                sleep(50);
                FrontLeft.setPower(0);
                BackLeft.setPower(0);
                FrontRight.setPower(0);
                BackRight.setPower(0);
                LeftPivot.setPower(0);
                RightPivot.setPower(0);
                sleep(30000);
            }
            FrontLeft.setPower(0);
            BackLeft.setPower(0);
            FrontRight.setPower(0);
            BackRight.setPower(0);
            LeftPivot.setPower(0);
            RightPivot.setPower(0);
            sleep(30000);
            telemetry.update();
        }

        }
    //arm function
    private void arm(double LeftArmTarget, double RightArmTarget, double Speed){
        LeftArmPos += LeftArmTarget;
        RightArmPos += RightArmTarget;
        LeftPivot.setTargetPosition(LeftArmPos);
        RightPivot.setTargetPosition(RightArmPos);
        LeftPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftPivot.setPower(Speed);
        RightPivot.setPower(Speed);
        while (opModeIsActive() && LeftPivot.isBusy() && RightPivot.isBusy()) {
            // Do nothing
        }
        LeftPivot.setPower(0);
        RightPivot.setPower(0);
    }
}
