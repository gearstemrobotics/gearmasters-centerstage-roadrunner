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

@Autonomous(name = "Driveforwardscorepurplecenter")
public class Driveforwardscorepurplecenter extends LinearOpMode {

    private DcMotor FrontRight;
    private DcMotor BackRight;
    private DcMotor FrontLeft;
    private DcMotor BackLeft;
    private DcMotor LeftPivot;
    private DcMotor RightPivot;
    private DcMotor Lift;
    //servos
    private Servo Plane;
    private Servo LeftClaw;
    private Servo RightClaw;
    private CRServo Wrist;
    //sensors
    private DistanceSensor RightDistance;
    private DistanceSensor LeftDistance;

    public static final double LEFT_CLAW_OPEN = 1;
    public static final double LEFT_CLAW_CLOSE = 0.3;
    public static final double RIGHT_CLAW_OPEN = 0;
    public static final double RIGHT_CLAW_CLOSE = 0.8;

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

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.SS
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
        Plane = hardwareMap.get(Servo.class, "Plane");
        LeftClaw = hardwareMap.get(Servo.class, "LeftClaw");
        RightClaw = hardwareMap.get(Servo.class, "RightClaw");
        Wrist = hardwareMap.get(CRServo.class, "Wrist");
        RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");
        LeftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistance");


        // Put initialization blocks here.
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        LeftPivot.setDirection(DcMotor.Direction.REVERSE);
        LeftClaw.setPosition(LEFT_CLAW_CLOSE);
        RightClaw.setPosition(RIGHT_CLAW_CLOSE);
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
            double rightDistance = RightDistance.getDistance(DistanceUnit.CM);
            double leftDistance = LeftDistance.getDistance(DistanceUnit.CM);
            sleep(500);
            hex_motor_ticks = 288;
            FrontRightPos = 0;
            BackRightPos = 0;
            FrontLeftPos = 0;
            BackLeftPos = 0;
            LeftArmPos = 0;
            RightArmPos = 0;
            sleep(500);
            drive(hex_motor_ticks * 5, hex_motor_ticks * 5, hex_motor_ticks * 5, hex_motor_ticks * 5, 0.5);
            drive(-hex_motor_ticks * 7, hex_motor_ticks * 7, hex_motor_ticks * 7, -hex_motor_ticks * 7, 0.5);
            arm(-hex_motor_ticks * 0.5, -hex_motor_ticks * 0.5, 0.4);
            LeftClaw.setPosition(LEFT_CLAW_OPEN);
            arm(hex_motor_ticks * 1, hex_motor_ticks * 1, 0.7);
            //drive(hex_motor_ticks * 3, hex_motor_ticks * 3, hex_motor_ticks * 3, hex_motor_ticks * 3, 0.5);
            //sleep(500);
            /*if (leftDistance < 20) {
                telemetry.addData("Distance", RightDistance.getDistance(DistanceUnit.CM));
                telemetry.update();
                arm(-hex_motor_ticks * 2.25, -hex_motor_ticks * 2.25, 0.1);
                LeftClaw.setPosition(LEFT_CLAW_OPEN);
                arm(hex_motor_ticks * 2.25, hex_motor_ticks * 2.25, 0.7);
                drive(-hex_motor_ticks * 1, -hex_motor_ticks * 1, -hex_motor_ticks * 1, -hex_motor_ticks * 1, 0.5);
                drive(hex_motor_ticks * 5, hex_motor_ticks * 5, hex_motor_ticks * 5, hex_motor_ticks * 5, 0.5);
                drive(-hex_motor_ticks * 5, -hex_motor_ticks * 5, -hex_motor_ticks * 5, -hex_motor_ticks * 5, 0.5);
                drive(-hex_motor_ticks * 1.5, -hex_motor_ticks * 1.5, -hex_motor_ticks * 1.5, -hex_motor_ticks * 1.5, 0.5);
                arm(hex_motor_ticks * 1, hex_motor_ticks * 1, 0.5);
                sleep(100);
                arm(hex_motor_ticks * 2.25, hex_motor_ticks * 2.25, 0.55);
                sleep(500);
                RightClaw.setPosition(RIGHT_CLAW_OPEN);
                sleep(50);
                FrontLeft.setPower(0);
                BackLeft.setPower(0);
                FrontRight.setPower(0);
                BackRight.setPower(0);
                LeftPivot.setPower(0);
                RightPivot.setPower(0);
                sleep(30000);
            }
            if (rightDistance < 20) {
                telemetry.addData("Distance", LeftDistance.getDistance(DistanceUnit.CM));
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
                drive(-hex_motor_ticks * 1, -hex_motor_ticks * 1, -hex_motor_ticks * 1, -hex_motor_ticks * 1, 0.5);
                arm(-hex_motor_ticks * 2.25, -hex_motor_ticks * 2.25, 0.2);
                //Wrist.setPower(-0.5);
                //sleep(300);
                LeftClaw.setPosition(LEFT_CLAW_OPEN);
                arm(hex_motor_ticks * 2.25, hex_motor_ticks * 2.25, 0.7);
                drive(hex_motor_ticks * 5, hex_motor_ticks * 5, -hex_motor_ticks * 5, -hex_motor_ticks * 5, 0.5);
                drive(hex_motor_ticks * 5, hex_motor_ticks * 5, hex_motor_ticks * 5, hex_motor_ticks * 5, 0.5);
                drive(-hex_motor_ticks * 4.25, hex_motor_ticks * 4.25, hex_motor_ticks * 4.25, -hex_motor_ticks * 4.25, 0.5);
                drive(hex_motor_ticks * 1.5, hex_motor_ticks * 1.5, hex_motor_ticks * 1.5, hex_motor_ticks * 1.5, 0.5);
                arm(-hex_motor_ticks * 0.5, -hex_motor_ticks * 0.5, 0.5);
                sleep(100);
                RightClaw.setPosition(RIGHT_CLAW_OPEN);
                sleep(50);
                FrontLeft.setPower(0);
                BackLeft.setPower(0);
                FrontRight.setPower(0);
                BackRight.setPower(0);
                LeftPivot.setPower(0);
                RightPivot.setPower(0);
                sleep(30000);
            }

             */
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

}