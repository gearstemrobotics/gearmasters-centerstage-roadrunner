package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Autonomous(name = "RedRight")
public class RedRight extends LinearOpMode{

    int LeftArmPos;
    int RightArmPos;

    private DcMotor LeftPivot;
    private DcMotor RightPivot;
    private Servo LeftClaw;
    private Servo RightClaw;
    private DistanceSensor RightDistance;
    private DistanceSensor LeftDistance;

    public static final double LEFT_CLAW_OPEN = 1;
    public static final double LEFT_CLAW_CLOSE = 0;
    public static final double RIGHT_CLAW_OPEN = 0;
    public static final double RIGHT_CLAW_CLOSE = 1;

    private enum PixelPos {left, right, center}
    PixelPos pixelPos = PixelPos.center;

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

    @Override
    public void runOpMode() throws InterruptedException {
        int hex_motor_ticks;


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(60,12,Math.toRadians(180)));

        LeftPivot = hardwareMap.get(DcMotor.class, "LeftPivot");
        RightPivot = hardwareMap.get(DcMotor.class, "RightPivot");
        LeftClaw = hardwareMap.get(Servo.class, "LeftClaw");
        RightClaw = hardwareMap.get(Servo.class, "RightClaw");
        RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");
        LeftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistance");

        LeftPivot.setDirection(DcMotor.Direction.REVERSE);

        LeftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // drive forward at start
        Action TrajA1 = drive.actionBuilder(drive.pose).lineToX(32).build();
        // back up to prep for scoring purple pixel
        Action TrajA2 = drive.actionBuilder(drive.pose).lineToX(36).build();
        // turn left to place purple pixel
        Action TrajA3 = drive.actionBuilder(drive.pose).turnTo(Math.toRadians(225)).build();
        // turn right to place purple pixel
        Action TrajA4 = drive.actionBuilder(drive.pose).turnTo(Math.toRadians(135)).build();
        Action TrajA5 = drive.actionBuilder(drive.pose).strafeToLinearHeading(new Vector2d(60,48),Math.toRadians(70)).build();

        waitForStart();

        hex_motor_ticks = 288;

        LeftArmPos = 0;
        RightArmPos = 0;



        double rightDistance = RightDistance.getDistance(DistanceUnit.CM);
        double leftDistance = LeftDistance.getDistance(DistanceUnit.CM);

        Actions.runBlocking(new SequentialAction(TrajA1));
        //Actions.runBlocking(new SleepAction(0.5));

        if (leftDistance < 20){
            pixelPos = PixelPos.left;
        }
        if (rightDistance < 20){
            pixelPos = PixelPos.right;
        }
        else {
            pixelPos = PixelPos.center;
        }

        Actions.runBlocking(new SequentialAction(TrajA2));

        switch (pixelPos){
            case left:
                telemetry.addLine("left");
                telemetry.update();
                Actions.runBlocking(new SequentialAction(TrajA3));
                arm(-hex_motor_ticks * 2.25, -hex_motor_ticks * 2.25, 0.4);
                LeftClaw.setPosition(LEFT_CLAW_OPEN);
                arm(hex_motor_ticks * 2.25, hex_motor_ticks * 2.25, 0.7);
                break;
            case center:
                telemetry.addLine("center");
                telemetry.update();
                arm(-hex_motor_ticks * 2.25, -hex_motor_ticks * 2.25, 0.4);
                LeftClaw.setPosition(LEFT_CLAW_OPEN);
                arm(hex_motor_ticks * 2.25, hex_motor_ticks * 2.25, 0.7);
                break;
            case right:
                telemetry.addLine("right");
                telemetry.update();
                Actions.runBlocking(new SequentialAction(TrajA4));
                arm(-hex_motor_ticks * 2.25, -hex_motor_ticks * 2.25, 0.4);
                LeftClaw.setPosition(LEFT_CLAW_OPEN);
                arm(hex_motor_ticks * 2.25, hex_motor_ticks * 2.25, 0.7);
                break;
        }


        //Actions.runBlocking(new SequentialAction(TrajA5));

    }


}
