package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Autonomous(name = "ThisOne (Blocks to Java)")
public class ThisOne extends LinearOpMode {

    private DcMotor FrontRight;
    private DcMotor BackRight;
    private DcMotor FrontLeft;
    private DcMotor BackLeft;
    private DcMotor leftarm;
    private DcMotor rightarm;
    private CRServo thingy1066;
    private CRServo thingy351;
    private ColorSensor sidecolorsensor_REV_ColorRangeSensor;
    private DcMotor intake;
    private ColorSensor frontcolorsensor_REV_ColorRangeSensor;

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

        FrontRight = hardwareMap.get(DcMotor.class, "Front Right");
        BackRight = hardwareMap.get(DcMotor.class, "Back Right");
        FrontLeft = hardwareMap.get(DcMotor.class, "Front Left");
        BackLeft = hardwareMap.get(DcMotor.class, "Back Left");
        leftarm = hardwareMap.get(DcMotor.class, "left arm");
        rightarm = hardwareMap.get(DcMotor.class, "right arm");
        thingy1066 = hardwareMap.get(CRServo.class, "thingy -106.6");
        thingy351 = hardwareMap.get(CRServo.class, "thingy 35.1");
        sidecolorsensor_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "side color sensor");
        intake = hardwareMap.get(DcMotor.class, "intake");
        frontcolorsensor_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "front color sensor");

        // Put initialization blocks here.
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        leftarm.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightarm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightarm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(500);
        hex_motor_ticks = 288;
        FrontRightPos = 0;
        BackRightPos = 0;
        FrontLeftPos = 0;
        BackLeftPos = 0;
        LeftArmPos = 0;
        RightArmPos = 0;
        BackLeft.setPower(0);
        BackRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
        sleep(10000);
        drive(hex_motor_ticks * 1.25, -hex_motor_ticks * 1.25, -hex_motor_ticks * 1.25, hex_motor_ticks * 1.25, 0.5);
        drive(hex_motor_ticks * 16, hex_motor_ticks * 16, hex_motor_ticks * 16, hex_motor_ticks * 16, 0.5);
    }

    /**
     * Describe this function...
     */
    private void arm(double LeftArmTarget, double RightArmTarget, double Speed) {
        LeftArmPos += LeftArmTarget;
        RightArmPos += RightArmTarget;
        leftarm.setTargetPosition(LeftArmPos);
        rightarm.setTargetPosition(RightArmPos);
        leftarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightarm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftarm.setPower(Speed);
        rightarm.setPower(Speed);
        while (opModeIsActive() && leftarm.isBusy() && rightarm.isBusy()) {
            // Do nothing
        }
        leftarm.setPower(0);
        rightarm.setPower(0);
    }
}
