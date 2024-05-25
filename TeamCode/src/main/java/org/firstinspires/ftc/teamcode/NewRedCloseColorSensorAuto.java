package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

//@Autonomous(name = "NewRedCloseColorSensorAuto (Blocks to Java)")
public class NewRedCloseColorSensorAuto extends LinearOpMode {

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
        int Green;
        int Blue;
        float ColorDetected;
        int Red;

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
        thingy1066.setPower(-0.1);
        thingy1066.setPower(-0.1);
        sleep(500);
        arm(hex_motor_ticks * 0.1, hex_motor_ticks * 0.1, 0.5);
        drive(hex_motor_ticks * 6.5, -hex_motor_ticks * 6.5, -hex_motor_ticks * 6.5, hex_motor_ticks * 6.5, 0.5);
        thingy1066.setPower(-0.1);
        thingy351.setPower(-0.1);
        sleep(500);
        telemetry.addData("Light Detected", ((OpticalDistanceSensor) sidecolorsensor_REV_ColorRangeSensor).getLightDetected());
        telemetry.addData("Red", sidecolorsensor_REV_ColorRangeSensor.red());
        telemetry.addData("Green", sidecolorsensor_REV_ColorRangeSensor.green());
        telemetry.addData("Blue", sidecolorsensor_REV_ColorRangeSensor.blue());
        ColorDetected = JavaUtil.rgbToValue(sidecolorsensor_REV_ColorRangeSensor.red(), sidecolorsensor_REV_ColorRangeSensor.green(), sidecolorsensor_REV_ColorRangeSensor.blue());
        Red = sidecolorsensor_REV_ColorRangeSensor.red();
        Green = sidecolorsensor_REV_ColorRangeSensor.green();
        Blue = sidecolorsensor_REV_ColorRangeSensor.blue();
        sleep(1000);
        if (Red > 200) {
            telemetry.addData("ColorDetected", "RedSide");
            telemetry.update();
            drive(-hex_motor_ticks * 1, hex_motor_ticks * 1, hex_motor_ticks * 1, -hex_motor_ticks * 1, 0.5);
            drive(-hex_motor_ticks * 0.8, -hex_motor_ticks * 0.8, -hex_motor_ticks * 0.8, -hex_motor_ticks * 0.8, 0.5);
            drive(hex_motor_ticks * 4.25, hex_motor_ticks * 4.25, -hex_motor_ticks * 4.25, -hex_motor_ticks * 4.25, 0.5);
            drive(-hex_motor_ticks * 1.5, -hex_motor_ticks * 1.5, -hex_motor_ticks * 1.5, -hex_motor_ticks * 1.5, 0.5);
            intake.setPower(-0.75);
            FrontLeft.setPower(0.3);
            FrontRight.setPower(0.3);
            BackLeft.setPower(0.3);
            BackRight.setPower(0.3);
            sleep(1000);
            drive(-hex_motor_ticks * 4.25, -hex_motor_ticks * 4.25, hex_motor_ticks * 4.25, hex_motor_ticks * 4.25, 0.5);
            drive(-hex_motor_ticks * 5.5, hex_motor_ticks * 5.5, hex_motor_ticks * 5.5, -hex_motor_ticks * 5.5, 0.5);
            drive(hex_motor_ticks * 7, hex_motor_ticks * 7, hex_motor_ticks * 7, hex_motor_ticks * 7, 0.5);
            drive(hex_motor_ticks * 4.75, -hex_motor_ticks * 4.75, -hex_motor_ticks * 4.75, hex_motor_ticks * 4.75, 0.5);
            drive(hex_motor_ticks * 2.75, hex_motor_ticks * 2.75, hex_motor_ticks * 2.75, hex_motor_ticks * 2.75, 0.5);
            arm(hex_motor_ticks * 1, hex_motor_ticks * 1, 0.5);
            intake.setPower(0.5);
            thingy351.setPower(-0.15);
            thingy1066.setPower(-0.15);
            sleep(100);
            arm(hex_motor_ticks * 2.25, hex_motor_ticks * 2.25, 0.55);
            sleep(500);
            thingy351.setPower(-0.2);
            thingy1066.setPower(-0.2);
            sleep(50);
            FrontLeft.setPower(0);
            BackLeft.setPower(0);
            FrontRight.setPower(0);
            BackRight.setPower(0);
            leftarm.setPower(0);
            rightarm.setPower(0);
            thingy351.setPower(0);
            thingy1066.setPower(0);
            intake.setPower(0);
            sleep(30000);
        } else {
            drive(-hex_motor_ticks * 1, hex_motor_ticks * 1, hex_motor_ticks * 1, -hex_motor_ticks * 1, 0.5);
            drive(hex_motor_ticks * 1, hex_motor_ticks * 1, hex_motor_ticks * 1, hex_motor_ticks * 1, 0.5);
        }
        telemetry.addData("Light Detected", ((OpticalDistanceSensor) frontcolorsensor_REV_ColorRangeSensor).getLightDetected());
        telemetry.addData("Red", frontcolorsensor_REV_ColorRangeSensor.red());
        telemetry.addData("Green", frontcolorsensor_REV_ColorRangeSensor.green());
        telemetry.addData("Blue", frontcolorsensor_REV_ColorRangeSensor.blue());
        ColorDetected = JavaUtil.rgbToValue(frontcolorsensor_REV_ColorRangeSensor.red(), frontcolorsensor_REV_ColorRangeSensor.green(), frontcolorsensor_REV_ColorRangeSensor.blue());
        Red = frontcolorsensor_REV_ColorRangeSensor.red();
        Green = frontcolorsensor_REV_ColorRangeSensor.green();
        Blue = frontcolorsensor_REV_ColorRangeSensor.blue();
        sleep(500);
        if (Red > 200) {
            telemetry.addData("ColorDetected", "RedFront");
            telemetry.update();
            drive(hex_motor_ticks * 3, hex_motor_ticks * 3, hex_motor_ticks * 3, hex_motor_ticks * 3, 0.5);
            intake.setPower(-0.75);
            BackLeft.setPower(0.3);
            BackRight.setPower(0.3);
            BackLeft.setPower(0.3);
            BackRight.setPower(0.3);
            sleep(1000);
            drive(hex_motor_ticks * 6, hex_motor_ticks * 6, hex_motor_ticks * 6, hex_motor_ticks * 6, 0.5);
            drive(hex_motor_ticks * 1.5, -hex_motor_ticks * 1.5, -hex_motor_ticks * 1.5, hex_motor_ticks * 1.5, 0.5);
            arm(hex_motor_ticks * 1, hex_motor_ticks * 1, 0.5);
            intake.setPower(0.5);
            thingy351.setPower(-0.15);
            thingy1066.setPower(-0.15);
            sleep(100);
            arm(hex_motor_ticks * 2.25, hex_motor_ticks * 2.25, 0.5);
            sleep(500);
            thingy351.setPower(-0.2);
            thingy1066.setPower(-0.2);
            sleep(50);
            FrontLeft.setPower(0);
            BackLeft.setPower(0);
            FrontRight.setPower(0);
            BackRight.setPower(0);
            leftarm.setPower(0);
            rightarm.setPower(0);
            thingy351.setPower(0);
            thingy1066.setPower(0);
            intake.setPower(0);
            sleep(30000);
        } else {
            telemetry.addData("ColorDetected", "RedBack");
            telemetry.update();
            drive(-hex_motor_ticks * 1.25, -hex_motor_ticks * 1.25, -hex_motor_ticks * 1.25, -hex_motor_ticks * 1.25, 0.5);
            intake.setPower(-0.75);
            BackLeft.setPower(0.3);
            BackRight.setPower(0.3);
            BackLeft.setPower(0.3);
            BackRight.setPower(0.3);
            sleep(1000);
            drive(-hex_motor_ticks * 4.75, hex_motor_ticks * 4.75, hex_motor_ticks * 4.75, -hex_motor_ticks * 4.75, 0.5);
            drive(hex_motor_ticks * 7, hex_motor_ticks * 7, hex_motor_ticks * 7, hex_motor_ticks * 7, 0.5);
            drive(hex_motor_ticks * 4, -hex_motor_ticks * 4, -hex_motor_ticks * 4, hex_motor_ticks * 4, 0.5);
            drive(hex_motor_ticks * 2.75, hex_motor_ticks * 2.75, hex_motor_ticks * 2.75, hex_motor_ticks * 2.75, 0.5);
            arm(hex_motor_ticks * 1, hex_motor_ticks * 1, 0.5);
            intake.setPower(0.5);
            thingy351.setPower(-0.15);
            thingy1066.setPower(-0.15);
            sleep(100);
            arm(hex_motor_ticks * 2.25, hex_motor_ticks * 2.25, 0.55);
            sleep(500);
            thingy351.setPower(-0.2);
            thingy1066.setPower(-0.2);
            sleep(50);
            FrontLeft.setPower(0);
            BackLeft.setPower(0);
            FrontRight.setPower(0);
            BackRight.setPower(0);
            leftarm.setPower(0);
            rightarm.setPower(0);
            thingy351.setPower(0);
            thingy1066.setPower(0);
            intake.setPower(0);
            sleep(30000);
        }
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);
        leftarm.setPower(0);
        rightarm.setPower(0);
        intake.setPower(0);
        sleep(30000);
        telemetry.update();
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
