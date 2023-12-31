package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

    @Autonomous(name = "JavaRedCloseAuto")
    public class JavaRedCloseAuto extends LinearOpMode {

        // Declare vars
        // timers
        private ElapsedTime runtime = new ElapsedTime();

        // motors
        private DcMotor BackLeft;
        private DcMotor FrontRight;
        private DcMotor FrontLeft;
        private DcMotor BackRight;
        private DcMotor LeftPivot;
        private DcMotor RightPivot;
        private DcMotor Lift;
        //private DcMotor intake;

        // servos
        //private CRServo thingy1066;
        private Servo plane;
        //private CRServo thingy351;

        // sensors
        private ColorSensor sidecolorsensor_REV_ColorRangeSensor;
        private ColorSensor frontcolorsensor_REV_ColorRangeSensor;

        // servo values
        public static final double PLANE_INT = 1;
        public static final double PLANE_SHOOT = 0;

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

            FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
            BackRight = hardwareMap.get(DcMotor.class, "BackRight");
            FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
            BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
            LeftPivot = hardwareMap.get(DcMotor.class, "LeftPivot");
            RightPivot = hardwareMap.get(DcMotor.class, "RightPivot");
            Lift = hardwareMap.get(DcMotor.class, "Lift");
            sidecolorsensor_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "side color sensor");
            frontcolorsensor_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "front color sensor");

            // Put initialization blocks here.
            BackRight.setDirection(DcMotor.Direction.REVERSE);
            RightPivot.setDirection(DcMotor.Direction.REVERSE);
            waitForStart();
            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RightPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LeftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LeftPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            RightPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            sleep(500);
            hex_motor_ticks = 288;
            FrontRightPos = 0;
            BackRightPos = 0;
            FrontLeftPos = 0;
            BackLeftPos = 0;
            LeftArmPos = 0;
            RightArmPos = 0;
            sleep(500);
            arm(hex_motor_ticks * 0.1, hex_motor_ticks * 0.1, 0.5);
            drive(hex_motor_ticks * 6.5, -hex_motor_ticks * 6.5, -hex_motor_ticks * 6.5, hex_motor_ticks * 6.5, 0.5);
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
                BackLeft.setPower(0.3);
                BackRight.setPower(0.3);
                BackLeft.setPower(0.3);
                BackRight.setPower(0.3);
                sleep(1000);
                drive(hex_motor_ticks * 6, hex_motor_ticks * 6, hex_motor_ticks * 6, hex_motor_ticks * 6, 0.5);
                drive(hex_motor_ticks * 1.5, -hex_motor_ticks * 1.5, -hex_motor_ticks * 1.5, hex_motor_ticks * 1.5, 0.5);
                arm(hex_motor_ticks * 1, hex_motor_ticks * 1, 0.5);
                sleep(100);
                arm(hex_motor_ticks * 2.25, hex_motor_ticks * 2.25, 0.5);
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
                telemetry.addData("ColorDetected", "RedBack");
                telemetry.update();
                drive(-hex_motor_ticks * 1.25, -hex_motor_ticks * 1.25, -hex_motor_ticks * 1.25, -hex_motor_ticks * 1.25, 0.5);
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

        /**
         * Describe this function...
         */
        private void arm(double LeftArmTarget, double RightArmTarget, double Speed) {
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

