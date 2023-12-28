package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Centerstage (Blocks to Java)")
public class Centerstage extends LinearOpMode {

    private DcMotor BackLeft;
    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor rightarm;
    private CRServo thingy1066;
    private Servo plane;
    private DcMotor BackRight;
    private DcMotor leftarm;
    private DcMotor intake;
    private CRServo thingy351;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int hex_motor_ticks;
        int Right_Arm;
        int Left_Arm;
        float vertical;
        float horizontal;
        float pivot;

        BackLeft = hardwareMap.get(DcMotor.class, "Back Left");
        FrontRight = hardwareMap.get(DcMotor.class, "Front Right");
        FrontLeft = hardwareMap.get(DcMotor.class, "Front Left");
        rightarm = hardwareMap.get(DcMotor.class, "right arm");
        thingy1066 = hardwareMap.get(CRServo.class, "thingy -106.6");
        plane = hardwareMap.get(Servo.class, "plane");
        BackRight = hardwareMap.get(DcMotor.class, "Back Right");
        leftarm = hardwareMap.get(DcMotor.class, "left arm");
        intake = hardwareMap.get(DcMotor.class, "intake");
        thingy351 = hardwareMap.get(CRServo.class, "thingy 35.1");

        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        rightarm.setDirection(DcMotor.Direction.REVERSE);
        thingy1066.setDirection(CRServo.Direction.REVERSE);
        plane.setPosition(1);
        hex_motor_ticks = 288;
        Right_Arm = 0;
        Left_Arm = 0;
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                vertical = -gamepad1.right_stick_y;
                horizontal = gamepad1.right_stick_x;
                pivot = gamepad1.left_stick_x;
                FrontRight.setPower((-pivot + (vertical - horizontal)) * 0.8);
                BackRight.setPower((-pivot + vertical + horizontal) * 0.8);
                FrontLeft.setPower((pivot + vertical + horizontal) * 0.8);
                BackLeft.setPower((pivot + (vertical - horizontal)) * 0.8);
                leftarm.setPower(gamepad2.left_stick_y * 0.75);
                rightarm.setPower(gamepad2.left_stick_y * 0.75);
                intake.setPower(gamepad2.left_trigger);
                intake.setPower(-gamepad2.right_trigger);
                BackLeft.setTargetPosition((int) 0.5);
                if (gamepad2.x) {
                    intake.setPower(0.75);
                }
                if (gamepad2.y) {
                    intake.setPower(-0.75);
                }
                if (gamepad2.b) {
                    thingy351.setPower(-0.4);
                    thingy1066.setPower(-0.4);
                } else if (gamepad2.a) {
                    thingy351.setPower(0.4);
                    thingy1066.setPower(0.4);
                } else {
                    thingy351.setPower(0);
                    thingy1066.setPower(0);
                }
                if (gamepad1.dpad_right) {
                    FrontRight.setPower(-1);
                    BackRight.setPower(1);
                    FrontLeft.setPower(1);
                    BackLeft.setPower(-1);
                }
                if (gamepad1.dpad_left) {
                    FrontRight.setPower(1);
                    BackRight.setPower(-1);
                    FrontLeft.setPower(-1);
                    BackLeft.setPower(1);
                }
                if (gamepad2.left_bumper) {
                    leftarm.setPower(-0.75);
                    rightarm.setPower(-0.75);
                    sleep(1500);
                }
                if (gamepad2.right_bumper) {
                    plane.setPosition(0);
                }
            }
        }
    }
}
