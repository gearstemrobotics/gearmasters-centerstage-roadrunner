package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// Enigma code
// https://github.com/BudsterGaming/centerstage-code-16265/tree/McGreen

@TeleOp(name = "NewJavaTest")
public class NewJavaTest extends LinearOpMode {
    // Declare vars
    // timers
    private ElapsedTime runtime = new ElapsedTime();

    // motors
    private DcMotor BackLeft;
    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor rightarm;
    private DcMotor BackRight;
    private DcMotor leftarm;
    private DcMotor intake;

    // servos
    private CRServo thingy1066;
    private Servo plane;
    private CRServo thingy351;

    // servo values
    public static final double PLANE_INT = 1;
    public static final double PLANE_SHOOT = 0;

    // Mecanum thread creation
    private class MecanumDriveRunnable implements Runnable {
        public volatile boolean running = true;

        @Override
        public void run() {
            while (running && opModeIsActive()) {
                try {
                    driveCode();
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    running = false; // Set running to false to stop the loop
                    Thread.currentThread().interrupt(); // Preserve interrupt status
                    break; // Break the loop to stop the thread
                }
            }
        }
    }

    // Meccanum drive
    private void driveCode() {
        // mecanum drive
        double forward = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);

        double FrontRightPower = (forward - strafe - turn) / denominator;
        double FrontLeftPower = (forward + strafe + turn) / denominator;
        double BackRightPower = (forward + strafe - turn) / denominator;
        double BackLeftPower = (forward - strafe + turn) / denominator;

        if (gamepad1.left_bumper) {
            FrontRightPower = Range.clip(FrontRightPower, -0.4, 0.4);
            FrontLeftPower = Range.clip(FrontLeftPower, -0.4, 0.4);
            BackRightPower = Range.clip(BackRightPower, -0.4, 0.4);
            BackLeftPower = Range.clip(BackLeftPower, -0.4, 0.4);
        } else {
            FrontRightPower = Range.clip(FrontRightPower, -1, 1);
            FrontLeftPower = Range.clip(FrontLeftPower, -1, 1);
            BackRightPower = Range.clip(BackRightPower, -1, 1);
            BackLeftPower = Range.clip(BackLeftPower, -1, 1);
        }

        FrontRight.setPower(FrontRightPower);
        FrontLeft.setPower(FrontLeftPower);
        BackRight.setPower(BackRightPower);
        BackLeft.setPower(BackLeftPower);
    }

    
    // launcher function
    private void plane() {
        if (gamepad2.right_bumper){
            plane.setPosition(PLANE_SHOOT);
        }
    }

    // arm function
    private void arm(){
            leftarm.setPower(gamepad2.left_stick_y * 0.75);
            rightarm.setPower(gamepad2.left_stick_y * 0.75);
    }

    public void runOpMode() throws InterruptedException {
        // hardware maps
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

        // Mecanum drive in a separate thread to avoid blocks in state machines like any loops (while/for) or sleep()
        MecanumDriveRunnable mecanumDriveRunnable = new MecanumDriveRunnable();
        Thread mecanumDriveThread = new Thread(mecanumDriveRunnable);

        waitForStart();
        runtime.reset();

        mecanumDriveThread.start();

        while(opModeIsActive()){
            plane();
            arm();

        }

        // Stop the mecanum drive thread after the op mode is over
        mecanumDriveRunnable.running = false;
        mecanumDriveThread.interrupt();
        try {
            mecanumDriveThread.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
