/*package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;

// Enigma code
// https://github.com/BudsterGaming/centerstage-code-16265/tree/McGreen

@TeleOp(name = "TryThisTeleOp")
public class TryThisTeleOp extends LinearOpMode {
    // Declare vars
    // timers
    private ElapsedTime runtime = new ElapsedTime();

    int RevCoreHex_CPR = 288;
    int RevCoreHex_RPM = 125;

    // motors
    Motor BackLeft = new Motor(hardwareMap, "BackLeft", RevCoreHex_CPR, RevCoreHex_RPM);
    Motor FrontLeft = new Motor(hardwareMap, "FrontLeft", RevCoreHex_CPR, RevCoreHex_RPM);
    Motor BackRight = new Motor(hardwareMap, "BackRight", RevCoreHex_CPR, RevCoreHex_RPM);
    Motor FrontRight = new Motor(hardwareMap, "BackRight", RevCoreHex_CPR, RevCoreHex_RPM);
    private DcMotor LeftPivot;
    private DcMotor RightPivot;
    private DcMotor Lift;
    //private DcMotor intake;

    MecanumDrive mecanum = new MecanumDrive(FrontLeft, FrontRight, BackLeft, BackRight);

    // servos
    //private CRServo thingy1066;
    private Servo plane;
    private Servo LeftClaw;
    private Servo RightClaw;
    private CRServo Wrist;
    //private CRServo thingy351;

    // servo values
    public static final double PLANE_INT = 1;
    public static final double PLANE_SHOOT = 0;
    public static final double LEFT_CLAW_OPEN = 1;
    public static final double LEFT_CLAW_CLOSE = 0;
    public static final double RIGHT_CLAW_OPEN = 0;
    public static final double RIGHT_CLAW_CLOSE = 1;

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
        mecanum.driveRobotCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_y);


    // launcher function
    /*private void plane() {
        if (gamepad2.dpad_up){
            Plane.setPosition(PLANE_SHOOT);
        } else{

        }
    }aas
*/
/*
    // arm function
        private void arm(){
            if (gamepad2.left_bumper) {
                LeftPivot.setPower(-gamepad2.left_stick_y * 0.3);
                RightPivot.setPower(-gamepad2.left_stick_y * 0.3);
            } else{
                LeftPivot.setPower(-gamepad2.left_stick_y * 0.75);
                RightPivot.setPower(-gamepad2.left_stick_y * 0.75);
            }
        }

    // Lift function
    /*private void lift() {
        if (gamepad2.x){
            Lift.setPower(0.5);
        }
        if (gamepad2.y){
            Lift.setPower(-0.5);
        }
    }
     */
/*
    //Lift function
    private void lift() {
        Lift.setPower(gamepad2.left_trigger);
        Lift.setPower(gamepad2.right_trigger * -1);
    }

*/
    // claw variables
    //double LeftClawPosition = robot.LeftClaw.getPosition();
    //double RightClawPosition = robot.RightClaw.getPosition();
    // claw function
    /*private void claws() {
        if (gamepad2.right_bumper){
            if (RightClawPosition == 0.2){
                RightClaw.setPosition(0.6);
            }
            else if (RightClawPosition == 0.6){
                RightClaw.setPosition(0.2);
            }
        }
        if (gamepad2.left_bumper){
            if (LeftClawPosition == 0.2){
                LeftClaw.setPosition(0.6);
            }
            else if (LeftClawPosition == 0.6){
                LeftClaw.setPosition(0.2);
            }
        }
    }*/
/*
    private void claws() {
        if (gamepad2.y){
            RightClaw.setPosition(RIGHT_CLAW_OPEN);
        }
        if (gamepad2.b){
            RightClaw.setPosition(RIGHT_CLAW_CLOSE);
        }
        if (gamepad2.x){
            LeftClaw.setPosition(LEFT_CLAW_OPEN);
        }
        if (gamepad2.a){
            LeftClaw.setPosition(LEFT_CLAW_CLOSE);
        }
    }


    // wrist function
    private void wrist() {
        Wrist.setPower(-gamepad2.right_stick_y);
        /*if (gamepad2.left_bumper){
            Wrist.setPosition(0);
        }
        if (gamepad2.right_bumper) {
            Wrist.setPosition(0.8);
        }*/
    //}
/*
    public void runOpMode() throws InterruptedException {
        // hardware maps
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        //thingy1066 = hardwareMap.get(CRServo.class, "thingy -106.6");
        plane = hardwareMap.get(Servo.class, "plane");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        LeftPivot = hardwareMap.get(DcMotor.class, "LeftPivot");
        RightPivot = hardwareMap.get(DcMotor.class, "RightPivot");
        Lift = hardwareMap.get(DcMotor.class, "Lift");
        LeftClaw = hardwareMap.get(Servo.class, "LeftClaw");
        RightClaw = hardwareMap.get(Servo.class, "RightClaw");
        Wrist = hardwareMap.get(CRServo.class, "Wrist");
        //intake = hardwareMap.get(DcMotor.class, "intake");
        //thingy351 = hardwareMap.get(CRServo.class, "thingy 35.1");

        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        LeftPivot.setDirection(DcMotor.Direction.REVERSE);
        //thingy1066.setDirection(CRServo.Direction.REVERSE);



        // Mecanum drive in a separate thread to avoid blocks in state machines like any loops (while/for) or sleep()
        MecanumDriveRunnable mecanumDriveRunnable = new MecanumDriveRunnable();
        Thread mecanumDriveThread = new Thread(mecanumDriveRunnable);

        //LeftClaw.setPosition(LEFT_CLAW_OPEN);
        //RightClaw.setPosition(RIGHT_CLAW_OPEN);

        waitForStart();
        runtime.reset();

        mecanumDriveThread.start();

        while(opModeIsActive()){
            plane();
            arm();
            lift();
            claws();
            wrist();

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

*/