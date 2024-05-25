package org.firstinspires.ftc.teamcode;

//import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
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

//@TeleOp(name = "BetterJavaCenterstageHopefully")
public class BetterJavaCenterstageHopefully extends LinearOpMode {
    // Declare varss
    // timers
    private ElapsedTime runtime = new ElapsedTime();

    int RevCoreHex_CPR = 288;
    int RevCoreHex_RPM = 125;

    // motors
    Motor BackLeft = new Motor(hardwareMap, "BackLeft", RevCoreHex_CPR, RevCoreHex_RPM);
    Motor FrontLeft = new Motor(hardwareMap, "FrontLeft", RevCoreHex_CPR, RevCoreHex_RPM);
    Motor BackRight = new Motor(hardwareMap, "BackRight", RevCoreHex_CPR, RevCoreHex_RPM);
    Motor FrontRight = new Motor(hardwareMap, "BackRight", RevCoreHex_CPR, RevCoreHex_RPM);
    Motor LeftPivot = new Motor(hardwareMap, "LeftPivot", RevCoreHex_CPR, RevCoreHex_RPM);
    Motor RightPivot = new Motor(hardwareMap, "RightPivot", RevCoreHex_CPR, RevCoreHex_RPM);
    Motor Lift = new Motor(hardwareMap, "Lift", RevCoreHex_CPR, RevCoreHex_RPM);
    MotorGroup pivot;


    MecanumDrive mecanum = new MecanumDrive(FrontLeft, FrontRight, BackLeft, BackRight);
    GamepadEx driverOP;
    GamepadEx controlsOP;

    private enum RobotState {preGrab, grab, transport, preScore, Score, plane, preClimb, Climb}
    RobotState robotState = RobotState.transport;
    private enum ClawState {open, closed}
    ClawState clawState = ClawState.open;
    private enum WristState {climbpos, intakepos, scorepos, transportpos}
    WristState wristState = WristState.climbpos;
    private enum ArmState {intakepos, levelOne, levelTwo, levelThree, preclimbpos, climbpos}
    ArmState armState = ArmState.climbpos;

    ElapsedTime transportTimer;
    ElapsedTime scoreTimer;


    // servos
    ServoEx Plane = new SimpleServo(hardwareMap, "Plane", 0, 180);
    ServoEx LeftClaw = new SimpleServo(hardwareMap, "LeftClaw", 0, 180);
    ServoEx RightClaw = new SimpleServo(hardwareMap, "RightClaw", 0, 180);
    ServoEx Wrist = new SimpleServo(hardwareMap, "Wrist", 0, 160);

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

    // Mecanum drive
    private void driveCode() {
        // mecanum drive
        mecanum.driveRobotCentric(driverOP.getLeftX(), driverOP.getLeftY(), driverOP.getRightY());
    }



    // manual override
    private void manualOP(){

        pivot.setRunMode(Motor.RunMode.RawPower);
        Lift.setRunMode(Motor.RunMode.RawPower);

        if (gamepad2.dpad_up){
            Plane.setPosition(PLANE_SHOOT);
        }

        if (gamepad2.left_trigger > 0.2) {
            pivot.set(-gamepad2.left_stick_y * 0.1);
        } else{
            pivot.set(-gamepad2.left_stick_y * 0.75);
        }

        Lift.set(controlsOP.getRightY());

        Wrist.rotateByAngle(controlsOP.getRightX());

        if (gamepad2.y){
            RightClaw.setPosition(0);
        }
        if (gamepad2.b){
            RightClaw.setPosition(1);
        }
        if (gamepad2.x){
            LeftClaw.setPosition(1);
        }
        if (gamepad2.a){
            LeftClaw.setPosition(0);
        }
    }

    // launcher function
    private void plane() {
        if (gamepad2.dpad_up){
            Plane.setPosition(PLANE_SHOOT);
        }


    }

    // arm function
    // intakepos, levelOne, levelTwo, levelThree, preclimbpos, climbpos
    private void arm(){
        switch (armState){

            case intakepos:
                Lift.setTargetPosition(0);
                pivot.setTargetPosition(-100);


                break;

            case levelOne:

                break;

            case levelTwo:

                break;

            case levelThree:

                break;

            case preclimbpos:

                break;

            case climbpos:

                break;
        }
    }




    private void claws() {
        if (gamepad2.y){
            RightClaw.setPosition(0);
        }
        if (gamepad2.b){
            RightClaw.setPosition(1);
        }
        if (gamepad2.x){
            LeftClaw.setPosition(1);
        }
        if (gamepad2.a){
            LeftClaw.setPosition(0);
        }
    }


    // wrist function
    // intakepos, scorepos, transportpos
    private void wrist() {
        switch (wristState) {

            case climbpos:
                Wrist.setPosition(0);
                break;

            case intakepos:
                Wrist.setPosition(40);
                break;

            case scorepos:
                Wrist.setPosition(20);
                break;

            case transportpos:
                Wrist.setPosition(65);
                break;


        }
    }

    private void intake() {

        if (robotState == RobotState.preGrab) {

            wristState = WristState.intakepos;
            armState = ArmState.intakepos;
            if (intake.wasJustPressed()){

                if (clawState == ClawState.open){

                    clawState = ClawState.closed;

                }

                if (clawState == ClawState.closed){

                    robotState = RobotState.transport;

                }

            }

        }

    }


    ButtonReader intake = new ButtonReader(controlsOP, GamepadKeys.Button.LEFT_BUMPER);
    //ButtonReader grab = new ButtonReader(controlsOP, GamepadKeys.Button.RIGHT_BUMPER);
    ButtonReader scoreOne = new ButtonReader(controlsOP, GamepadKeys.Button.X);
    ButtonReader scoreTwo = new ButtonReader(controlsOP, GamepadKeys.Button.Y);
    ButtonReader scoreThree = new ButtonReader(controlsOP, GamepadKeys.Button.B);
    ButtonReader preClimb = new ButtonReader(controlsOP, GamepadKeys.Button.DPAD_UP);
    ButtonReader climb = new ButtonReader(controlsOP, GamepadKeys.Button.DPAD_DOWN);
    ButtonReader plane = new ButtonReader(controlsOP, GamepadKeys.Button.DPAD_LEFT);
    ToggleButtonReader manualOverride = new ToggleButtonReader(controlsOP, GamepadKeys.Button.DPAD_RIGHT);
    TriggerReader planeSafety = new TriggerReader(controlsOP, GamepadKeys.Trigger.RIGHT_TRIGGER);

    private void readGamePad(){

        intake.readValue();
        scoreOne.readValue();
        scoreTwo.readValue();
        scoreThree.readValue();
        preClimb.readValue();
        climb.readValue();
        plane.readValue();
        manualOverride.readValue();
        planeSafety.readValue();


    }

    public void runOpMode() throws InterruptedException {
        driverOP = new GamepadEx(gamepad1);
        controlsOP = new GamepadEx(gamepad2);

        transportTimer = new ElapsedTime();
        scoreTimer = new ElapsedTime();

        LeftPivot.setInverted(true);
        pivot = new MotorGroup(LeftPivot, RightPivot);
        pivot.setRunMode(Motor.RunMode.PositionControl);
        pivot.setTargetPosition(0);
        pivot.setPositionTolerance(20);
        pivot.set(0.5);
        BackRight.setInverted(true);

        Lift.resetEncoder();
        Lift.setRunMode(Motor.RunMode.PositionControl);
        Lift.setTargetPosition(0);
        Lift.setPositionTolerance(20);
        Lift.set(1);




        // Mecanum drive in a separate thread to avoid blocks in state machines like any loops (while/for) or sleep()
        MecanumDriveRunnable mecanumDriveRunnable = new MecanumDriveRunnable();
        Thread mecanumDriveThread = new Thread(mecanumDriveRunnable);


        waitForStart();
        runtime.reset();

        mecanumDriveThread.start();

        while(opModeIsActive()){

            if (manualOverride.getState()){
                manualOP();
            }
            else{
                Lift.setRunMode(Motor.RunMode.PositionControl);
                pivot.setRunMode(Motor.RunMode.PositionControl);
                Lift.set(1);
                pivot.set(0.5);

                plane();
                arm();
                claws();
                wrist();
            }

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

