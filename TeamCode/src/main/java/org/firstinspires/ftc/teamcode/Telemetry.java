package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "DebuggerTelemetry")
public class Telemetry extends OpMode {

    int RevCoreHex_CPR = 288;
    int RevCoreHex_RPM = 125;
    Motor Lift = new Motor(hardwareMap, "Lift", RevCoreHex_CPR, RevCoreHex_RPM);
    @Override
    public void init() {
        Lift.setRunMode(Motor.RunMode.PositionControl);
    }

    @Override
    public void loop() {

        telemetry.addData("Liftpos", Lift.getCurrentPosition());

    }
}
