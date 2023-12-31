// Tutorial.java
//
// Progressive tutorial file based on playlist from Artemis Robotics.
// https://www.youtube.com/playlist?list=PLnyefHqkc2K-gQD4hO5x3IcANuDuNMKDP (22-23 season)
// https://github.com/artemis18715/New-Programming-Tutorial-23-24 (23-24 season)

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//@TeleOp(name = "Tutorial Test")
public class Tutorial extends OpMode {

    DcMotor motor;

    @Override
    public void init() {
        // Send text to driver hub/phone
        telemetry.addData( "Initialization:", "Success!");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            telemetry.addData( "GamePad:", "The a button is pressed.");
            telemetry.update();
        }

        if (gamepad1.b) {
            telemetry.addData( "GamePad:", "The b button is pressed.");
            telemetry.update();
        }

    }

}