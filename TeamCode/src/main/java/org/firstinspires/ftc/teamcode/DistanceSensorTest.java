package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "DistanceSensorTest")
public class DistanceSensorTest extends LinearOpMode{
    private DistanceSensor Distance;

    // center is 47-50 away
    // sides are 35-38 away


    @Override
    public void runOpMode() {
        Distance = hardwareMap.get(DistanceSensor.class, "Distance");


        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                telemetry.addData("Distance", Distance.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
        }
    }
}

