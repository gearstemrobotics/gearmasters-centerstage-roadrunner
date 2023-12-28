package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "ColorSensorTest (Blocks to Java)")
public class ColorSensorTest extends LinearOpMode {

    private ColorSensor sidecolorsensor_REV_ColorRangeSensor;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        float ColorDetected;
        int Red;
        int Green;
        int Blue;

        sidecolorsensor_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "side color sensor");

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                telemetry.addData("Light Detected", ((OpticalDistanceSensor) sidecolorsensor_REV_ColorRangeSensor).getLightDetected());
                telemetry.addData("Red", sidecolorsensor_REV_ColorRangeSensor.red());
                telemetry.addData("Green", sidecolorsensor_REV_ColorRangeSensor.green());
                telemetry.addData("Blue", sidecolorsensor_REV_ColorRangeSensor.blue());
                ColorDetected = JavaUtil.rgbToValue(sidecolorsensor_REV_ColorRangeSensor.red(), sidecolorsensor_REV_ColorRangeSensor.green(), sidecolorsensor_REV_ColorRangeSensor.blue());
                Red = sidecolorsensor_REV_ColorRangeSensor.red();
                Green = sidecolorsensor_REV_ColorRangeSensor.green();
                Blue = sidecolorsensor_REV_ColorRangeSensor.blue();
                if (Red > Green && Red > Blue) {
                    telemetry.addData("ColorDetected", "Red");
                } else if (Green > Red && Green > Blue) {
                    telemetry.addData("ColorDetected", "Green");
                } else if (Blue > Red && Blue > Green) {
                    telemetry.addData("ColorDetected", "Blue");
                }
                telemetry.update();
            }
        }
    }
}