package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@Autonomous(name = "FancyAutoAttemptTwo")
public class FancyAutoAttemptTwo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // runs once after init
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,Math.toRadians(90)));

        // loop until play button pressed
        while (!isStopRequested() && !opModeIsActive()){

        }

        // this is play button code that runs once
        // linetoy or linetox
        Actions.runBlocking(

                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(0, 24),Math.toRadians(180))
                        .build()

        );

        // loop until stop
        while(opModeIsActive() && !isStopRequested()){

        }

    }

}

