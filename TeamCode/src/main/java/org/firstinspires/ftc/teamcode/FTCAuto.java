package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name = "Main Autonomous")
public class FTCAuto extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this.hardwareMap);

        robot.prepareMotorsAuto();

        robot.drivetrain.driveInches(12.0);
        robot.drivetrain.rotateRadians(Math.PI);

    }
}
