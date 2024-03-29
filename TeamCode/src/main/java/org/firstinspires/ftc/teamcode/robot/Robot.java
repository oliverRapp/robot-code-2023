package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drone;
import org.firstinspires.ftc.teamcode.robot.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.robot.subsystems.Slide;
import org.firstinspires.ftc.teamcode.robot.subsystems.SlidePivot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Wrist;

public class Robot {
    public enum Positions {
        COLLECT,
        SECURE,
        DEPOSIT
    }

    public Drivetrain drivetrain;
    public SlidePivot pivot;
    public Slide slide;
    public Wrist wrist;
    public Gripper gripper;
    public Drone drone;

    public Robot(HardwareMap hwMap) {
        drivetrain = new Drivetrain(hwMap);
        pivot = new SlidePivot(hwMap);
        slide = new Slide(hwMap);
        wrist = new Wrist(hwMap);
        gripper = new Gripper(hwMap);
        drone = new Drone(hwMap);

    }

    public void moveToPosition(Positions pos) {
        pivot.moveToPosition(pos);
        slide.moveToPosition(pos);
        wrist.moveToPosition(pos);
        gripper.moveToPosition(pos);
    }

    public void prepareMotorsTeleOp() {
        drivetrain.prepareMotors(false);
        slide.prepareMotors(true);
        pivot.prepareMotors(true);
    }

    public void prepareMotorsAuto() {
        slide.prepareMotors(true);
        pivot.prepareMotors(true);
    }

    public void updatePositions() {
        pivot.updatePositions();
        slide.updatePositions();
        wrist.updatePositions();
        gripper.updatePositions();
    }
}
