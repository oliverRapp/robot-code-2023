package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.Arm;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystems.Hand;

public class Robot {
    public Drivetrain drivetrain;
    public Arm arm;
    public Hand hand;

    public enum Positions {
        COLLECT,
        SECURE,
        DEPOSIT
    }

    public Robot(HardwareMap hwMap) {
        drivetrain = new Drivetrain(hwMap);
        arm = new Arm(hwMap);
        hand = new Hand(hwMap);
    }

    public void moveToPosition(Positions pos) {
        arm.moveToPosition(pos);
        hand.moveToPosition(pos);
    }

    public void updatePositions() {
        arm.updatePositions();
        hand.updatePositions();
    }
}
