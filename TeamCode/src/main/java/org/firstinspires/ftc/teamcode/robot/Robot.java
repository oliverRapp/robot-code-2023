package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystems.Arm;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystems.Hand;

public class Robot {
    public Drivetrain drivetrain;
    public Arm arm;
    public Hand hand;

    public Robot(HardwareMap hwMap) {
        drivetrain = new Drivetrain(hwMap);
        arm = new Arm(hwMap);
        hand = new Hand(hwMap);
    }

}
