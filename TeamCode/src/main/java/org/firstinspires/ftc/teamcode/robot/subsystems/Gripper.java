package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class Gripper {
    private final double GRIPPER_MIN_POS = 0;
    private final double GRIPPER_MAX_POS = 1;

    private final double GRIPPER_SHORT_MOVEMENT = 0.05;

    private final double GRIPPER_CLOSED_POS = 0.0;
    private final double GRIPPER_OPEN_POS = 1.0;

    private final Servo gripperServo;

    private double currGripperPos;

    public Gripper(HardwareMap hwMap) {
        gripperServo = hwMap.get(Servo.class, "gripper");

        currGripperPos = 0;
    }

    public void moveToPosition(Robot.Positions pos) {
        switch (pos) {
            case COLLECT:
                gripperServo.setPosition(GRIPPER_OPEN_POS);
                break;
            case SECURE:
            case DEPOSIT:
                gripperServo.setPosition(GRIPPER_CLOSED_POS);
                break;
        }
    }

    public void openShort() {
        currGripperPos = Range.clip(currGripperPos + GRIPPER_SHORT_MOVEMENT, GRIPPER_MIN_POS, GRIPPER_MAX_POS);
    }

    public void closeShort() {
        currGripperPos = Range.clip(currGripperPos - GRIPPER_SHORT_MOVEMENT, GRIPPER_MIN_POS, GRIPPER_MAX_POS);
    }

    /**
     * VERY IMPORTANT
     * Moves the actual robot based on curr values
     */
    public void updatePositions() {
        gripperServo.setPosition(currGripperPos);
    }
}
