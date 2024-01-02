package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Hand {
    private final double WRIST_MIN_POS = 0;
    private final double WRIST_MAX_POS = 1;
    private final double GRIPPER_MIN_POS = 0;
    private final double GRIPPER_MAX_POS = 1;

    private final double WRIST_SHORT_MOVEMENT = 0.05;
    private final double GRIPPER_SHORT_MOVEMENT = 0.05;

    private final double WRIST_COLLECTION_POS = 0.1;
    private final double WRIST_SECURE_POS = 0.3;
    private final double WRIST_DEPOSITION_POS = 0.5;

    private final double GRIPPER_CLOSED_POS = 0.0;
    private final double GRIPPER_OPEN_POS = 1.0;

    private Servo wristServo;
    private Servo gripperServo;

    private double currWristPos;
    private double currGripperPos;

    public Hand(HardwareMap hwMap) {
        wristServo = hwMap.get(Servo.class, "wrist");
        gripperServo = hwMap.get(Servo.class, "gripper");

        currWristPos = 0;
        currGripperPos = 0;
    }

    public void raiseWristShort() {
        currWristPos = Range.clip(currWristPos + WRIST_SHORT_MOVEMENT, WRIST_MIN_POS, WRIST_MAX_POS);
    }

    public void lowerWristShort() {
        currWristPos = Range.clip(currWristPos - WRIST_SHORT_MOVEMENT, WRIST_MIN_POS, WRIST_MAX_POS);
    }

    public void openGripperShort() {
        currGripperPos = Range.clip(currGripperPos + GRIPPER_SHORT_MOVEMENT, GRIPPER_MIN_POS, GRIPPER_MAX_POS);
    }

    public void closeGripperShort() {
        currGripperPos = Range.clip(currGripperPos - GRIPPER_SHORT_MOVEMENT, GRIPPER_MIN_POS, GRIPPER_MAX_POS);
    }

    /**
     * VERY IMPORTANT
     * Moves the actual robot based on curr values
     */
    public void updatePositions() {
        wristServo.setPosition(currWristPos);
        gripperServo.setPosition(currGripperPos);
    }
}
