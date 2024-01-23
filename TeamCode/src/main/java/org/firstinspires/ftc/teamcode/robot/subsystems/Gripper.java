package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class Gripper {
    private final double GRIPPER_MIN_POS = 0.0;
    private final double GRIPPER_MAX_POS = 1.0;

    private final double GRIPPER_SHORT_MOVEMENT = 0.01;

    private final double GRIPPER_CLOSED_POS = 0.5;
    private final double GRIPPER_OPEN_POS = 1.0;

    private Servo leftServo;
    private Servo rightServo;

    private double currGripperPos;

    public Gripper(HardwareMap hwMap) {
        leftServo = hwMap.get(Servo.class, "gripper_left");
        rightServo = hwMap.get(Servo.class, "gripper_right");

        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);

        currGripperPos = leftServo.getPosition();
    }

    public double getPos() {
        return leftServo.getPosition();
    }

    public void moveToPosition(Robot.Positions pos) {
        switch (pos) {
            case COLLECT:
                currGripperPos = GRIPPER_OPEN_POS;
                break;
            case SECURE:
            case DEPOSIT:
                currGripperPos = GRIPPER_CLOSED_POS;
                break;
        }
    }

    public void openShort() {
        currGripperPos = Range.clip(currGripperPos + GRIPPER_SHORT_MOVEMENT, GRIPPER_MIN_POS, GRIPPER_MAX_POS);
    }

    public void closeShort() {
        currGripperPos = Range.clip(currGripperPos - GRIPPER_SHORT_MOVEMENT, GRIPPER_MIN_POS, GRIPPER_MAX_POS);
    }

    public void open() {
        currGripperPos = GRIPPER_OPEN_POS;
    }

    public void close() {
        currGripperPos = GRIPPER_CLOSED_POS;
    }

    /**
     * VERY IMPORTANT
     * Moves the actual robot based on curr values
     */
    public void updatePositions() {
        leftServo.setPosition(currGripperPos);
        rightServo.setPosition(currGripperPos);
    }
}
