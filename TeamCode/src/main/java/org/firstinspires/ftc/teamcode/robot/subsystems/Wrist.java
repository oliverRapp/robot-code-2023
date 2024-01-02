package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class Wrist {
    private final double WRIST_MIN_POS = 0;
    private final double WRIST_MAX_POS = 1;

    private final double WRIST_SHORT_MOVEMENT = 0.05;

    private final double WRIST_COLLECTION_POS = 0.1;
    private final double WRIST_SECURE_POS = 0.3;
    private final double WRIST_DEPOSITION_POS = 0.5;

    private final Servo wristServo;

    private double currWristPos;

    public Wrist(HardwareMap hwMap) {
        wristServo = hwMap.get(Servo.class, "wrist");

        currWristPos = 0;
    }

    public void moveToPosition(Robot.Positions pos) {
        switch (pos) {
            case COLLECT:
                wristServo.setPosition(WRIST_COLLECTION_POS);
                break;
            case SECURE:
                wristServo.setPosition(WRIST_SECURE_POS);
                break;
            case DEPOSIT:
                wristServo.setPosition(WRIST_DEPOSITION_POS);
                break;
        }
    }

    public void raiseShort() {
        currWristPos = Range.clip(currWristPos + WRIST_SHORT_MOVEMENT, WRIST_MIN_POS, WRIST_MAX_POS);
    }

    public void lowerShort() {
        currWristPos = Range.clip(currWristPos - WRIST_SHORT_MOVEMENT, WRIST_MIN_POS, WRIST_MAX_POS);
    }

    /**
     * VERY IMPORTANT
     * Moves the actual robot based on curr values
     */
    public void updatePositions() {
        wristServo.setPosition(currWristPos);
    }
}
