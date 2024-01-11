package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.util.MotorHelper;

public class Slide {
    private final int CPR_SLIDE = 1;

    private final double SLIDE_POWER = 1;

    private final int SLIDE_MIN_POS = 0;
    private final int SLIDE_MAX_POS = 200;

    private final int SLIDE_SHORT_MOVEMENT = 5;

    private final int SLIDE_COLLECTION_POS = 0;
    private final int SLIDE_SECURE_POS = 50;
    private final int SLIDE_DEPOSITION_POS = 100;

    private DcMotor slideMotor;

    private int currSlidePos;

    public Slide(HardwareMap hwMap) {
        slideMotor = hwMap.get(DcMotor.class, "slide");

        currSlidePos = 0;
    }

    /**
     * Changes encoder settings on motors
     *
     * @param useMotorPosition: Whether caller wants to use encoders to call getPos(), setPos()
     */
    public void prepareMotors(boolean useMotorPosition) {
        MotorHelper.prepareMotorEncoder(slideMotor, useMotorPosition);

        if (useMotorPosition) {
            currSlidePos = slideMotor.getCurrentPosition();
        }
    }

    public void moveToPosition(Robot.Positions pos) {
        switch(pos) {
            case COLLECT:
                MotorHelper.moveMotor(slideMotor, SLIDE_COLLECTION_POS, SLIDE_COLLECTION_POS);
                break;
            case SECURE:
                MotorHelper.moveMotor(slideMotor, SLIDE_SECURE_POS, SLIDE_COLLECTION_POS);
                break;
            case DEPOSIT:
                MotorHelper.moveMotor(slideMotor, SLIDE_DEPOSITION_POS, SLIDE_COLLECTION_POS);
                break;
        }
    }

    public void extendShort() {
        currSlidePos += Range.clip(currSlidePos + SLIDE_SHORT_MOVEMENT, SLIDE_MIN_POS, SLIDE_MAX_POS);
    }

    public void retractShort() {
        currSlidePos += Range.clip(currSlidePos - SLIDE_SHORT_MOVEMENT, SLIDE_MIN_POS, SLIDE_MAX_POS);
    }

    /**
     * VERY IMPORTANT
     * Moves the actual robot based on curr values
     */
    public void updatePositions() {
        MotorHelper.moveMotor(slideMotor, currSlidePos, SLIDE_POWER);
    }
}
