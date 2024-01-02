package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.util.MotorHelper;

public class Arm {
    private final int CPR_PIVOT = 1;
    private final int CPR_SLIDE = 1;

    private final double PIVOT_POWER = 1;
    private final double SLIDE_POWER = 1;

    private final int PIVOT_MIN_POS = 0;
    private final int PIVOT_MAX_POS = 200;
    private final int SLIDE_MIN_POS = 0;
    private final int SLIDE_MAX_POS = 200;

    private final int PIVOT_SHORT_MOVEMENT = 5;
    private final int SLIDE_SHORT_MOVEMENT = 5;

    private final int PIVOT_COLLECTION_POS = 0;
    private final int PIVOT_SECURE_POS = 50;
    private final int PIVOT_DEPOSITION_POS = 100;

    private final int SLIDE_COLLECTION_POS = 0;
    private final int SLIDE_SECURE_POS = 50;
    private final int SLIDE_DEPOSITION_POS = 100;

    private DcMotor pivotMotor;
    private DcMotor slideMotor;

    private int currPivotPos;
    private int currSlidePos;

    public Arm(HardwareMap hwMap) {
        pivotMotor = hwMap.get(DcMotor.class, "pivot");
        slideMotor = hwMap.get(DcMotor.class, "slide");

        currPivotPos = 0;
        currSlidePos = 0;
    }

    /**
     * Changes encoder settings on motors
     *
     * @param useMotorPosition: Whether caller wants to use encoders to call getPos(), setPos()
     */
    public void prepareMotors(boolean useMotorPosition) {
        MotorHelper.prepareMotorEncoder(pivotMotor, useMotorPosition);
        MotorHelper.prepareMotorEncoder(slideMotor, useMotorPosition);

        if (useMotorPosition) {
            currPivotPos = pivotMotor.getCurrentPosition();
            currSlidePos = slideMotor.getCurrentPosition();
        }
    }

    public void raisePivotShort() {
       currPivotPos += Range.clip(currPivotPos + PIVOT_SHORT_MOVEMENT, PIVOT_MIN_POS, PIVOT_MAX_POS);
    }

    public void lowerPivotShort() {
        currPivotPos += Range.clip(currPivotPos - PIVOT_SHORT_MOVEMENT, PIVOT_MIN_POS, PIVOT_MAX_POS);
    }

    public void extendSlideShort() {
        currSlidePos += Range.clip(currSlidePos + SLIDE_SHORT_MOVEMENT, SLIDE_MIN_POS, SLIDE_MAX_POS);
    }

    public void retractSlideShort() {
        currSlidePos += Range.clip(currSlidePos - SLIDE_SHORT_MOVEMENT, SLIDE_MIN_POS, SLIDE_MAX_POS);
    }

    /**
     * VERY IMPORTANT
     * Moves the actual robot based on curr values
     */
    public void updatePositions() {
        MotorHelper.moveMotor(pivotMotor, currPivotPos, PIVOT_POWER);
        MotorHelper.moveMotor(slideMotor, currSlidePos, SLIDE_POWER);
    }
}
