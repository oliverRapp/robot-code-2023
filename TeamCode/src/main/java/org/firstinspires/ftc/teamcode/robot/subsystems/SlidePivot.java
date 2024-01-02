package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.util.MotorHelper;

public class SlidePivot {
    private final int CPR_PIVOT = 1;

    private final double PIVOT_POWER = 1;

    private final int PIVOT_MIN_POS = 0;
    private final int PIVOT_MAX_POS = 200;

    private final int PIVOT_SHORT_MOVEMENT = 5;

    private final int PIVOT_COLLECTION_POS = 0;
    private final int PIVOT_SECURE_POS = 50;
    private final int PIVOT_DEPOSITION_POS = 100;

    private DcMotor pivotMotor;

    private int currPivotPos;

    public SlidePivot(HardwareMap hwMap) {
        pivotMotor = hwMap.get(DcMotor.class, "pivot");

        currPivotPos = 0;
    }

    /**
     * Changes encoder settings on motors
     *
     * @param useMotorPosition: Whether caller wants to use encoders to call getPos(), setPos()
     */
    public void prepareMotors(boolean useMotorPosition) {
        MotorHelper.prepareMotorEncoder(pivotMotor, useMotorPosition);

        if (useMotorPosition) {
            currPivotPos = pivotMotor.getCurrentPosition();
        }
    }

    public void moveToPosition(Robot.Positions pos) {
        switch(pos) {
            case COLLECT:
                MotorHelper.moveMotor(pivotMotor, PIVOT_COLLECTION_POS, PIVOT_POWER);
                break;
            case SECURE:
                MotorHelper.moveMotor(pivotMotor, PIVOT_SECURE_POS, PIVOT_POWER);
                break;
            case DEPOSIT:
                MotorHelper.moveMotor(pivotMotor, PIVOT_DEPOSITION_POS, PIVOT_POWER);
                break;
        }
    }

    public void raiseShort() {
        currPivotPos += Range.clip(currPivotPos + PIVOT_SHORT_MOVEMENT, PIVOT_MIN_POS, PIVOT_MAX_POS);
    }

    public void lowerShort() {
        currPivotPos += Range.clip(currPivotPos - PIVOT_SHORT_MOVEMENT, PIVOT_MIN_POS, PIVOT_MAX_POS);
    }

    /**
     * VERY IMPORTANT
     * Moves the actual robot based on curr values
     */
    public void updatePositions() {
        MotorHelper.moveMotor(pivotMotor, currPivotPos, PIVOT_POWER);
    }
}
