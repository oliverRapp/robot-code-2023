package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.util.MotorHelper;

public class SlidePivot {
    private final int CPR_PIVOT = 1;

    private final double PIVOT_POWER = 0.8;

    private final int PIVOT_MIN_POS = 0;
    private final int PIVOT_MAX_POS = 490;

    private final int PIVOT_SHORT_MOVEMENT = 5;

    private final int PIVOT_COLLECTION_POS = 15;
    private final int PIVOT_SECURE_POS = 100;
    private final int PIVOT_DEPOSITION_POS = 300;

    private DcMotor pivotMotor;

    private int currPivotPos;

    public SlidePivot(HardwareMap hwMap) {
        pivotMotor = hwMap.get(DcMotor.class, "pivot");

        pivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        currPivotPos = 0;
    }

    public int getPos() {
        return pivotMotor.getCurrentPosition();
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
                currPivotPos = PIVOT_COLLECTION_POS;
                break;
            case SECURE:
                currPivotPos = PIVOT_SECURE_POS;
                break;
            case DEPOSIT:
                currPivotPos = PIVOT_DEPOSITION_POS;
                break;
        }
    }

    public void raiseShort() {
        currPivotPos = Range.clip(currPivotPos + PIVOT_SHORT_MOVEMENT, PIVOT_MIN_POS, PIVOT_MAX_POS);
    }

    public void lowerShort() {
        currPivotPos = Range.clip(currPivotPos - PIVOT_SHORT_MOVEMENT, PIVOT_MIN_POS, PIVOT_MAX_POS);
    }

    /**
     * VERY IMPORTANT
     * Moves the actual robot based on curr values
     */
    public void updatePositions() {
        MotorHelper.moveMotor(pivotMotor, currPivotPos, PIVOT_POWER);
    }
}
