package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class TeleOpRobot extends Robot {
    private int currPivotPosCount;
    private int currArmPosCount;
    private double currWristPos;
    private double currGripperPos;

    public TeleOpRobot(OpMode opmode) {
        super(opmode);
        prepareMotors();
        initPositions();
    }

    //--------------------------------------------------------------------------
    // Initialization
    //--------------------------------------------------------------------------

    /**
     * Changes encoder settings on motors for TeleOp
     */
    private void prepareMotors() {
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Initializes position variables (so nothing suddenly moves to 0)
     */
    private void initPositions() {
        currPivotPosCount = pivotMotor.getCurrentPosition();
        currArmPosCount = armMotor.getCurrentPosition();
        currWristPos = wristServo.getPosition();
        currGripperPos = gripperServo.getPosition();
    }

    //--------------------------------------------------------------------------
    // Driving
    //--------------------------------------------------------------------------

    /**
     * Fluid driving w/ encoders
     *
     * @param drive: Raw forward/backward
     * @param turn:  Raw left/right
     */
    public void driveRobot(double drive, double turn) {
        // Combine drive and turn for blended motion.
        double left = drive + turn;
        double right = drive - turn;

        // Scale the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) {
            left /= max;
            right /= max;
        }

        // Scale for max speed (drive power)
        left = Range.scale(left, -1.0, 1.0, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
        right = Range.scale(right, -1.0, 1.0, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);

        leftDriveMotor.setPower(left);
        rightDriveMotor.setPower(right);
    }

    //--------------------------------------------------------------------------
    // Short Movements (may have to change sign)
    //--------------------------------------------------------------------------

    public void raisePivotShort() {
        addPosition(currPivotPosCount, PIVOT_SHORT_MOVEMENT, 0, Integer.MAX_VALUE);
    }

    public void lowerPivotShort() {
        addPosition(currPivotPosCount, -PIVOT_SHORT_MOVEMENT, 0, Integer.MAX_VALUE);
    }

    public void extendArmShort() {
        addPosition(currArmPosCount, ARM_SHORT_MOVEMENT, 0, Integer.MAX_VALUE);
    }

    public void retractArmShort() {
        addPosition(currArmPosCount, -ARM_SHORT_MOVEMENT, 0, Integer.MAX_VALUE);
    }

    public void raiseWristShort() {
        addPosition(currWristPos, WRIST_SHORT_MOVEMENT, 0, 1);
    }

    public void lowerWristShort() {
        addPosition(currWristPos, -WRIST_SHORT_MOVEMENT, 0, 1);
    }

    public void openGripperShort() {
        addPosition(currGripperPos, GRIPPER_SHORT_MOVEMENT, 0, 1);
    }

    public void closeGripperShort() {
        addPosition(currGripperPos, -GRIPPER_SHORT_MOVEMENT, 0, 1);
    }

    private void addPosition(int pos, int addend, int min, int max) {
        pos = Range.clip(pos + addend, min, max);
    }

    private void addPosition(double pos, double addend, int min, int max) {
        pos = Range.clip(pos + addend, min, max);
    }

    /**
     * VERY IMPORTANT
     * Moves the actual robot based on curr values
     */
    public void updateRobotPositions() {
        moveRobot(currPivotPosCount, currArmPosCount, currWristPos, currGripperPos);
    }
}
