package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.util.IMUnit;
import org.firstinspires.ftc.teamcode.robot.util.MotorHelper;

public class Drivetrain {
    private final int CPR = 28;
    private final double WHEEL_DIAMETER_INCHES = 90 / 25.4;
    private final double COUNTS_PER_INCH = CPR / (WHEEL_DIAMETER_INCHES * Math.PI);

    private final double MAX_DRIVE_POWER = 1;

    private final double MAX_TURN_POWER = 0.5;
    private final double MIN_TURN_POWER = 0.05;
    private final double TURN_ACCEPTABLE_ERROR = Math.PI / 180;


    private DcMotor rightMotor;
    private DcMotor leftMotor;

    public Drivetrain(HardwareMap hwMap) {
        rightMotor = hwMap.get(DcMotor.class, "right_drive");
        leftMotor = hwMap.get(DcMotor.class, "left_drive");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Changes encoder settings on motors
     *
     * @param useMotorPosition: Whether caller wants to use encoders to call getPos(), setPos()
     */
    public void prepareMotors(boolean useMotorPosition) {
        MotorHelper.prepareMotorEncoder(rightMotor, useMotorPosition);
        MotorHelper.prepareMotorEncoder(leftMotor, useMotorPosition);
    }

    /**
     * Fluid driving w/ encoders
     *
     * @param drive: Raw forward/backward
     * @param turn:  Raw left/right
     */
    public void driveFluid(double drive, double turn) {
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
        rightMotor.setPower(left);
        leftMotor.setPower(right);
    }

    public void driveInches(double inches) {
        prepareMotors(true);

        int targetCount = (int) Math.round(inches * COUNTS_PER_INCH);
        MotorHelper.moveMotor(leftMotor, targetCount, MAX_DRIVE_POWER);
        MotorHelper.moveMotor(rightMotor, targetCount, MAX_DRIVE_POWER);

        while (leftMotor.isBusy() || rightMotor.isBusy()) {}
    }

    /**
     * @param target: +: left, -: right
     */
    public void rotateRadians(double target) {
        prepareMotors(false);

        int dir = (int) Math.signum(target);

        IMUnit.resetAngle();

        double error = Math.abs(IMUnit.getAngle()) - Math.abs(target);
        while (error >= TURN_ACCEPTABLE_ERROR) {
            double power = Range.scale(error, TURN_ACCEPTABLE_ERROR, Math.PI, MIN_TURN_POWER, MAX_TURN_POWER);

            leftMotor.setPower(power * -dir);
            rightMotor.setPower(power * dir);
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }
}
