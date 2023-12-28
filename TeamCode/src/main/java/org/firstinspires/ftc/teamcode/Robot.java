/*
 * Handles all robot hardware (motors, servos).
 *
 * Motors/servos:
 *      - 2 drivetrain motors (same)
 *      - Arm motor, which pivots the linear slide
 *      - Linear slide motor, opens and closes linear slide
 *      - 1 wrist servo, 1 gripper servo
 *
 * Name of stuff:
 *      1. Right drive
 *      2. Left drive
 *      3. Right arm
 *      4. Left arm (arms are driven simultaneously, NEVER apart)
 *      5. Wrist servo
 *      6. Gripper servo (maybe multiple with new arrangement)
 *      FUTURE: Webcam, sensors
 *
 * Drive needs to be fluid.
 * Basic steps:
 *      1. Lowered to pick up pixel (open gripper)
 *      2. Lowered with pixel (closed gripper)
 *      3. Raised slightly to carry
 *      ... (drives to backboard)
 *      4. Flipped to meet backboard (only need variation when trying for mosaic)
 *
 * Necessary Positions (could be hard-coded here):
 *      1. Lowered
 *      2. Carrying
 *      3. Depositing
 *
 * Necessary Functionality (not for here):
 *      1. Drivetrain (always same speed???)
 *      2. Open/close gripper (binary, hopefully)
 *
 * Testing:
 *      - Make sure driveRobot can handle args outside of (-1, 1) range
 *
 * TODO: Implement linear slide motor
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Robot {
    // Counts Per Revolution = CPR
    public static final int CPR_DRIVE_MOTOR = 100; // TODO: Update with actual value
    public static final int CPR_ARM_MOTOR = 100; // Same

    public static final int ARM_COLLECTION_POS_COUNT = 0;
    public static final int ARM_SECURE_POS_COUNT = 100;
    public static final int ARM_DEPOSITION_POS_COUNT = 200;

    public static final double WRIST_COLLECTION_POS = 0.1;
    public static final double WRIST_SECURE_POS = 0.3;
    public static final double WRIST_DEPOSITION_POS = 0.5;

    // Might have to reverse
    public static final double GRIPPER_CLOSED_POS = 0.0;
    public static final double GRIPPER_OPEN_POS = 1.0;
    public static final double GRIPPER_SHORT_MOVEMENT = 0.05;

    public static final double MAX_DRIVE_POWER = 1;
    public static final double ARM_SPEED = 1;

    //
    private Servo wristServo = null;
    private Servo gripperServo = null;

    //
    private DcMotor rightDriveMotor = null;
    private DcMotor leftDriveMotor = null;
    private DcMotor armMotor = null;

    //
    private OpMode opMode = null;

    public Robot(OpMode opmode) {
        opMode = opmode;
    }

    public void init() {
        // TODO: Update mapped names
        leftDriveMotor = opMode.hardwareMap.get(DcMotor.class, "left_drive");
        rightDriveMotor = opMode.hardwareMap.get(DcMotor.class, "right_drive");
        armMotor = opMode.hardwareMap.get(DcMotor.class, "arm");

        leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        rightDriveMotor.setDirection(DcMotor.Direction.FORWARD);

        wristServo = opMode.hardwareMap.get(Servo.class, "wrist");
        gripperServo = opMode.hardwareMap.get(Servo.class, "gripper");

        opMode.telemetry.addData(">", "Hardware Initialized");
        opMode.telemetry.update();
    }

    /**
     * In position to grab pixel
     */
    public void moveToCollectionPosition() {
        moveMotor(armMotor, ARM_COLLECTION_POS_COUNT, ARM_SPEED);
        moveWristGripper(WRIST_COLLECTION_POS, GRIPPER_OPEN_POS);
    }

    /**
     * Secure position when moving (ideally to backboard)
     */
    public void moveToSecurePosition() {
        moveMotor(armMotor, ARM_SECURE_POS_COUNT, ARM_SPEED);
        moveWristGripper(WRIST_SECURE_POS, GRIPPER_CLOSED_POS);
    }

    /**
     * In position to place pixel on backboard
     */
    public void moveToDepositionPosition() {
        moveMotor(armMotor, ARM_DEPOSITION_POS_COUNT, ARM_SPEED);
        moveWristGripper(WRIST_DEPOSITION_POS, GRIPPER_CLOSED_POS);
    }

    /**
     * Moves motor to desired position at desired speed
     *
     * @param motor
     * @param position
     * @param power
     */
    private void moveMotor(DcMotor motor, int position, double power) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }

    /**
     * Moves hand and wrist servos to desired position
     *
     * @param wristPos
     * @param gripperPos
     */
    private void moveWristGripper(double wristPos, double gripperPos) {
        wristServo.setPosition(wristPos);
        gripperServo.setPosition(gripperPos);
    }

    private void openGripper() {
        gripperServo.setPosition(GRIPPER_OPEN_POS);
    }

    private void closeGripper() {
        gripperServo.setPosition(GRIPPER_CLOSED_POS);
    }

    public void openGripperShort() {
        // May have to change sign
        gripperServo.setPosition(gripperServo.getPosition() + GRIPPER_SHORT_MOVEMENT);
    }

    public void closeGripperShort() {
        // May have to change sign
        gripperServo.setPosition(gripperServo.getPosition() - GRIPPER_SHORT_MOVEMENT);
    }

    /**
     * Changes encoder settings on motors for TeleOp
     */
    public void prepareMotorsTeleOp() {
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * ONLY for TeleOp
     * Doesn't use encoders
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
}
