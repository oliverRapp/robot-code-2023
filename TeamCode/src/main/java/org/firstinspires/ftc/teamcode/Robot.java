/*
 * Handles all robot hardware (motors, servos).
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
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Robot {
    // TODO: Actually use these constants (should be changed here, not in teleOp code);
    public static final double MID_SERVO = 0.5;
    public static final double HAND_SPEED = 0.02;  // sets rate to move servo
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;

    public static final double MAX_DRIVE_POWER = 1;

    private OpMode opMode = null;

    private DcMotor rightDriveMotor = null;
    private DcMotor leftDriveMotor = null;
    private DcMotor rightArmMotor = null;
    private DcMotor leftArmMotor = null;
    private Servo wristServo = null;
    private Servo gripperServo = null;

    // most similar to gripperServo, used as guide
    private Servo leftHand = null;
    private Servo rightHand = null;

    public Robot(OpMode opmode) {
        opMode = opmode;
    }

    public void init() {
        // TODO: Update mapped names
        leftDriveMotor = opMode.hardwareMap.get(DcMotor.class, "left_drive");
        rightDriveMotor = opMode.hardwareMap.get(DcMotor.class, "right_drive");
        rightArmMotor = opMode.hardwareMap.get(DcMotor.class, "arm");
        leftArmMotor = opMode.hardwareMap.get(DcMotor.class, "arm");

        leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        rightDriveMotor.setDirection(DcMotor.Direction.FORWARD);

        // Is this necessary?
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftHand = opMode.hardwareMap.get(Servo.class, "left_hand");
        rightHand = opMode.hardwareMap.get(Servo.class, "right_hand");

        // Is this necessary?
        leftHand.setPosition(MID_SERVO);
        rightHand.setPosition(MID_SERVO);

        opMode.telemetry.addData(">", "Hardware Initialized");
        opMode.telemetry.update();
    }

    public void driveRobot(double drive, double turn) {
        // Combine drive and turn for blended motion.
        double left = drive + turn;
        double right = drive - turn;

        // Scale the values so neither exceed +/- 1.0
        double max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) {
            left /= max;
            right /= max;
            max = Math.max(Math.abs(left), Math.abs(right));
        }

        // Scale for max speed (drive power)
        left = Range.scale(left, -1.0, 1.0, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
        left = Range.scale(right, -1.0, 1.0, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);

        setDrivePower(left, right);
    }

    public void setDrivePower(double leftWheel, double rightWheel) {
        leftDriveMotor.setPower(leftWheel);
        rightDriveMotor.setPower(rightWheel);
    }

    // All wrong below, we're using diff config

    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param power driving power (-1.0 to 1.0)
     */
    public void setArmPower(double power) {
        rightArmMotor.setPower(power);
        leftArmMotor.setPower(power);
    }

    // TODO: write set wrist/gripper position

    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset
     */
    public void setHandPositions(double offset) {
        offset = Range.clip(offset, -0.5, 0.5);
        leftHand.setPosition(MID_SERVO + offset);
        rightHand.setPosition(MID_SERVO - offset);
    }
}
