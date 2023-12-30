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
 *      - Make sure motors and servos can handle args outside of motion range (ideally, they just wouldn't move)
 *
 * TODO: Implement linear slide motor
 */

package org.firstinspires.ftc.teamcode.robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    // TeleOp Movements
    protected final int PIVOT_SHORT_MOVEMENT = 5;
    protected final int ARM_SHORT_MOVEMENT = 5;
    protected final double WRIST_SHORT_MOVEMENT = 0.05;
    protected final double GRIPPER_SHORT_MOVEMENT = 0.05;
    // Power
    protected final double MAX_DRIVE_POWER = 1;
    protected final double PIVOT_POWER = 1;
    protected final double ARM_POWER = 1;
    // Counts Per Revolution = CPR
    protected final int CPR_DRIVE_MOTOR = 100;
    protected final int CPR_PIVOT_MOTOR = 100;
    protected final int CPR_ARM_MOTOR = 100;
    // Positions (fractional & encoder counts)
    protected final int PIVOT_COLLECTION_POS_COUNT = 0;
    protected final int PIVOT_SECURE_POS_COUNT = 100;
    protected final int PIVOT_DEPOSITION_POS_COUNT = 200;
    protected final int ARM_COLLECTION_POS_COUNT = 0;
    protected final int ARM_SECURE_POS_COUNT = 100;
    protected final int ARM_DEPOSITION_POS_COUNT = 200;
    protected final double WRIST_COLLECTION_POS = 0.1;
    protected final double WRIST_SECURE_POS = 0.3;
    protected final double WRIST_DEPOSITION_POS = 0.5;
    protected final double GRIPPER_CLOSED_POS = 0.0;
    protected final double GRIPPER_OPEN_POS = 1.0;
    // Motors
    protected DcMotor rightDriveMotor = null;
    protected DcMotor leftDriveMotor = null;
    protected DcMotor pivotMotor = null; // To change angle of linear slide
    protected DcMotor armMotor = null; // To extend/retract linear slide

    // Servos
    protected Servo wristServo = null;
    protected Servo gripperServo = null;

    // For hardwareMap & telemetry
    protected OpMode opMode = null;

    //--------------------------------------------------------------------------
    // Constructors & Initialization
    //--------------------------------------------------------------------------

    public Robot(OpMode opmode) {
        opMode = opmode;
        init();
    }

    public void init() {
        // TODO: Update mapped names
        leftDriveMotor = opMode.hardwareMap.get(DcMotor.class, "left_drive");
        rightDriveMotor = opMode.hardwareMap.get(DcMotor.class, "right_drive");
        pivotMotor = opMode.hardwareMap.get(DcMotor.class, "pivot");
        armMotor = opMode.hardwareMap.get(DcMotor.class, "arm");

        leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        rightDriveMotor.setDirection(DcMotor.Direction.FORWARD);

        wristServo = opMode.hardwareMap.get(Servo.class, "wrist");
        gripperServo = opMode.hardwareMap.get(Servo.class, "gripper");

        opMode.telemetry.addData(">", "Hardware Initialized");
        opMode.telemetry.update();
    }

    //--------------------------------------------------------------------------
    // Robot Positions
    //--------------------------------------------------------------------------

    /**
     * In position to grab pixel
     */
    public void moveToCollectionPosition() {
        moveRobot(PIVOT_COLLECTION_POS_COUNT, ARM_COLLECTION_POS_COUNT, WRIST_COLLECTION_POS, GRIPPER_OPEN_POS);
    }

    /**
     * Secure position when moving (ideally to backboard)
     */
    public void moveToSecurePosition() {
        moveRobot(PIVOT_SECURE_POS_COUNT, ARM_SECURE_POS_COUNT, WRIST_SECURE_POS, GRIPPER_CLOSED_POS);
    }

    /**
     * In position to place pixel on backboard
     */
    public void moveToDepositionPosition() {
        moveRobot(PIVOT_DEPOSITION_POS_COUNT, ARM_DEPOSITION_POS_COUNT, WRIST_DEPOSITION_POS, GRIPPER_CLOSED_POS);
    }

    //--------------------------------------------------------------------------
    // Helper
    //--------------------------------------------------------------------------

    /**
     * Template for changing robot position
     *
     * @param pivotPosCount
     * @param armPosCount
     * @param wristPos
     * @param gripperPos
     */
    protected void moveRobot(int pivotPosCount, int armPosCount, double wristPos, double gripperPos) {
        moveMotor(pivotMotor, pivotPosCount, PIVOT_POWER);
        moveMotor(armMotor, armPosCount, ARM_POWER);
        wristServo.setPosition(wristPos);
        gripperServo.setPosition(gripperPos);
    }

    /**
     * Moves motor to desired position at desired speed
     *
     * @param motor
     * @param position
     * @param power
     */
    protected void moveMotor(DcMotor motor, int position, double power) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }
}
