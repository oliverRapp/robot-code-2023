/*
 * Handles interaction between driver and robot (Robot)
 *
 * Needs to handle fluid driving
 * Just intermediate or positions (handled in Robot)
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.TeleOpRobot;

@TeleOp(name = "Main TeleOp (for comp 2)")
public class MainTeleOp extends OpMode {
    private final double GAMEPAD_DRIVE = gamepad1.left_stick_y;
    private final double GAMEPAD_TURN = gamepad1.right_stick_x;

    private final boolean GAMEPAD_COLLECTION = gamepad1.a;
    private final boolean GAMEPAD_SECURE = gamepad1.x;
    private final boolean GAMEPAD_DEPOSITION = gamepad1.y;

    private final boolean GAMEPAD_RAISE_PIVOT = gamepad1.right_bumper;
    private final boolean GAMEPAD_LOWER_PIVOT = gamepad1.left_bumper;
    private final float GAMEPAD_EXTEND_ARM = gamepad1.right_trigger;
    private final float GAMEPAD_RETRACT_ARM = gamepad1.left_trigger;
    private final boolean GAMEPAD_RAISE_WRIST = gamepad1.dpad_up;
    private final boolean GAMEPAD_LOWER_WRIST = gamepad1.dpad_down;
    private final boolean GAMEPAD_OPEN_GRIPPER = gamepad1.dpad_right;
    private final boolean GAMEPAD_CLOSE_GRIPPER = gamepad1.dpad_left;

    TeleOpRobot robot = null;

    @Override
    public void init() {
        robot = new TeleOpRobot(this);

        telemetry.addData(">", "Initialized");
        telemetry.addData(">", "Waiting for start..");
        telemetry.update();
    }

    @Override
    public void start() {
        telemetry.addData(">", "Started");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Driving (mostly handled by Robot)
        robot.driveRobot(-GAMEPAD_DRIVE, GAMEPAD_TURN);

        // Hard-coded positions
        if (GAMEPAD_COLLECTION) {
            robot.moveToCollectionPosition();
        } else if (GAMEPAD_SECURE) {
            robot.moveToSecurePosition();
        } else if (GAMEPAD_DEPOSITION) {
            robot.moveToDepositionPosition();
        }

        // Small adjustments
        if (GAMEPAD_RAISE_PIVOT) {
            robot.raisePivotShort();
        }
        if (GAMEPAD_LOWER_PIVOT) {
            robot.lowerPivotShort();
        }
        if (GAMEPAD_EXTEND_ARM > 0) {
            robot.extendArmShort();
        }
        if (GAMEPAD_RETRACT_ARM > 0) {
            robot.retractArmShort();
        }
        if (GAMEPAD_RAISE_WRIST) {
            robot.raiseWristShort();
        }
        if (GAMEPAD_LOWER_WRIST) {
            robot.lowerWristShort();
        }
        if (GAMEPAD_OPEN_GRIPPER) {
            robot.openGripperShort();
        }
        if (GAMEPAD_CLOSE_GRIPPER) {
            robot.closeGripperShort();
        }

        robot.updateRobotPositions();
    }
}
