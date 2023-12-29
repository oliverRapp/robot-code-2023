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
    private final boolean GAMEPAD_COLLECTION = true;
    private final boolean GAMEPAD_SECURE = true;
    private final boolean GAMEPAD_DEPOSITION = true;

    private final boolean GAMEPAD_RAISE_PIVOT = true;
    private final boolean GAMEPAD_LOWER_PIVOT = true;
    private final boolean GAMEPAD_EXTEND_ARM = true;
    private final boolean GAMEPAD_RETRACT_ARM = true;
    private final boolean GAMEPAD_RAISE_WRIST = true;
    private final boolean GAMEPAD_LOWER_WRIST = true;
    private final boolean GAMEPAD_OPEN_GRIPPER = true;
    private final boolean GAMEPAD_CLOSE_GRIPPER = true;

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
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        robot.driveRobot(drive, turn);

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
        if (GAMEPAD_EXTEND_ARM) {
            robot.extendArmShort();
        }
        if (GAMEPAD_RETRACT_ARM) {
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
