/*
 * Handles interaction between driver and robot (Robot)
 *
 * Needs to handle fluid driving
 * Just intermediate or positions (handled in Robot)
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.Robot;

@TeleOp(name = "Main TeleOp (for comp 2)")
public class MainTeleOp extends OpMode {

    private final boolean GAMEPAD_COLLECTION = true;
    private final boolean GAMEPAD_SECURE = true;
    private final boolean GAMEPAD_DEPOSITION = true;

    Robot robot = null;

    @Override
    public void init() {
        robot = new Robot(this);

        robot.prepareMotorsTeleOp();

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

        // TODO: fluid movement of other parts
    }
}
