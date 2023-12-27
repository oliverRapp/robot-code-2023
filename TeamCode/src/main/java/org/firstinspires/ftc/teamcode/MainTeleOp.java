/*
 * Handles interaction between driver and robot (Robot)
 *
 * Needs to handle fluid driving
 * Just intermediate or positions (handled in Robot)
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Main TeleOp (for comp 2)")
public class MainTeleOp extends OpMode {

    Robot robot = null;

    @Override
    public void init() {
        robot = new Robot(this);

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
        // driving (mostly handled by Robot)
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        robot.driveRobot(drive, turn);

        /*
        TODO:
        Deal with set positions
        "Attach listeners to call Robot button press handlers"
         */
    }
}
