package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "Main TeleOp (for comp 2)")
public class FTCTeleOp extends OpMode {
    private double GAMEPAD_DRIVE;
    private final double GAMEPAD_TURN = gamepad1.right_stick_x;

    private final boolean GAMEPAD_RAISE_PIVOT = gamepad1.right_bumper;
    private final boolean GAMEPAD_LOWER_PIVOT = gamepad1.left_bumper;

    private final float GAMEPAD_EXTEND_SLIDE = gamepad1.right_trigger;
    private final float GAMEPAD_RETRACT_SLIDE = gamepad1.left_trigger;

    private final boolean GAMEPAD_RAISE_WRIST = gamepad1.dpad_up;
    private final boolean GAMEPAD_LOWER_WRIST = gamepad1.dpad_down;

    private final boolean GAMEPAD_OPEN_GRIPPER = gamepad1.dpad_right;
    private final boolean GAMEPAD_CLOSE_GRIPPER = gamepad1.dpad_left;

    // Add drone button
    private final boolean GAMEPAD_RELEASE_DRONE = true;

    private final boolean GAMEPAD_COLLECTION = gamepad1.a;
    private final boolean GAMEPAD_SECURE = gamepad1.x;
    private final boolean GAMEPAD_DEPOSITION = gamepad1.y;

    Robot robot = null;

    @Override
    public void init() {
        robot = new Robot(this.hardwareMap);

        GAMEPAD_DRIVE = gamepad1.left_stick_y;

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
        // Driving
        robot.drivetrain.driveFluid(-GAMEPAD_DRIVE, GAMEPAD_TURN);

        // Hard-coded positions
        if (GAMEPAD_COLLECTION) {
            robot.moveToPosition(Robot.Positions.COLLECT);
        } else if (GAMEPAD_SECURE) {
            robot.moveToPosition(Robot.Positions.SECURE);
        } else if (GAMEPAD_DEPOSITION) {
            robot.moveToPosition(Robot.Positions.DEPOSIT);
        }

        // Small adjustments
        if (GAMEPAD_RAISE_PIVOT) {
            robot.pivot.raiseShort();
        }
        if (GAMEPAD_LOWER_PIVOT) {
            robot.pivot.lowerShort();
        }
        if (GAMEPAD_EXTEND_SLIDE > 0) {
            robot.slide.extendShort();
        }
        if (GAMEPAD_RETRACT_SLIDE > 0) {
            robot.slide.retractShort();
        }
        if (GAMEPAD_RAISE_WRIST) {
            robot.wrist.raiseShort();
        }
        if (GAMEPAD_LOWER_WRIST) {
            robot.wrist.lowerShort();
        }
        if (GAMEPAD_OPEN_GRIPPER) {
            robot.gripper.openShort();
        }
        if (GAMEPAD_CLOSE_GRIPPER) {
            robot.gripper.closeShort();
        }
        if (GAMEPAD_RELEASE_DRONE) {
            robot.drone.release();
        }

        robot.updatePositions();
    }
}
