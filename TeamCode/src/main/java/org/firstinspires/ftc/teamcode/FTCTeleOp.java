package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "Main TeleOp (Gamepad Test)")
public class FTCTeleOp extends OpMode {
    private double GAMEPAD_DRIVE;
    private double GAMEPAD_TURN;

    private boolean GAMEPAD_RAISE_PIVOT;
    private boolean GAMEPAD_LOWER_PIVOT;

    private float GAMEPAD_EXTEND_SLIDE;
    private float GAMEPAD_RETRACT_SLIDE;

    private boolean GAMEPAD_RAISE_WRIST;
    private boolean GAMEPAD_LOWER_WRIST;

    private boolean GAMEPAD_OPEN_GRIPPER;
    private boolean GAMEPAD_CLOSE_GRIPPER;

    // Add drone button
    private boolean GAMEPAD_RELEASE_DRONE;

    private boolean GAMEPAD_COLLECTION;
    private boolean GAMEPAD_SECURE;
    private boolean GAMEPAD_DEPOSITION;

    Robot robot = null;

    @Override
    public void init() {
        robot = new Robot(this.hardwareMap);

        GAMEPAD_DRIVE = gamepad1.left_stick_y;
        GAMEPAD_TURN = gamepad1.right_stick_x;

        GAMEPAD_RAISE_PIVOT = gamepad1.right_bumper;
        GAMEPAD_LOWER_PIVOT = gamepad1.left_bumper;

        GAMEPAD_EXTEND_SLIDE = gamepad1.right_trigger;
        GAMEPAD_RETRACT_SLIDE = gamepad1.left_trigger;

        GAMEPAD_RAISE_WRIST = gamepad1.dpad_up;
        GAMEPAD_LOWER_WRIST = gamepad1.dpad_down;

        GAMEPAD_OPEN_GRIPPER = gamepad1.dpad_right;
        GAMEPAD_CLOSE_GRIPPER = gamepad1.dpad_left;

        // Add drone button
        GAMEPAD_RELEASE_DRONE = true;

        GAMEPAD_COLLECTION = gamepad1.a;
        GAMEPAD_SECURE = gamepad1.x;
        GAMEPAD_DEPOSITION = gamepad1.y;

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
//        if (GAMEPAD_RAISE_WRIST) {
//            robot.wrist.raiseShort();
//        }
//        if (GAMEPAD_LOWER_WRIST) {
//            robot.wrist.lowerShort();
//        }
//        if (GAMEPAD_OPEN_GRIPPER) {
//            robot.gripper.openShort();
//        }
//        if (GAMEPAD_CLOSE_GRIPPER) {
//            robot.gripper.closeShort();
//        }
        if (GAMEPAD_RELEASE_DRONE) {
            robot.drone.release();
        }

        robot.updatePositions();
    }
}
