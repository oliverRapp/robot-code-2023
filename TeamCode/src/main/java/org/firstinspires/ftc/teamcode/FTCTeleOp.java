package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "Main TeleOp")
public class FTCTeleOp extends OpMode {

    private Robot robot = null;

    @Override
    public void init() {
        robot = new Robot(this.hardwareMap);

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
        double GAMEPAD_DRIVE = gamepad1.left_stick_y;
        double GAMEPAD_TURN = gamepad1.right_stick_x;

        boolean GAMEPAD_RAISE_PIVOT = gamepad1.right_bumper;
        boolean GAMEPAD_LOWER_PIVOT = gamepad1.left_bumper;

        float GAMEPAD_EXTEND_SLIDE = gamepad1.right_trigger;
        float GAMEPAD_RETRACT_SLIDE = gamepad1.left_trigger;

        boolean GAMEPAD_RAISE_WRIST = gamepad1.dpad_up;
        boolean GAMEPAD_LOWER_WRIST = gamepad1.dpad_down;

        boolean GAMEPAD_OPEN_GRIPPER = gamepad1.dpad_right;
        boolean GAMEPAD_CLOSE_GRIPPER = gamepad1.dpad_left;

        // Add drone button
        // Add drone button
        boolean GAMEPAD_RELEASE_DRONE = gamepad1.b;

        boolean GAMEPAD_COLLECTION = gamepad1.a;
        boolean GAMEPAD_SECURE = gamepad1.x;
        boolean GAMEPAD_DEPOSITION = gamepad1.y;


        // Driving
        double driveSq = GAMEPAD_DRIVE * GAMEPAD_DRIVE;
        double turnSq = GAMEPAD_TURN * GAMEPAD_TURN;
        robot.drivetrain.driveFluid(-driveSq * Math.signum(GAMEPAD_DRIVE), -turnSq * Math.signum(GAMEPAD_TURN));

        // Testing logging
        telemetry.addData(">", "slide: " + robot.slide.getPos());
        telemetry.addData(">", "pivot: " + robot.pivot.getPos());
        telemetry.addData(">", "drone: " + robot.drone.getPos());
        telemetry.addData(">", "gripper: " + robot.gripper.getPos());
        telemetry.addData(">", "wrist: " + robot.wrist.getPos());

        telemetry.update();

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
            robot.gripper.open();
        }
        if (GAMEPAD_CLOSE_GRIPPER) {
            robot.gripper.close();
        }
        if (GAMEPAD_RELEASE_DRONE) {
            telemetry.addData(">","Drone released");
            telemetry.update();
            robot.drone.release();
        }

        robot.updatePositions();
    }
}
