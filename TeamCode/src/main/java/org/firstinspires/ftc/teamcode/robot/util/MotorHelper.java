package org.firstinspires.ftc.teamcode.robot.util;

import com.qualcomm.robotcore.hardware.DcMotor;

public final class MotorHelper {
   private MotorHelper() {}

    public static void prepareMotorEncoder(DcMotor motor, boolean useMotorPosition) {
        if (useMotorPosition) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public static void moveMotor(DcMotor motor, int position, double power) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }
}
