package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Drone {
    private final double DRONE_CLOSE_POS = 0;
    private final double DRONE_RELEASE_POS = 1;

    private Servo droneServo;

    public Drone(HardwareMap hwMap) {
        droneServo = hwMap.get(Servo.class, "drone");
    }

    public double getPos() {
        return droneServo.getPosition();
    }

    public void close() {
        droneServo.setPosition(DRONE_CLOSE_POS);
    }

    public void release() {
        droneServo.setPosition(DRONE_RELEASE_POS);
    }
}
