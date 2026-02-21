package org.firstinspires.ftc.teamcode.config.Limelight;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class AprilTagDistanceServoController {

    private final Servo angleServo;

    private double currentPosition = 0.8;
    private final double increment = 0.1;

    private boolean lastB = false;
    private boolean lastA = false;

    private static final double MIN_POS = 0.2;
    private static final double MAX_POS = 1;

    public AprilTagDistanceServoController(Servo angleServo) {
        this.angleServo = angleServo;
        this.angleServo.setPosition(currentPosition);
    }

    public void update(Gamepad gamepad) {

        // Increase
        if (gamepad.b && !lastB) {
            currentPosition = Range.clip(currentPosition + increment, MIN_POS, MAX_POS);
            angleServo.setPosition(currentPosition);
        }

        // Decrease
        if (gamepad.a && !lastA) {
            currentPosition = Range.clip(currentPosition - increment, MIN_POS, MAX_POS);
            angleServo.setPosition(currentPosition);
        }

        lastB = gamepad.b;
        lastA = gamepad.a;
    }

    public double getPosition() {
        return currentPosition;
    }

    public void updateIncrement(Gamepad gamepad2) {
    }
}