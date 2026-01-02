package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.commands.Instant;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Shooter {

    private DcMotorEx shooter;

    private double t = 0; // target RPM
    private double previousError = 0; // for derivative term

    // PIDF TUNING
    public static double kS = 0.05;       // static feedforward
    public static double kV = 0.0015;     // velocity feedforward
    public static double kP = 0.001;      // proportional
    public static double kD = 0.0002;     // derivative (optional)
    public static double MAX_POWER = 0.5; // cap

    private boolean activated = true;

    public static double near = 1500; // RPM for close shot
    public static double far = 1700;  // RPM for far shot

    // CONSTRUCTOR
    public Shooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
    }


    public double getTarget() {
        return t;
    }

    public double getVelocity() {
        return shooter.getVelocity();
    }

    // POWER CONTROL
    public void setPower(double p) {
        double cappedPower = Math.max(-MAX_POWER, Math.min(MAX_POWER, p));
        shooter.setPower(-cappedPower); // negative if motor is reversed
    }

    public void off() {
        activated = false;
        setPower(0);
    }

    public void on() {
        activated = true;
    }

    public void shooterToggle() {
        activated = !activated;
        if (!activated) setPower(0);
    }

    public Instant toggle() {
        return new Instant(this::shooterToggle);
    }

    public void shootFar() {
        setTarget(far);
        on();
    }

    public void shootNear() {
        setTarget(near);
        on();
    }

    public Instant near() {
        return new Instant(this::shootNear);
    }

    public Instant far() {
        return new Instant(this::shootFar);
    }

    public void setTarget(double velocity) {
        t = velocity;
    }

    // MAIN LOOP CONTROL
    public void periodic() {
        if (activated) {
            double error = getTarget() - getVelocity();
            double derivative = error - previousError;
            previousError = error;

            double power = (kV * getTarget()) + (kP * error) + (kD * derivative) + kS;
            setPower(power);
        }
    }

    public boolean atTarget() {
        return Math.abs(getTarget() - getVelocity()) < 50;
    }

    public void forDistance(double distance, boolean close) {
        if (close) {
            setTarget((0.00180088 * Math.pow(distance, 2)) + (4.14265 * distance) + 948.97358);
        } else {
            setTarget(1500);
        }
    }
}
