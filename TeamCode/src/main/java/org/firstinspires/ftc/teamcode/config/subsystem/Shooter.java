package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo   ;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Config
@Configurable
public class Shooter extends SubsystemBase {

    // private Servo f;
    private DcMotorEx shooter;
    private PIDFController boostPID, stablePID;

    private double targetRPM = 0;
    private boolean activated = true;

    // ================= PIDF TUNING VALUES =================
    public static double bp = 0.001;   // boost P
    public static double bd = 0.0005 ; // boost D
    public static double bf = 0.0; // boost F

    public static double sp = 0.09;   // stable P
    public static double sd = 0.00003; // stable D
    public static double sf = 0.00; // stable F

    public static double pSwitch = 150; // error threshold to switch PID

    // ================= PRESET RPM VALUES =================
    public static double closeRPM = 1200;
    public static double farRPM = 1400;

    // Servo positions (optional)
//    public static double flipUp = 0.3;
//    public static double flipDown = 0.71;

    public Shooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        boostPID = new PIDFController(new PIDFCoefficients(bp, 0, bd, bf));
        stablePID = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));
    }

    // ==================== GETTERS ====================
    public double getTarget() {
        return targetRPM;
    }

    public double getVelocity() {
        return shooter.getVelocity();
    }

    public boolean isActivated() {
        return activated;
    }

    // ==================== POWER CONTROL ====================
    public void setPower(double power) {
        // clamp power to safe range [-1, 1] Gobuilda motor heating up
        power = Math.max(-0.82, Math.min(0.82, power));
        shooter.setPower(power);
    }

    public void on() {
        activated = true;
    }

    public void off() {
        activated = false;
        setPower(0);
    }

    // ==================== PRESETS ====================
    public void close() {
        setTarget(closeRPM);
        on();
    }

    public void far() {
        setTarget(farRPM);
        on();
    }

    public void setTarget(double rpm) {
        targetRPM = rpm;
    }

    // ==================== PERIODIC PID LOOP ====================
    @Override
    public void periodic() {
        // Update PID coefficients in case they are changed on the dashboard
        boostPID.setCoefficients(new PIDFCoefficients(bp, 0, bd, bf));
        stablePID.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));

        if (!activated) return;

        double error = getTarget() - getVelocity();
        double power;

        if (Math.abs(error) < pSwitch) {
            // Use stable PID for fine control
            stablePID.updateError(error);
            power = stablePID.run();
        } else {
            // Use boost PID for quick approach
            boostPID.updateError(error);
            power = boostPID.run();
        }

        setPower(power);
    }

    // ==================== OPTIONAL SERVOS ====================
    public void up() {
        // f.setPosition(flipUp);
    }

    public void down() {
        // f.setPosition(flipDown);
    }

    /*
    public void flip() {
        if (f.getPosition() == flipDown)
            up();
        else
            down();
    }
    */

    // ==================== HELPER FUNCTIONS ====================
    public boolean atTarget() {
        return Math.abs(getTarget() - getVelocity()) < 30;
    }

    public void forDistance(double distance) {
        // Example quadratic formula for distance → RPM mapping
        setTarget((0.00180088 * Math.pow(distance, 2)) + (4.14265 * distance) + 948.97358);
    }

    /*
    public boolean atUp() {
        return f.getPosition() == flipUp;
    }
    */
}
