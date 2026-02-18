//package org.firstinspires.ftc.teamcode.config.subsystem;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.pedropathing.control.PIDFCoefficients;
//import com.pedropathing.control.PIDFController;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//@Config
//public class ShooterV3 {
//
//    // =============================
//    // MOTORS
//    // =============================
//    public DcMotorEx shooter;
//    public DcMotorEx shooter2;
//
//    // =============================
//    // TUNING CONSTANTS
//    // =============================
//
//    // PID
//    public static double kP = 0.0008;
//    public static double kI = 0.0000005;
//    public static double kD = 0.00005;
//
//    // Feedforward
//    public static double kF = 0.00022;
//    public static double kS = 0.03;   // Static friction compensation
//
//    // RPM Targets
//    public static double closeRPM = 1800;
//    public static double farRPM = 5000;
//
//    public static double closePowerClamp = 0.88;
//
//    // Ramp limiter (prevents spike)
//    private static final double MAX_POWER_STEP = 0.05;
//
//    private PIDFController pid;
//
//    private double targetRPM = 0;
//    private boolean activated = false;
//    private boolean isCloseShot = true;
//
//    private double lastPower = 0;
//
//    private HardwareMap hardwareMap;
//
//    // =============================
//    // CONSTRUCTOR
//    // =============================
//    public ShooterV3(HardwareMap hardwareMap) {
//        this.hardwareMap = hardwareMap;
//
//        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
//        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
//
//        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        pid = new PIDFController(new PIDFCoefficients(kP, kI, kD, kF));
//    }
//
//    // =============================
//    // CONTROL METHODS
//    // =============================
//
//    public void close() {
//        isCloseShot = true;
//        setTarget(closeRPM);
//    }
//
//    public void far() {
//        isCloseShot = false;
//        setTarget(farRPM);
//    }
//
//    public void setTarget(double rpm) {
//        targetRPM = rpm;
//        activated = true;
//    }
//
//    public void stop() {
//        activated = false;
//        lastPower = 0;
//        shooter.setPower(0);
//        shooter2.setPower(0);
//    }
//
//    public double getVelocity() {
//        return shooter2.getVelocity(); // ticks/sec
//    }
//
//    public boolean atTarget() {
//        return Math.abs(targetRPM - getVelocity()) < 40;
//    }
//
//    // =============================
//    // MAIN LOOP
//    // =============================
//    public void periodic() {
//
//        pid.setCoefficients(new PIDFCoefficients(kP, kI, kD, kF));
//
//        if (!activated) return;
//
//        // Battery voltage compensation
//        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
//        double compensatedF = kF * (12.0 / voltage);
//
//        // PID
//        double currentVel = getVelocity();
//        double error = targetRPM - currentVel;
//
//        pid.updateError(error);
//        double power = pid.run() + compensatedF;
//
//        // ----------------------------
//        // kS Static Compensation
//        // ----------------------------
//        if (targetRPM > 0) {
//            power += Math.signum(targetRPM) * kS;
//        }
//
//        // ----------------------------
//        // Ramp limiter
//        // ----------------------------
//        double delta = power - lastPower;
//        delta = Math.max(-MAX_POWER_STEP, Math.min(MAX_POWER_STEP, delta));
//        power = lastPower + delta;
//        lastPower = power;
//
//        // ----------------------------
//        // Clamp
//        // ----------------------------
//        if (isCloseShot) {
//            power = Math.max(-closePowerClamp,
//                    Math.min(closePowerClamp, power));
//        } else {
//            power = Math.max(-1.0, Math.min(1.0, power));
//        }
//
//        shooter.setPower(power);
//        shooter2.setPower(power);
//    }
//}
