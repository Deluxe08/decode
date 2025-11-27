package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;

@TeleOp(name="Shooter Tuner v2", group="Tuning")
public class ShooterTuningTeleOp extends OpMode {

    private DcMotorEx shooterMotor;
    private DcMotorEx inTakeMotor;

    private DcMotorEx insideInTakeMotor;


    private PIDFController boostPID, stablePID;

    // TUNED PIDF coefficients for 2800 RPM target
    public static double bp = 0.04;
    public static double bd = 0.0;
    public static double bf = 0.0;

    // STABLE LOOP (hold RPM)
    public static double sp = 0.01;
    public static double sd = 0.1;
    public static double sf = 0.0;

    public static double pSwitch = 50;  // Switch from boost to stable PID

    private boolean activated = false;
    public static double targetRPM = 1400;

    TelemetryManager telemetryM;
    MultipleTelemetry multiTelemetry;

    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        inTakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "inTake");
        insideInTakeMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "insideInTake");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        boostPID = new PIDFController(new PIDFCoefficients(bp, 0, bd, bf));
        stablePID = new PIDFController(new PIDFCoefficients(sp, 0, sd, sf));

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

        // Toggle shooter ON/OFF
        if (gamepad1.a) activated = true;
        if (gamepad1.b) activated = false;

        // Adjust target RPM live
        if (gamepad1.dpad_up) targetRPM += 25;
        if (gamepad1.dpad_down) targetRPM -= 25;
        if (targetRPM < 0) targetRPM = 0;
        if (targetRPM > 5000) targetRPM = 5000;

        boostPID.setCoefficients(new PIDFCoefficients(bp, 0, bd, bf));
        stablePID.setCoefficients(new PIDFCoefficients(sp, 0, sd, sf));

        double velocity = shooterMotor.getVelocity();
        double error = targetRPM - velocity;
        double power = 0;

        if (activated) {
            if (Math.abs(error) > pSwitch) {
                boostPID.updateError(error);
                power = boostPID.run();
                insideInTakeMotor.setPower(0.85);
                inTakeMotor.setPower(-0.8);
            } else {
                stablePID.updateError(error);
                power = stablePID.run();
            }
            shooterMotor.setPower(power);
        } else {
            shooterMotor.setPower(0);
            insideInTakeMotor.setPower(0);
            inTakeMotor.setPower(0);
        }

        // Telemetry packet for graphs
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target RPM", targetRPM);
        packet.put("Velocity RPM", velocity);
        packet.put("Error", error);
        packet.put("Shooter Power", power);
        packet.put("Shooter On", activated);

        packet.put("v_graph", velocity);
        packet.put("t_graph", targetRPM);
        packet.put("error_graph", error);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetryM.addData("Shooter On", activated);
        telemetryM.addData("Target RPM", targetRPM);
        telemetryM.addData("Velocity RPM", velocity);
        telemetryM.addData("Error", error);
        telemetryM.addData("Power", power);
        telemetryM.update(multiTelemetry);
    }
}
