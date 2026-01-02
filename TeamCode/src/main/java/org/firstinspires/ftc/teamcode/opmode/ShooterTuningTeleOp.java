package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;

@TeleOp(name = "Shooter Test (Shooter + Intake)", group = "Test")
public class ShooterTuningTeleOp extends OpMode {

    private Shooter shooter;

    private DcMotor inTake;
    private DcMotor insideInTake;

    @Override
    public void init() {

        shooter = new Shooter(hardwareMap);

        inTake = hardwareMap.get(DcMotor.class, "inTake");
        insideInTake = hardwareMap.get(DcMotor.class, "insideInTake");

        telemetry.addLine("Shooter + Intake Test Ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ================= SHOOTER CONTROLS =================
        if (gamepad1.a) shooter.shootNear();
        if (gamepad1.b) shooter.shootFar();
        if (gamepad1.x) shooter.off();

        shooter.periodic();

        // ================= INTAKE LOGIC =================
        if (shooter.getTarget() > 0 && shooter.atTarget()) {
            // Shooter ON → feed rings
            inTake.setPower(-0.6);
            insideInTake.setPower(0.6);
        } else if (shooter.getTarget() > 0) {
            // Shooter spinning but not at speed yet
            inTake.setPower(-0.3);
            insideInTake.setPower(0.3);
        } else {
            // Shooter OFF
            inTake.setPower(0);
            insideInTake.setPower(0);
        }

        // ================= TELEMETRY =================
        telemetry.addData("Target RPM", shooter.getTarget());
        telemetry.addData("Velocity RPM", shooter.getVelocity());
        telemetry.addData("At Target", shooter.atTarget());
        telemetry.addData("Shooter Active", shooter.getTarget() > 0);
        telemetry.update();
    }
}
