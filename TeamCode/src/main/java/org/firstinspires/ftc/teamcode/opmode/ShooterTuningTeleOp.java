package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;

@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTuningTeleOp extends OpMode {

    private Shooter shooter;

    private DcMotor inTake;
    private DcMotor insideInTake;

    @Override
    public void init() {

        shooter = new Shooter(hardwareMap);

        inTake = hardwareMap.get(DcMotor.class, "inTake");
        inTake.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Shooter + Intake Test Ready");
        telemetry.update();
    }
// Test if shooter is working properly for it's RPM values
    @Override
    public void loop() {

        //  SHOOTER CONTROLS
        if (gamepad1.a) shooter.close();
        if (gamepad1.b) shooter.far();
        if (gamepad1.x) shooter.off();

        shooter.periodic();
        inTake.setPower(-0.6);

        // TELEMETRY
        telemetry.addData("Target RPM", shooter.getTarget());
        telemetry.addData("Velocity RPM", shooter.getVelocity());
        telemetry.addData("Shooter Active", shooter.getTarget() > 0);
        telemetry.update();
    }
}
