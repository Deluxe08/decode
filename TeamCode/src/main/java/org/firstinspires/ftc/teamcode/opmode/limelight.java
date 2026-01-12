package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;
//import org.firstinspires.ftc.teamcode.config.subsystem.Turret;
import org.firstinspires.ftc.teamcode.config.vision.Limelight;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

@TeleOp(name = "limelight")
public class limelight extends LinearOpMode {


    // ---------------- TURRET ----------------
   // private Turret turret;

    // ---------------- LIMELIGHT ----------------
    private Limelight limelight;

    // ---------------- CONSTANTS ----------------


    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        // -------- HARDWARE --------

      //  turret = new Turret(hardwareMap);

        limelight = new Limelight(hardwareMap, Alliance.BLUE);
        limelight.start();

        telemetry.addLine("Dashboard Auto Turret Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean autoAim = gamepad1.dpad_down;

            if (autoAim) {
                double angleDeg = limelight.angleFromShoot();
                double angleRad = Math.toRadians(angleDeg);

               // turret.setYaw(angleRad);
                //turret.automatic();
            }

            //turret.periodic();

            // ================= DASHBOARD TELEMETRY (GRAPHABLE) =================
//            double turretPos = turret.getTurret();
//            double turretTarget = turret.getTurretTarget();
//            double turretError = turretTarget - turretPos;

//            telemetry.addData("Turret Pos (ticks)", turretPos);
//            telemetry.addData("Turret Target (ticks)", turretTarget);
//            telemetry.addData("Turret Error", turretError);
//            telemetry.addData("Turret Power", Turret.power);

            telemetry.addData("Limelight Angle (deg)", limelight.angleFromShoot());
            telemetry.addData("Limelight Distance", limelight.distanceFromShoot());

            telemetry.update();
        }
    }

}
