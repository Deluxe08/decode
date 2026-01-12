package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystem.Turret;
import org.firstinspires.ftc.teamcode.config.vision.Limelight;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

@TeleOp(name = "mecaLimeLight")
public class mecaLimeLight extends LinearOpMode {

    // ---------------- DRIVE ----------------
    private DcMotor leftFront, rightFront, leftRear, rightRear;

    // ---------------- INTAKE ----------------
    private DcMotor inTake;

    // ---------------- SHOOTER ----------------
    private Shooter shooter;
    private DcMotorEx shooterMotor;

    // ---------------- TURRET ----------------
    private Turret turret;

    // ---------------- LIMELIGHT ----------------
    private Limelight limelight;

    // ---------------- CONSTANTS ----------------
    public static double DRIVE_MAX_POWER = 0.9;
    public static double INTAKE_IN_POWER = -0.85;
    public static double INTAKE_OUT_POWER = 0.85;

    // Intake toggle
    private boolean intakeOn = false;
    private boolean lastA = false;
    private boolean lastY = false;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        // -------- HARDWARE --------
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        inTake = hardwareMap.get(DcMotor.class, "inTake");

        shooter = new Shooter(hardwareMap);
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        turret = new Turret(hardwareMap);

        limelight = new Limelight(hardwareMap, Alliance.BLUE);
        limelight.start();

        // -------- DIRECTIONS --------
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        inTake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Dashboard Auto Turret Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ================= DRIVE =================
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            leftFront.setPower(cap((y + x + rx) / denom));
            leftRear.setPower(cap((y - x + rx) / denom));
            rightFront.setPower(cap((y - x - rx) / denom));
            rightRear.setPower(cap((y + x - rx) / denom));

            // ================= SHOOTER =================
            if (gamepad1.b) shooter.close();
            if (gamepad1.right_trigger > 0.05) shooter.far();
            if (gamepad1.left_bumper) shooter.off();

            shooter.periodic();

            // ================= INTAKE =================
            if (gamepad1.a && !lastA) intakeOn = true;
            if (gamepad1.y && !lastY) intakeOn = false;

            boolean intakeOutHold = gamepad1.x;

            if (intakeOutHold) {
                inTake.setPower(INTAKE_OUT_POWER);
            } else if (intakeOn) {
                inTake.setPower(INTAKE_IN_POWER);
            } else {
                inTake.setPower(0);
            }

            lastA = gamepad1.a;
            lastY = gamepad1.y;

            // ================= TURRET (AUTO ONLY) =================
            boolean autoAim = gamepad1.dpad_down;

            if (autoAim) {
                double angleDeg = limelight.angleFromShoot();
                double angleRad = Math.toRadians(angleDeg);

                turret.setYaw(angleRad);
                turret.automatic();
            }

            turret.periodic();

            // ================= DASHBOARD TELEMETRY (GRAPHABLE) =================
            double turretPos = turret.getTurret();
            double turretTarget = turret.getTurretTarget();
            double turretError = turretTarget - turretPos;

            telemetry.addData("Turret Pos (ticks)", turretPos);
            telemetry.addData("Turret Target (ticks)", turretTarget);
            telemetry.addData("Turret Error", turretError);
            telemetry.addData("Turret Power", Turret.power);

            telemetry.addData("Limelight Angle (deg)", limelight.angleFromShoot());
            telemetry.addData("Limelight Distance", limelight.distanceFromShoot());

            telemetry.addData("Shooter Target RPM", shooter.getTarget());
            telemetry.addData("Shooter Velocity RPM", shooter.getVelocity());
            telemetry.addData("Shooter Error", shooter.getTarget() - shooter.getVelocity());
            telemetry.addData("Shooter Power", shooterMotor.getPower());

            telemetry.addData("Intake Power", inTake.getPower());

            telemetry.update();
        }
    }

    // ---------------- POWER CAP ----------------
    private double cap(double power) {
        return Math.max(-DRIVE_MAX_POWER, Math.min(DRIVE_MAX_POWER, power));
    }
}
