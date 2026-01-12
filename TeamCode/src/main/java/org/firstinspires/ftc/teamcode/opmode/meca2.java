package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class meca2 extends LinearOpMode {

    // DRIVE
    private DcMotor leftFront, rightFront, leftRear, rightRear;

    private Servo limit;

    // INTAKE
    private DcMotor inTake;

    // SHOOTER
    private Shooter shooterSubsystem;

    private DcMotorEx shooterMotor;

    // SENSOR
    private DistanceSensor sensorDistance;

    // POWER CAPS
    private static final double DRIVE_MAX_POWER = 0.95;
    private static final double INTAKE_IN_POWER  = -0.85;
    private static final double INTAKE_OUT_POWER =  0.85;

    // INTAKE STATE
    private boolean intakeToggledOn = false;

    // BUTTON EDGE MEMORY
    private boolean lastA = false;
    private boolean lastY = false;

    @Override
    public void runOpMode() {

        // HARDWARE
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        limit = hardwareMap.get(Servo.class, "limit");

        inTake = hardwareMap.get(DcMotor.class, "inTake");

        shooterSubsystem = new Shooter(hardwareMap);
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        // DIRECTIONS
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        inTake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double FL = (y + x + rx) / denominator;
            double BL = (y - x + rx) / denominator;
            double FR = (y - x - rx) / denominator;
            double BR = (y + x - rx) / denominator;

            leftFront.setPower(cap(FL, DRIVE_MAX_POWER));
            leftRear.setPower(cap(BL, DRIVE_MAX_POWER));
            rightFront.setPower(cap(FR, DRIVE_MAX_POWER));
            rightRear.setPower(cap(BR, DRIVE_MAX_POWER));

            // ---------------- SHOOTER ----------------
            if (gamepad1.b) shooterSubsystem.close();
            if (gamepad1.right_trigger > 0.05) shooterSubsystem.far();
            if (gamepad1.x) shooterSubsystem.off();

            shooterSubsystem.periodic();

            // ---------------- INTAKE LOGIC ----------------

            // A → toggle intake IN
            if (gamepad1.a && !lastA) {
                intakeToggledOn = true;
            }

            // Y → stop intake
            if (gamepad1.y && !lastY) {
                intakeToggledOn = false;
            }

            boolean holdOut = gamepad1.left_bumper; // spit out
            boolean holdIn  = gamepad1.right_bumper; // intake in (HOLD)

            // FINAL INTAKE DECISION (PRIORITY-BASED)
            if (holdOut) {
                inTake.setPower(INTAKE_OUT_POWER);
            } else if (holdIn) {
                inTake.setPower(INTAKE_IN_POWER);
            } else if (intakeToggledOn) {
                inTake.setPower(INTAKE_IN_POWER);
            } else {
                inTake.setPower(0);
            }
            // SERVO COMMMANDS -----------------

            if (gamepad1.dpad_up) {
                limit.setPosition(0.4);
            }

            if (gamepad1.dpad_up){
                limit.setPosition(0.1);
            }

            // SAVE BUTTON STATES
            lastA = gamepad1.a;
            lastY = gamepad1.y;

            // ---------------- SENSOR ----------------
            double distanceCM = sensorDistance.getDistance(DistanceUnit.CM);
            if (distanceCM < 10.0) {
                gamepad2.rumble(0.8, 0.8, 200);
            } else {
                gamepad2.stopRumble();
            }

            // ---------------- TELEMETRY ----------------
            telemetry.addLine("INTAKE");
            telemetry.addData("Hold In (B)", holdIn);
            telemetry.addData("Hold Out (X)", holdOut);
            telemetry.addData("Toggled In (A)", intakeToggledOn);
            telemetry.addData("Intake Power", inTake.getPower());

            telemetry.addLine("SHOOTER");
            telemetry.addData("Target RPM", shooterSubsystem.getTarget());
            telemetry.addData("Velocity RPM", shooterSubsystem.getVelocity());
            telemetry.addData("Power", shooterMotor.getPower());

            telemetry.update();
        }
    }

    private double cap(double power, double max) {
        return Math.max(-max, Math.min(max, power));
    }
}
