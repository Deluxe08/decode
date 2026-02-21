package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooterv2;

@TeleOp
public class meca2 extends LinearOpMode {

    private DcMotor leftFront, rightFront, leftRear, rightRear;
    private DcMotor leftInTake, rightInTake;
    private Servo leftLimit, rightLimit, angleServo;
    private Shooter shooterSubsystem;
    private Shooterv2 shooterSubsystem2;
    private DcMotorEx shooterMotor, shooterMotor2;
    private DistanceSensor sensorDistance;

    private static final double DRIVE_MAX_POWER = 1;
    private static final double INTAKE_IN_POWER  = -0.85;
    private static final double INTAKE_OUT_POWER = 1;

    private boolean intakeToggledOn = false;
    private boolean lastA = false;
    private boolean lastY = false;
    private boolean lastX = false;
    private boolean lastB = false;

    private double servoPosition = 0.1; // starting position
    private static final double INCREMENT = 0.05;
    private static final double SERVO_MIN = 0.2;
    private static final double SERVO_MAX = 0.8;

    @Override
    public void runOpMode() {

        // Hardware
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        leftInTake = hardwareMap.get(DcMotor.class, "leftInTake");
        rightInTake = hardwareMap.get(DcMotor.class, "rightInTake");

        leftLimit = hardwareMap.get(Servo.class, "leftLimit");
        rightLimit = hardwareMap.get(Servo.class, "rightLimit");
        angleServo = hardwareMap.get(Servo.class, "angleServo");

        shooterSubsystem = new Shooter(hardwareMap);
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        shooterSubsystem2 = new Shooterv2(hardwareMap);
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        // Directions
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftInTake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Intake encoder setup
        leftInTake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftInTake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightInTake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightInTake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Drive braking
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Drive
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double FL = (y + x + rx) / denominator;
            double BL = (y - x + rx) / denominator;
            double FR = (y - x - rx) / denominator;
            double BR = (y + x - rx) / denominator;

            leftFront.setPower(cap(FL, DRIVE_MAX_POWER));
            leftRear.setPower(cap(BL, DRIVE_MAX_POWER));
            rightFront.setPower(cap(FR, DRIVE_MAX_POWER));
            rightRear.setPower(cap(BR, DRIVE_MAX_POWER));

            // Shooter
            if (gamepad1.right_trigger > 0.05) {
                shooterSubsystem.far();
                shooterSubsystem2.far();
            }
            if (gamepad1.b) {
                shooterSubsystem.close();
                shooterSubsystem2.close();
            }
            shooterSubsystem.periodic();
            shooterSubsystem2.periodic();

            if (gamepad1.x) {
                shooterSubsystem.off();
                shooterSubsystem2.off();
            }

            // Intake toggle
            if (gamepad1.a && !lastA) intakeToggledOn = true;
            if (gamepad1.y && !lastY) intakeToggledOn = false;

            boolean holdOut = gamepad1.left_bumper;
            boolean holdIn  = gamepad1.right_bumper;

            if (holdOut) {
                leftInTake.setPower(INTAKE_OUT_POWER);
                rightInTake.setPower(INTAKE_OUT_POWER);
            } else if (holdIn || intakeToggledOn) {
                leftInTake.setPower(INTAKE_IN_POWER);
                rightInTake.setPower(INTAKE_IN_POWER);
            } else {
                leftInTake.setPower(0);
                rightInTake.setPower(0);
            }

            // Limit servos
            if (gamepad2.dpad_up) rightLimit.setPosition(0.3);
            if (gamepad2.dpad_down) rightLimit.setPosition(0.1);

            // Direct button servo positions
            if (gamepad2.y) servoPosition = 0.2;
            if (gamepad2.a) servoPosition = 0.8;

            // Incremental control: X increases, B decreases
            if (gamepad2.x && !lastX) {
                servoPosition += INCREMENT;
            }
            if (gamepad2.b && !lastB) {
                servoPosition -= INCREMENT;
            }

            // Clamp to limits
            servoPosition = Range.clip(servoPosition, SERVO_MIN, SERVO_MAX);
            angleServo.setPosition(servoPosition);

            // Distance sensor feedback
            double distanceCM = sensorDistance.getDistance(DistanceUnit.CM);
            if (distanceCM < 10.0) gamepad2.rumble(0.8, 0.8, 200);
            else gamepad2.stopRumble();

            // Save last button states
            lastA = gamepad1.a;
            lastY = gamepad1.y;
            lastX = gamepad2.x;
            lastB = gamepad2.b;

            // Telemetry
            telemetry.addData("Servo Position", servoPosition);
            telemetry.addLine("INTAKE");
            telemetry.addData("Hold In (Bumper Right)", holdIn);
            telemetry.addData("Hold Out (Bumper Left)", holdOut);
            telemetry.addData("Toggled In (A)", intakeToggledOn);
            telemetry.addData("Intake Power Left", leftInTake.getPower());
            telemetry.addData("Intake Power Right", rightInTake.getPower());
            telemetry.addLine("SHOOTER");
            telemetry.addData("Target RPM", shooterSubsystem.getTarget());
            telemetry.addData("Power Shooter 1", shooterMotor.getPower());
            telemetry.addData("Power Shooter 2", shooterMotor2.getPower());
            telemetry.update();
        }
    }

    private double cap(double power, double max) {
        return Math.max(-max, Math.min(max, power));
    }
}