package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooterv2;

@TeleOp
public class meca2 extends LinearOpMode {

    // DRIVE
    private DcMotor leftFront, rightFront, leftRear, rightRear;

    private Servo limit;

    // INTAKE
    private DcMotor leftInTake, rightInTake;

    private Servo leftLimit, rightLimit;

    // SHOOTER
    private Shooter shooterSubsystem;
    private Shooterv2 shooterSubsystem2;

    private DcMotorEx shooterMotor,shooterMotor2;

    // SENSOR
    private DistanceSensor sensorDistance;

    // POWER CAPS
    private static final double DRIVE_MAX_POWER = 1;
    private static final double INTAKE_IN_POWER  = -0.85;
    private static final double INTAKE_OUT_POWER =  1;

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

        // in take
        leftInTake = hardwareMap.get(DcMotor.class, "leftInTake");
        rightInTake = hardwareMap.get(DcMotor.class, "rightInTake");

        leftLimit = hardwareMap.get(Servo.class, "leftLimit");
        rightLimit = hardwareMap.get(Servo.class, "rightLimit");

        shooterSubsystem = new Shooter(hardwareMap);
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        shooterSubsystem2 = new Shooterv2(hardwareMap);
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        // DIRECTIONS
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftInTake.setDirection(DcMotorSimple.Direction.REVERSE);
        // reveresed
        //rightInTake.setDirection(DcMotorSimple.Direction);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

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

            // ---------------- SHOOTER ----------------
//            if (gamepad1.b) shooterSubsystem.close();
           // if (gamepad1.right_trigger > 0.05) shooterSubsystem.far();
//            if (gamepad1.x) shooterSubsystem.off();

            // shooterSubsystem.periodic();

            // ---------------- INTAKE LOGIC ----------------
            if (gamepad1.right_trigger > 0.05) {
                shooterSubsystem.far();
                shooterSubsystem2.far();
            }
//            shooterSubsystem.periodic();
//            shooterSubsystem2.periodic();

            if (gamepad1.x) {
                shooterSubsystem.off();
                shooterSubsystem2.off();
            }


            // A → toggle intake IN
            if (gamepad1.a && !lastA) {
                intakeToggledOn = true;
            }
            if (gamepad1.b) {
                shooterSubsystem.close();
                shooterSubsystem2.close();
            }
            shooterSubsystem.periodic();
            shooterSubsystem2.periodic();

            // Y → stop intake
            if (gamepad1.y && !lastY) {
                intakeToggledOn = false;
            }

            boolean holdOut = gamepad1.left_bumper; // spit out
            boolean holdIn  = gamepad1.right_bumper; // intake in (HOLD)

            // FINAL INTAKE DECISION (PRIORITY-BASED)
            if (holdOut) {
                leftInTake.setPower(INTAKE_OUT_POWER);
                rightInTake.setPower(INTAKE_OUT_POWER);
            } else if (holdIn) {
                leftInTake.setPower(INTAKE_IN_POWER);
                rightInTake.setPower(INTAKE_IN_POWER);
            } else if (intakeToggledOn) {
                leftInTake.setPower(INTAKE_IN_POWER);
                rightInTake.setPower(INTAKE_IN_POWER);
            } else {
                leftInTake.setPower(0);
                rightInTake.setPower(0);
            }

            // SERVO COMMMANDS -----------------

            if (gamepad2.dpad_up) {
                rightLimit.setPosition(0.5); // right side position
                leftLimit.setPosition(0.29); // left side position
            }

            // dpad down moves the servos down
            if (gamepad2.dpad_down){
                rightLimit.setPosition(0.1); //2
                leftLimit.setPosition(0.0); //1
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
            telemetry.addData("Intake Power", leftInTake.getPower());
            telemetry.addData("Intake Power", rightInTake.getPower());

            telemetry.addLine("SHOOTER");
            telemetry.addData("Target RPM", shooterSubsystem.getTarget());
            telemetry.addData("Power Shooter 1:", shooterMotor.getPower());
            telemetry.addData("Power Shooter 2:", shooterMotor2.getPower());


            telemetry.update();
        }
    }

    private double cap(double power, double max) {
        return Math.max(-max, Math.min(max, power));
    }
}
