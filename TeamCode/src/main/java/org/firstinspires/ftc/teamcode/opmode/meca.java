package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// linear opmode gives us a single runOpMode loop
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// motor and hardware imports
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

// shooter subsystem wrapper (PID / velocity control)
import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;
// distance sensor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class meca extends LinearOpMode {

    // DRIVE
    // drive motors
    private DcMotor leftFront, rightFront, leftRear, rightRear;

    // SERVOS
    // limit servos used for trapping the balls in the intake before launching them
    private Servo leftLimit, rightLimit;

    // INTAKE
    // intake motor that pulls game pieces in or spits them out
    private DcMotor inTake;

    // SHOOTER
    // shooter subsystem
    private Shooter shooterSubsystem;
    // direct reference to shooter motor for telemetry
    private DcMotorEx shooterMotor;

    // SENSOR
    // distance sensor used to detect nearby objects
    private DistanceSensor sensorDistance;

    // POWER CAPS
    // maximum allowed drive power to prevent brownouts / motor strain
    private static final double DRIVE_MAX_POWER = 0.95;
    // intake power when pulling game pieces in
    private static final double INTAKE_IN_POWER  = -0.9;
    // intake power when pushing game pieces out
    private static final double INTAKE_OUT_POWER =  0.8;

    // INTAKE STATE
    // remembers whether intake is toggled on
    private boolean intakeToggledOn = false;

    // BUTTON EDGE MEMORY
    // used to detect button press events (not holding)
    private boolean lastA = false;
    private boolean lastY = false;

    @Override
    public void runOpMode() {

        // HARDWARE
        // map drive motors from the configuration
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        // map servos
        leftLimit = hardwareMap.get(Servo.class, "leftLimit");
        rightLimit = hardwareMap.get(Servo.class, "rightLimit");

        // map intake motor
        inTake = hardwareMap.get(DcMotor.class, "inTake");

        // initialize shooter subsystem
        shooterSubsystem = new Shooter(hardwareMap);
        // map shooter motor separately for telemetry
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        // map distance sensor
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        // DIRECTIONS
        // reverse left side motors so forward stick drives forward
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        // reverse intake so negative power intakes correctly
        inTake.setDirection(DcMotorSimple.Direction.REVERSE);

        // BRAKE mode so robot stops quickly when sticks are released
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // show initialization status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // wait for driver to press play
        waitForStart();

        while (opModeIsActive()) {

            // DRIVE INPUTS
            // forward/back from left stick Y (inverted)
            double y  = -gamepad1.left_stick_y;
            // strafe from left stick X
            double x  =  gamepad1.left_stick_x;
            // rotation from right stick X
            double rx =  gamepad1.right_stick_x;

            // normalization to keep motor powers within [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            // mecanum wheel power calculations
            double FL = (y + x + rx) / denominator;
            double BL = (y - x + rx) / denominator;
            double FR = (y - x - rx) / denominator;
            double BR = (y + x - rx) / denominator;

            // apply capped power to each motor
            leftFront.setPower(cap(FL, DRIVE_MAX_POWER));
            leftRear.setPower(cap(BL, DRIVE_MAX_POWER));
            rightFront.setPower(cap(FR, DRIVE_MAX_POWER));
            rightRear.setPower(cap(BR, DRIVE_MAX_POWER));

            // SHOOTER
            // close-range shot when B is pressed
            if (gamepad1.b) shooterSubsystem.close();
            // far-range shot when right trigger is pressed slightly or more
            if (gamepad1.right_trigger > 0.05) shooterSubsystem.far();
            // turn shooter off when X is pressed
            if (gamepad1.x) shooterSubsystem.off();

            // updates PID / velocity control every loop
            shooterSubsystem.periodic();

            // INTAKE LOGIC

            // A button toggles intake on
            if (gamepad1.a && !lastA) {
                intakeToggledOn = true;
            }

            // Y button turns intake off
            if (gamepad1.y && !lastY) {
                intakeToggledOn = false;
            }

            // left bumper forces intake to spit out
            boolean holdOut = gamepad1.left_bumper;
            // right bumper forces intake to pull in while held
            boolean holdIn  = gamepad1.right_bumper;

            // FINAL INTAKE DECISION
            // highest priority: spit out
            if (holdOut) {
                inTake.setPower(INTAKE_OUT_POWER);
                // next priority: hold-to-intake
            } else if (holdIn) {
                inTake.setPower(INTAKE_IN_POWER);
                // next: toggled intake state
            } else if (intakeToggledOn) {
                inTake.setPower(INTAKE_IN_POWER);
                // otherwise stop intake
            } else {
                inTake.setPower(0);
            }

            // SERVO COMMANDS
            // dpad up moves servos to shooting position
            if (gamepad2.dpad_up) {
                rightLimit.setPosition(0.5); // right side position
                leftLimit.setPosition(0.29); // left side position
            }

            // dpad down moves the servos down
            if (gamepad2.dpad_down){
                rightLimit.setPosition(0.1); //2
                leftLimit.setPosition(0.1); //1
            }

            // SAVE BUTTON STATES
            // store previous button values
            lastA = gamepad1.a;
            lastY = gamepad1.y;

            // SENSOR
            // read distance in centimeters
            double distanceCM = sensorDistance.getDistance(DistanceUnit.CM);
            // rumble controller if object is very close
            if (distanceCM < 10.0) {
                gamepad2.rumble(0.8, 0.8, 200);
            } else {
                gamepad2.stopRumble();
            }

            // TELEMETRY
            // intake status information
            telemetry.addLine("INTAKE");
            telemetry.addData("Hold In (B)", holdIn);
            telemetry.addData("Hold Out (X)", holdOut);
            telemetry.addData("Toggled In (A)", intakeToggledOn);

            // shooter telemetry values
            double shooterPower = shooterMotor.getPower();
            double shooterVel   = shooterSubsystem.getVelocity();
            double shooterTarget= shooterSubsystem.getTarget();
            double shooterError = shooterTarget - shooterVel;

            telemetry.addLine("SHOOTER INFO");
            telemetry.addData("Target RPM", shooterTarget);
            telemetry.addData("Actual RPM", shooterVel);
            telemetry.addData("Error", shooterError);
            telemetry.addData("Power", shooterPower);

            // drive motor powers
            telemetry.addLine("DRIVE POWER");
            telemetry.addData("BL", leftRear.getPower());
            telemetry.addData("FR", rightFront.getPower());
            telemetry.addData("BR", rightRear.getPower());
            telemetry.addData("FL", leftFront.getPower());

            // encoder target positions
            telemetry.addData("FL", leftFront.getTargetPosition());
            telemetry.addData("BL", leftRear.getTargetPosition());
            telemetry.addData("FR", rightFront.getTargetPosition());
            telemetry.addData("BR", rightRear.getTargetPosition());

            // distance sensor readout
            telemetry.addData("Distance (cm)", "%.02f", distanceCM);
            telemetry.addData("Status", "Running");

            telemetry.update();
        }
    }

    // caps motor power so it never exceeds the specified max
    private double cap(double power, double max) {
        return Math.max(-max, Math.min(max, power));
    }
}
