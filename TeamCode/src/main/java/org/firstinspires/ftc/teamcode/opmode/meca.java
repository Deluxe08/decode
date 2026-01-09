package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import com.qualcomm.robotcore.robot.RobotState;
//import com.qualcomm.robotcore.external.navigation.CurrentUnit;

import java.util.List;

@TeleOp
public class meca extends LinearOpMode {

    // DRIVE MOTORS
    private DcMotor leftFront, rightFront, leftRear, rightRear;

    private DistanceSensor sensorDistance;


    // INTAKE MOTORS
    private DcMotor inTake ,insideInTake;

    // SHOOTER
    private Shooter shooterSubsystem;
    private DcMotorEx shooterMotor;

    // HUB
    private LynxModule hub;

    // POWER CAPS
    private static final double DRIVE_MAX_POWER = 0.6;
    private static final double INTAKE_MAX_POWER = -0.85;
    private static final double INNER_INTAKE_MAX_POWER = -0.5;

    public static double TICKS_PER_REV = 537.6;

    //MultipleTelemetry multiTelemetry;


    @Override
    public void runOpMode() {

        // HARDWARE INIT
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        inTake = hardwareMap.get(DcMotor.class, "inTake");
        insideInTake = hardwareMap.get(DcMotor.class, "insideInTake");

        shooterSubsystem = new Shooter(hardwareMap);
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        // DRIVE DIRECTIONS
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);


        inTake.setDirection(DcMotorSimple.Direction.REVERSE);
        //insideInTake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        multiTelemetry = new MultipleTelemetry(
//                FtcDashboard.getInstance().getTelemetry(),
//                telemetry
//        );

        telemetry.addLine("Drivetrain RPM Telemetry Ready");
        telemetry.update();

        // GET HUB
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hub = hubs.get(0); // assume single hub

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // -------- DRIVE CONTROL --------
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


            if (gamepad1.right_trigger > 0.05) {
                shooterSubsystem.far();
            }

            //intake power
            if (gamepad1.yWasPressed()) {
                inTake.setPower(-0.75); // set automatic
            }else if (gamepad1.aWasPressed()) {
                inTake.setPower(0);
            }

            // -------- SHOOTER CONTROL --------
            if (gamepad1.right_bumper) {
                shooterSubsystem.close();
               // insideInTake.setPower(cap(-0.5, INNER_INTAKE_MAX_POWER));
                //inTake.setPower(cap(-0.85, INTAKE_MAX_POWER));
            } else if (gamepad1.right_trigger > 0.05) {
                shooterSubsystem.far();
                //insideInTake.setPower(cap(-0.5, INNER_INTAKE_MAX_POWER));
                inTake.setPower(cap(-0.85, INTAKE_MAX_POWER));
            }

            double distanceCM = sensorDistance.getDistance(DistanceUnit.CM);
            // check if the distance is less than 2 cm
            // Note: The REV color-range sensor can saturate at ~5cm, returning 5cm for
            // anything closer. The REV 2m distance sensor is generally better
            // for short distances.
            //!Double.isNaN(distanceCM) &&
            if (distanceCM < 4.0) {
                gamepad2.rumble(1.0, 1.0, 200); // Max power rumble for 200ms
            } else {
                gamepad2.stopRumble();
            }

            if (gamepad1.left_bumper) {
                shooterSubsystem.off();
            }

            if (gamepad1.b) {
                inTake.setPower(cap(-7.5, INTAKE_MAX_POWER));

            } else {
                inTake.setPower(0);
            }

            // UPDATE SHOOTER LOOP
            shooterSubsystem.periodic();

            // -------- TELEMETRY --------
            double shooterPower = shooterMotor.getPower();
            double shooterVel   = shooterSubsystem.getVelocity();
            double shooterTarget= shooterSubsystem.getTarget();
            double shooterError = shooterTarget - shooterVel;

            telemetry.addLine("SHOOTER INFO");
            telemetry.addData("Target RPM", shooterTarget);
            telemetry.addData("Actual RPM", shooterVel);
            telemetry.addData("Error", shooterError);
            telemetry.addData("Power", shooterPower);

            telemetry.addLine("DRIVE POWER");
            telemetry.addData("BL", leftRear.getPower());
            telemetry.addData("FR", rightFront.getPower());
            telemetry.addData("BR", rightRear.getPower());
            telemetry.addData("FL", leftFront.getPower());


            telemetry.addData("FL", leftFront.getTargetPosition());
            telemetry.addData("BL", leftRear.getTargetPosition());
            telemetry.addData("FR", rightFront.getTargetPosition());
            telemetry.addData("BR", rightRear.getTargetPosition());

            telemetry.addData("Distance (cm)", "%.02f", distanceCM);
            telemetry.addData("Status", "Running");

            telemetry.addLine("INTAKE POWER");
            //  telemetry.addData("Intake", inTake.getPower());
            telemetry.addData("Inner Intake", insideInTake.getPower());

            telemetry.update();
        }
    }

    // power cap function
    private double cap(double power, double max) {
        double absPower = Math.abs(power);

        // If the stick is pushed very little, keep it at 0
        if (absPower < 0.01) {
            return 0.0001;
        }

        // Ensure power is at least 0.0059
        if (absPower < 0.0001) {
            absPower = 0.0001;
        }

        // Ensure power does not exceed the max cap
        if (absPower > max) {
            absPower = max;
        }

        // Return the value with the original direction (positive or negative)
        return Math.copySign(absPower, power);
    }
}
