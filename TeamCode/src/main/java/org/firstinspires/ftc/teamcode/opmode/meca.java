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
import com.qualcomm.robotcore.robot.RobotState;
//import com.qualcomm.robotcore.external.navigation.CurrentUnit;

import java.util.List;

@TeleOp
public class meca extends LinearOpMode {

    // DRIVE MOTORS
    private DcMotor leftFront, rightFront, leftRear, rightRear;

    // INTAKE MOTORS
    private DcMotor /*inTake ,*/ insideInTake;

    // SHOOTER
    private Shooter shooterSubsystem;
    private DcMotorEx shooterMotor;

    // HUB
    private LynxModule hub;

    // POWER CAPS
    private static final double DRIVE_MAX_POWER = 0.6;
    private static final double INTAKE_MAX_POWER = 0.6;
    private static final double INNER_INTAKE_MAX_POWER = 0.5;

    public static double TICKS_PER_REV = 537.6;

    MultipleTelemetry multiTelemetry;


    @Override
    public void runOpMode() {

        // HARDWARE INIT
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

       // inTake = hardwareMap.get(DcMotor.class, "inTake");
        insideInTake = hardwareMap.get(DcMotor.class, "insideInTake");

        shooterSubsystem = new Shooter(hardwareMap);
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        // DRIVE DIRECTIONS
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        multiTelemetry = new MultipleTelemetry(
                FtcDashboard.getInstance().getTelemetry(),
                telemetry
        );

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

            if (gamepad1.options) {
                shooterSubsystem.close();
                insideInTake.setPower(cap(-0.5, INNER_INTAKE_MAX_POWER));
               // inTake.setPower(cap(0.0, INTAKE_MAX_POWER));
            } else if (gamepad1.right_trigger > 0.05) {
                shooterSubsystem.far();
                insideInTake.setPower(cap(-0.5, INNER_INTAKE_MAX_POWER));
                //inTake.setPower(cap(0.0, INTAKE_MAX_POWER));
            }
            // -------- SHOOTER CONTROL --------
            if (gamepad1.right_bumper) {
                shooterSubsystem.close();
                insideInTake.setPower(cap(-0.5, INNER_INTAKE_MAX_POWER));
               // inTake.setPower(cap(0.0, INTAKE_MAX_POWER));
            } else if (gamepad1.right_trigger > 0.05) {
                shooterSubsystem.far();
                insideInTake.setPower(cap(-0.5, INNER_INTAKE_MAX_POWER));
                //inTake.setPower(cap(0.0, INTnn   AKE_MAX_POWER));
            }

            if (gamepad1.left_bumper) {
                shooterSubsystem.off();
                insideInTake.setPower(0);
                //inTake.setPower(0);
            }

            // -------- INTAKE CONTROL --------
            if (gamepad1.y) {
                //inTake.setPower(cap(-0.6, INTAKE_MAX_POWER));
                insideInTake.setPower(cap(-0.5, INNER_INTAKE_MAX_POWER));
            } else if (gamepad1.a) {
                //inTake.setPower(0);
                insideInTake.setPower(0);
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

            // Velocity telemary
//            double rfRPM = (rfVel / TICKS_PER_REV) * 60.0;
//            double lrRPM = (lrVel / TICKS_PER_REV) * 60.0;
//            double rrRPM = (rrVel / TICKS_PER_REV) * 60.0;
//
//            multiTelemetry.addLine("DRIVETRAIN RPM");
//            multiTelemetry.addData("LF RPM", lfRPM);
//            multiTelemetry.addData("RF RPM", rfRPM);
//            multiTelemetry.addData("LR RPM", lrRPM);
//            multiTelemetry.addData("RR RPM", rrRPM);
//
//            multiTelemetry.addLine("VELOCITY (ticks/sec)");
//            multiTelemetry.addData("LF Vel", lfVel);
//            multiTelemetry.addData("RF Vel", rfVel);
//            multiTelemetry.addData("LR Vel", lrVel);
//            multiTelemetry.addData("RR Vel", rrVel);



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
            return 0;
        }

        // Ensure power is at least 0.0059
        if (absPower < 0.0059) {
            absPower = 0.0059;
        }

        // Ensure power does not exceed the max cap
        if (absPower > max) {
            absPower = max;
        }

        // Return the value with the original direction (positive or negative)
        return Math.copySign(absPower, power);
    }
}
