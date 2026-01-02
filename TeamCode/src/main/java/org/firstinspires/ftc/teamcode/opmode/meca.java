package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

@TeleOp
public class meca extends LinearOpMode {

    // MOTORS
    private DcMotor leftFront, rightFront, leftRear, rightRear;
    private DcMotor inTake, insideInTake;

    private Shooter shooterSubsystem;

    // POWER CAPS
    public static double DRIVE_MAX_POWER = 0.6;
    public static double INTAKE_MAX_POWER = 0.6;
    public static double INNER_INTAKE_MAX_POWER = 0.5;

    // HUB
    private LynxModule hub;

    @Override
    public void runOpMode() {

        // Drivetrain
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        // Intake
        inTake = hardwareMap.get(DcMotor.class, "inTake");
        insideInTake = hardwareMap.get(DcMotor.class, "insideInTake");

        //Shooter
        shooterSubsystem = new Shooter(hardwareMap);

        // Directions
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Hub reference
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hub = hubs.get(0); // assume single hub for current telemetry

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // DRIVE CONTROL
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

            //SHOOTER
            if (gamepad1.right_bumper) {
                shooterSubsystem.shootNear();
                insideInTake.setPower(cap(0.8, INNER_INTAKE_MAX_POWER));
            }

            if (gamepad1.right_trigger > 0.05) {
                shooterSubsystem.shootFar();
                insideInTake.setPower(cap(0.8, INNER_INTAKE_MAX_POWER));
            }

            if (gamepad1.left_bumper) {
                shooterSubsystem.off();
                inTake.setPower(0);
                insideInTake.setPower(0);
            }

            // INTAKE
            if (gamepad1.y) {
                inTake.setPower(cap(-0.6, INTAKE_MAX_POWER));
                insideInTake.setPower(cap(0.5, INNER_INTAKE_MAX_POWER));
            }

            if (gamepad1.a) {
                inTake.setPower(0);
                insideInTake.setPower(0);
            }

            //UPDATE SHOOTER
            shooterSubsystem.periodic();

            //TELEMETRY
            telemetry.addData("Shooter Target", shooterSubsystem.getTarget());
            telemetry.addData("Shooter Velocity", shooterSubsystem.getVelocity());

            telemetry.addData("FL Power", leftFront.getPower());
            telemetry.addData("FL Power", leftFront.getPower());
            telemetry.addData("FR Power", rightFront.getPower());
            telemetry.addData("BL Power", leftRear.getPower());
            telemetry.addData("BR Power", rightRear.getPower());

            telemetry.addData("Intake Power", inTake.getPower());
            telemetry.addData("Inner Intake Power", insideInTake.getPower());

            telemetry.addData("Hub Current (A)", String.format("%.2f", hub.getCurrent(CurrentUnit.AMPS)));

            telemetry.update();
        }
    }

    // POWER CAP
    private double cap(double power, double max) {
        return Math.max(-max, Math.min(max, power));
    }
}
