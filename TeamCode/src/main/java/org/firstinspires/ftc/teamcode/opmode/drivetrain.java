package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "drivtrain")
public class drivetrain extends LinearOpMode {

    DcMotorEx leftFront, rightFront, leftRear, rightRear;
    MultipleTelemetry multiTelemetry;

    public static double TICKS_PER_REV = 537.6;

    @Override
    public void runOpMode() {

        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear   = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

//        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        multiTelemetry = new MultipleTelemetry(
                FtcDashboard.getInstance().getTelemetry(),
                telemetry
        );

        telemetry.addLine("Drivetrain RPM Telemetry Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double fl = (y + x + rx) / denominator;
            double bl = (y - x + rx) / denominator;
            double fr = (y - x - rx) / denominator;
            double br = (y + x - rx) / denominator;

            leftFront.setPower(fl*0.5);
            leftRear.setPower(bl*0.5);
            rightFront.setPower(fr*0.5);
            rightRear.setPower(br*0.5);

            double lfVel = leftFront.getVelocity();
            double rfVel = rightFront.getVelocity();
            double lrVel = leftRear.getVelocity();
            double rrVel = rightRear.getVelocity();

            double lfRPM = (lfVel / TICKS_PER_REV) * 60.0;
            double rfRPM = (rfVel / TICKS_PER_REV) * 60.0;
            double lrRPM = (lrVel / TICKS_PER_REV) * 60.0;
            double rrRPM = (rrVel / TICKS_PER_REV) * 60.0;

            multiTelemetry.addLine("DRIVETRAIN RPM");
            multiTelemetry.addData("LF RPM", lfRPM);
            multiTelemetry.addData("RF RPM", rfRPM);
            multiTelemetry.addData("LR RPM", lrRPM);
            multiTelemetry.addData("RR RPM", rrRPM);

            multiTelemetry.addLine("VELOCITY (ticks/sec)");
            multiTelemetry.addData("LF Vel", lfVel);
            multiTelemetry.addData("RF Vel", rfVel);
            multiTelemetry.addData("LR Vel", lrVel);
            multiTelemetry.addData("RR Vel", rrVel);

            multiTelemetry.addLine("POWER");
            multiTelemetry.addData("LF Power", leftFront.getPower());
            multiTelemetry.addData("RF Power", rightFront.getPower());
            multiTelemetry.addData("LR Power", leftRear.getPower());
            multiTelemetry.addData("RR Power", rightRear.getPower());

            multiTelemetry.update();
        }
    }
}

