package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.config.Limelight.AprilTagDistance;

@TeleOp(name = "Meca AprilTag Servo", group = "Vision")
public class MecaAprilTagServo extends LinearOpMode {

    private DcMotor leftFront, rightFront, leftRear, rightRear;
    private Servo angleServo;
    private AprilTagDistance tagDistance;

    @Override
    public void runOpMode() {

        // Initialize motors
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        // Initialize servo
        angleServo = hardwareMap.get(Servo.class, "angleServo");

        // Initialize Limelight - Pipeline 0, AprilTag 24
        tagDistance = new AprilTagDistance(hardwareMap, 24);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Mecanum drive
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double fl = (y + x + rx) / denominator;
            double bl = (y - x + rx) / denominator;
            double fr = (y - x - rx) / denominator;
            double br = (y + x - rx) / denominator;

            leftFront.setPower(fl);
            leftRear.setPower(bl);
            rightFront.setPower(fr);
            rightRear.setPower(br);

            // Get distance from AprilTag
            double distance = tagDistance.getDistanceCM();

            // Set servo based on distance
            if (distance > 0) {
                if (distance <= 50) {
                    angleServo.setPosition(0.0);
                } else if (distance >= 150) {
                    angleServo.setPosition(0.2);
                } else {
                    // Interpolate between 50-150cm
                    double ratio = (distance - 50) / 100;
                    angleServo.setPosition(ratio * 0.2);
                }
            }

            // Telemetry
            telemetry.addData("Distance (cm)", distance);
            telemetry.addData("Tag Visible", tagDistance.isTargetVisible());
            telemetry.addData("Servo Position", angleServo.getPosition());
            telemetry.update();
        }
    }
}