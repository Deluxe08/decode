package org.firstinspires.ftc.teamcode.opmode;
//package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp

public class meca extends LinearOpMode {
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;

    //Other motors
    private DcMotor inTake = null;

    private DcMotor insideInTake = null;

    private DcMotor shooter = null;


    //private DcMotor sweeperMotor = null;
    //private DcMotor extendMotor = null;

    @Override
    public void runOpMode() {

        //declare variables for drive train motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class,  "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        inTake = hardwareMap.get(DcMotor.class, "inTake");
        insideInTake = hardwareMap.get(DcMotor.class, "insideInTake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        //WHEELS BRAKE
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //When code is initialized on the phone, telemetry will print
        telemetry.addData("Status", "Initialized, hi driver");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //When code is active on the phone, telemetry will print
            telemetry.addData("Status", "Running");
            telemetry.update();

            // Set controller input
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x; // this is strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double FL_power = (y + x + rx) / denominator;
            double FR_power = (y - x - rx) / denominator;
            double BL_power = (y - x + rx) / denominator;
            double BR_power = (y + x - rx) / denominator;

            // Throttle speed
            if(gamepad1.right_trigger > 0) {
                FR_power *= Math.min(1, 1.5 - gamepad1.right_trigger);
                BR_power *= Math.min(1, 1.5 - gamepad1.right_trigger);
                FL_power *= Math.min(1, 1.5 - gamepad1.right_trigger);
                BL_power *= Math.min(1, 1.5 - gamepad1.right_trigger);
            }

            // Set motor powers
            // changeing the percenatge of how fast we go
            rightFront.setPower(FR_power*0.7);
            rightRear.setPower(BR_power*0.7);
            leftFront.setPower(FL_power*0.7);
            leftRear.setPower(BL_power*0.7);

            if (gamepad1.right_bumper) { //SHOOTER
                shooter.setPower(0.85);
                insideInTake.setPower(0.75);
                inTake.setPower(0);

            } if (gamepad1.left_bumper) { // OFF SHOOTER
                shooter.setPower(0.01);
                insideInTake.setPower(0.1);
            } if (gamepad1.a) { // OFF INTAKE + OFF ROLLER
                inTake.setPower(0.0);
                insideInTake.setPower(0.0);
            } if (gamepad1.y) { // ON BUTTON FOR PCB INTAKE + SLOW ROLLER
                inTake.setPower(-0.8);
                insideInTake.setPower(0.2);
            }
            if (gamepad1.b) { // HOLD BUTTON FOR INNER ROLLER
                insideInTake.setPower(0.6);
            } else {
                insideInTake.setPower(0.01);
            }
        }
    }
}