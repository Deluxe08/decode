package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@Autonomous

public class auto2 extends LinearOpMode {
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;

    //Other motors
    private DcMotor inTake = null;

    private DcMotor insideInTake = null;

    private DcMotor shooter = null;

    private ElapsedTime period  = new ElapsedTime();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.5;
    static final double     FASTER_SPEED = 0.8;
    static final double     TURN_SPEED    = 0.5;
    static final double     BackWARD_SPEED = -1.0;

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class,  "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        inTake = hardwareMap.get(DcMotor.class, "inTake");
        insideInTake = hardwareMap.get(DcMotor.class, "insideInTake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        //WHEELS BRAKE
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();
        //move to high junction - forwards # ft ish
        leftFront.setPower(FORWARD_SPEED);
        rightFront.setPower(FORWARD_SPEED);
        leftRear.setPower(FORWARD_SPEED);
        leftFront.setPower(FORWARD_SPEED);
        //shooter.setPower(0.9);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
            telemetry.addData("Path", "command 1: %2.5f S ran", runtime.seconds());
            telemetry.update();
        }

        //PAUSE and put cone on junction
//        leftFront.setPower(0);
//        rightFront.setPower(0);
//        leftRear.setPower(0);
//        rightRear.setPower(0);
//        shooter.setPower(0.9);
//        insideInTake.setPower(-0.85);
//        inTake.setPower(0.5);
//
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 10)) {
//            telemetry.addData("Path", "command 2: %2.5f S ran", runtime.seconds());
//            telemetry.update();
//        }

//        //backwards 2 ft
//         leftFront.setPower(FASTER_SPEED);
//         rightFront.setPower(FORWARD_SPEED);
//         leftRear.setPower(FASTER_SPEED);
//         rightRear.setPower(FORWARD_SPEED);
//         runtime.reset();
//         while (opModeIsActive() && (runtime.seconds() < 1.0)) {
//             telemetry.addData("Path", "command 3: %2.5f S ran", runtime.seconds());
//             telemetry.update();
        //}

        //PAUSE
        // LeftFront.setPower(0);
        // RightFront.setPower(0);
        // LeftBack.setPower(0);
        // RightBack.setPower(0);
        // runtime.reset();
        // while (opModeIsActive() && (runtime.seconds() < 1.0)) {
        //     telemetry.addData("Path", "command PAUSE: %2.5f S ran", runtime.seconds());
        //     telemetry.update();
        // }

        //case 1 - left square - strafe to the left
        // LeftFront.setPower(-FORWARD_SPEED);
        // RightFront.setPower( FORWARD_SPEED);
        // LeftBack.setPower(FORWARD_SPEED);
        // RightBack.setPower(-FORWARD_SPEED);
        // runtime.reset();
        // while (opModeIsActive() && (runtime.seconds() < 1.0)) {
        //     telemetry.addData("Path", "command 4: %2.5f S ran", runtime.seconds());
        //     telemetry.update();
        // }

        //CASE 2 - PAUSE
        // LeftFront.setPower(0);
        // RightFront.setPower(0);
        // LeftBack.setPower(0);
        // RightBack.setPower(0);
        // runtime.reset();
        // while (opModeIsActive() && (runtime.seconds() < 1.0)) {
        //     telemetry.addData("Path", "command PAUSE: %2.5f S ran", runtime.seconds());
        //     telemetry.update();
        // }

        //case 3 - right square - strafe to the right
        // LeftFront.setPower(FORWARD_SPEED);
        // RightFront.setPower(-FORWARD_SPEED);
        // LeftBack.setPower(-FORWARD_SPEED);
        // RightBack.setPower(FORWARD_SPEED);
        // runtime.reset();
        // while (opModeIsActive() && (runtime.seconds() < 0.8)) {
        //     telemetry.addData("Path", "command 5: %2.5f S ran", runtime.seconds());
        //     telemetry.update();
        // }

        //Step 2:  Spin right for 1.3 seconds
        /*LeftFront.setPower(FORWARD_SPEED);
        RightFront.setPower(-FORWARD_SPEED);
        LeftBack.setPower(-FORWARD_SPEED);
        RightBack.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "command 3: %2.5f S ran", runtime.seconds());
            telemetry.update();
        }
        */
    }
}
