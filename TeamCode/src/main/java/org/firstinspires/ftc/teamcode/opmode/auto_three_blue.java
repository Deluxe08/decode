package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooterv2;


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

public class auto_three_blue extends LinearOpMode {
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;

    private Shooter shooterSubsystem;

    private Shooterv2 shooterSubsystem2;

    // direct reference to shooter motor for telemetry
    private DcMotorEx shooterMotor;
    private DcMotorEx shooterMotor2;

    //Other motors
    private DcMotor leftInTake, rightInTake;

    private DcMotor insideInTake = null;

    private DcMotor shooter = null;
    private DcMotor shooter2 = null;

    private ElapsedTime period  = new ElapsedTime();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.5;
    static final double     FASTER_SPEED = 0.8;
    static final double     TURN_SPEED    = 0.4;
    static final double     BackWARD_SPEED = -0.5;

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class,  "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        rightInTake = hardwareMap.get(DcMotor.class, "rightInTake");
        leftInTake = hardwareMap.get(DcMotor.class, "leftInTake");
        //insideInTake = hardwareMap.get(DcMotor.class, "insideInTake");
        shooterSubsystem = new Shooter(hardwareMap);
        shooterSubsystem2 = new Shooterv2(hardwareMap);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
       // reverse
        leftInTake.setDirection(DcMotorSimple.Direction.REVERSE);

        //WHEELS BRAKE
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();
        //move tBACK
//        shooterSubsystem.close();
//        shooterSubsystem.periodic();
        leftFront.setPower(BackWARD_SPEED);
        rightFront.setPower(BackWARD_SPEED);
        leftRear.setPower(BackWARD_SPEED);
        rightRear.setPower(BackWARD_SPEED);
        //shooter.setPower(0.9);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.6)) {
            telemetry.addData("Path", "command 1: %2.5f S ran", runtime.seconds());
            telemetry.update();
        }

        //RAMP UP
        shooterSubsystem.close();
        shooterSubsystem2.close();
        shooterSubsystem.periodic();
        shooterSubsystem2.periodic();
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
            telemetry.addData("Path", "command 2: %2.5f S ran", runtime.seconds());
            telemetry.update();
        }

        //shoot 1
        shooterSubsystem.close();
        shooterSubsystem2.close();
        shooterSubsystem.periodic();
        shooterSubsystem2.periodic();
        leftInTake.setPower(-0.7);
        rightInTake.setPower(-0.7);
         runtime.reset();
         while (opModeIsActive() && (runtime.seconds() < 3)) {
             telemetry.addData("Path", "command 3: %2.5f S ran", runtime.seconds());
             telemetry.update();
        }

         //turn
        shooterSubsystem.off();
        //shooterSubsystem.periodic();
        leftFront.setPower(-TURN_SPEED);
        rightFront.setPower(TURN_SPEED);
        leftRear.setPower(-TURN_SPEED);
        rightRear.setPower(TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.3)) {
            telemetry.addData("Path", "command 3: %2.5f S ran", runtime.seconds());
            telemetry.update();
        }
        //
        leftFront.setPower(FORWARD_SPEED);
        rightFront.setPower(FORWARD_SPEED);
        leftRear.setPower(FORWARD_SPEED);
        rightRear.setPower(FORWARD_SPEED);
        leftInTake.setPower(-0.85);
        rightInTake.setPower(-0.85);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 5)) {
            telemetry.addData("Path", "command 2: %2.5f S ran", runtime.seconds());
            telemetry.update();
        }

        // go backwards after intaking the middle spike
        leftFront.setPower(BackWARD_SPEED);
        rightFront.setPower(BackWARD_SPEED);
        leftRear.setPower(BackWARD_SPEED);
        rightRear.setPower(BackWARD_SPEED);
        //shooter.setPower(0.9);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3)) {
            telemetry.addData("Path", "command 1: %2.5f S ran", runtime.seconds());
            telemetry.update();
        }
        //turn angle at goal

        leftFront.setPower(-TURN_SPEED);
        rightFront.setPower(TURN_SPEED);
        leftRear.setPower(TURN_SPEED);
        rightRear.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
            telemetry.addData("Path", "command 3: %2.5f S ran", runtime.seconds());
            telemetry.update();
        }
        //slide to the side
        leftFront.setPower(FORWARD_SPEED);
        rightFront.setPower(BackWARD_SPEED);
        leftRear.setPower(FORWARD_SPEED);
        rightRear.setPower(BackWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
            telemetry.addData("Path", "command 3: %2.5f S ran", runtime.seconds());
            telemetry.update();
        }

//        leftFront.setPower(FORWARD_SPEED);
//        rightFront.setPower(BackWARD_SPEED);
//        leftRear.setPower(FORWARD_SPEED);
//        rightRear.setPower(BackWARD_SPEED);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
//            telemetry.addData("Path", "command 3: %2.5f S ran", runtime.seconds());
//            telemetry.update();
//        }








//        shooterSubsystem.close();
//        shooterSubsystem.periodic();
//        inTake.setPower(0);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 2.5)) {
//            telemetry.addData("Path", "command 3: %2.5f S ran", runtime.seconds());
//            telemetry.update();
//        }// shoot 3
//        shooterSubsystem.close();
//        shooterSubsystem.periodic();
//        inTake.setPower(-0.9);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 5)) {
//            telemetry.addData("Path", "command 3: %2.5f S ran", runtime.seconds());
//            telemetry.update();
//        }// pause 3
//        shooterSubsystem.close();
//        shooterSubsystem.periodic();
//        inTake.setPower(0);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 2.5)) {
//            telemetry.addData("Path", "command 3: %2.5f S ran", runtime.seconds());
//            telemetry.update();
//        }
//        shooterSubsystem.off();
//        shooterSubsystem.periodic();
//        leftFront.setPower(0);
//        rightFront.setPower(0.4);
//        leftRear.setPower(0.4);
//        rightRear.setPower(0);
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < 2.5)) {
//            telemetry.addData("Path", "command 3: %2.5f S ran", runtime.seconds());
//            telemetry.update();
//        }

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
