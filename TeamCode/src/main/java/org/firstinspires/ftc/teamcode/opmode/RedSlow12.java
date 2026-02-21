package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
//import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.config.subsystem.Shooterv2;

@Autonomous(name = "Red Slow 12", group = "Red")
public class RedSlow12 extends LinearOpMode {

    private Follower follower;
    //private Robot robot;
    private Shooter shooter;
    private Shooterv2 shooter2;

    private Pose start = new Pose(24 + 6.25, 120 + 8 + 6, Math.toRadians(90));
    private Pose scorePControl = new Pose(55.593, 94.779);
    private Pose score = new Pose(48, 96.0, Math.toRadians(135));
    private Pose intake1 = new Pose(17, 88 - 1.5, Math.toRadians(180));
    private Pose intake1Mid = intake1.withX(48);
    private Pose intake2 = new Pose(10, 62 - 1, Math.toRadians(180));
    private Pose intake2Mid = intake2.withX(48);
    private Pose intake2Return = intake2.withX(40);
    private Pose intake3 = new Pose(10, 39.750 + 1.5 - 3 - 1, Math.toRadians(180));
    private Pose intake3Mid = intake3.withX(48);
    private Pose park = new Pose(48, 72, Math.toRadians(180));

    @Override
    public void runOpMode() {

        //robot = new Robot(hardwareMap, telemetry);
        //follower = robot.f;
        
        shooter = new Shooter(hardwareMap);
        shooter2 = new Shooterv2(hardwareMap);

        follower.setStartingPose(start);

        telemetry.addLine("Red Slow 12 Auto Ready");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // Score preload
            followPathAndScore(scoreP());

            // Intake 1
            followPath(alignIntake1());
            followPath(intake1());

            // Score 1
            followPathAndScore(score1());

            // Intake 2
            followPath(alignIntake2());
            followPath(intake2());

            // Score 2
            followPathAndScore(score2());

            // Intake 3
            followPath(alignIntake3());
            followPath(intake3());

            // Score 3
            followPathAndScore(score3());

            // Park
            followPath(park());

            telemetry.addLine("Auto Complete");
            telemetry.update();
        }
    }

    private void followPath(PathChain path) {
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            
            shooter.periodic();
            shooter2.periodic();
            
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }

    private void followPathAndScore(PathChain path) {
        follower.followPath(path);

        // Start shooter
        shooter.close();
        shooter2.close();

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            
            // MUST call periodic to update shooter motors
            shooter.periodic();
            shooter2.periodic();
            
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addLine("Shooter: RUNNING");
            telemetry.update();
        }

        // Wait for settle
        sleep(500);

        // Turn off shooter
        shooter.off();
        shooter2.off();
        
        // Keep calling periodic during off transition
        for (int i = 0; i < 10; i++) {
            shooter.periodic();
            shooter2.periodic();
            sleep(10);
        }

        telemetry.addLine("Shooter: STOPPED");
        telemetry.update();
    }

    private PathChain scoreP() {
        return follower.pathBuilder()
                .addPath(new BezierCurve(start, scorePControl, score))
                .setLinearHeadingInterpolation(start.getHeading(), score.getHeading())
                .build();
    }

    private PathChain alignIntake1() {
        return follower.pathBuilder()
                .addPath(new BezierLine(score, intake1Mid))
                .setLinearHeadingInterpolation(score.getHeading(), intake1Mid.getHeading(), 0.5)
                .build();
    }

    private PathChain intake1() {
        return follower.pathBuilder()
                .addPath(new BezierLine(intake1Mid, intake1))
                .setConstantHeadingInterpolation(intake1.getHeading())
                .setBrakingStrength(0.75)
                .build();
    }

    private PathChain score1() {
        return follower.pathBuilder()
                .addPath(new BezierLine(intake1, score))
                .setLinearHeadingInterpolation(intake1.getHeading(), score.getHeading())
                .build();
    }

    private PathChain alignIntake2() {
        return follower.pathBuilder()
                .addPath(new BezierLine(score, intake2Mid))
                .setLinearHeadingInterpolation(score.getHeading(), intake2Mid.getHeading(), 0.5)
                .build();
    }

    private PathChain intake2() {
        return follower.pathBuilder()
                .addPath(new BezierLine(intake2Mid, intake2))
                .setConstantHeadingInterpolation(intake2.getHeading())
                .setBrakingStrength(0.75)
                .build();
    }

    private PathChain score2() {
        return follower.pathBuilder()
                .addPath(new BezierCurve(intake2, intake2Return, score))
                .setLinearHeadingInterpolation(intake2.getHeading(), score.getHeading())
                .build();
    }

    private PathChain alignIntake3() {
        return follower.pathBuilder()
                .addPath(new BezierLine(score, intake3Mid))
                .setLinearHeadingInterpolation(score.getHeading(), intake3Mid.getHeading(), 0.5)
                .build();
    }

    private PathChain intake3() {
        return follower.pathBuilder()
                .addPath(new BezierLine(intake3Mid, intake3))
                .setConstantHeadingInterpolation(intake3.getHeading())
                .setBrakingStrength(0.75)
                .build();
    }

    private PathChain score3() {
        return follower.pathBuilder()
                .addPath(new BezierLine(intake3, score))
                .setLinearHeadingInterpolation(intake3.getHeading(), score.getHeading())
                .build();
    }

    private PathChain park() {
        return follower.pathBuilder()
                .addPath(new BezierLine(score, park))
                .setLinearHeadingInterpolation(score.getHeading(), park.getHeading())
                .build();
    }
}