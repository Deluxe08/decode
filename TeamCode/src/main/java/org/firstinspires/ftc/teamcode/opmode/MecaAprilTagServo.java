package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp
public class MecaAprilTagServo extends OpMode {

    private Limelight3A limelight3A;
    private Servo angleServo;
    private IMU imu;

    double FAR_CM = 250;
    double MID_CM = 160;
    double CLOSE_CM = 90;

    @Override
    public void init() {

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        angleServo = hardwareMap.get(Servo.class, "angleServo");
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(parameters);

        limelight3A.pipelineSwitch(6);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight3A.start();
        imu.resetYaw();
    }

    @Override
    public void loop() {

        // --- IMU TELEMETRY (ALWAYS ON) ---
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        double yaw = orientation.getYaw(AngleUnit.DEGREES);
        double pitch = orientation.getPitch(AngleUnit.DEGREES);
        double roll = orientation.getRoll(AngleUnit.DEGREES);

        telemetry.addData("IMU Yaw", yaw);
        telemetry.addData("IMU Pitch", pitch);
        telemetry.addData("IMU Roll", roll);

        limelight3A.updateRobotOrientation(yaw);

        // --- LIMELIGHT TELEMETRY (ALWAYS ON) ---
        LLResult result = limelight3A.getLatestResult();

        if (result == null) {
            telemetry.addLine("Limelight: No Data");
        }
        else if (!result.isValid()) {
            telemetry.addLine("Limelight: No Tag Detected");
        }
        else {

            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

            if (tags.isEmpty()) {
                telemetry.addLine("Limelight: No Tag Found");
            }
            else {

                telemetry.addData("Tags Detected", tags.size());

                for (LLResultTypes.FiducialResult tag : tags) {

                    int id = tag.getFiducialId();
                    Pose3D pose = tag.getTargetPoseCameraSpace();

                    double x = pose.getPosition().x;
                    double yPos = pose.getPosition().y;
                    double z = pose.getPosition().z;

                    double distanceCm = Math.sqrt(x * x + yPos * yPos + z * z);

                    telemetry.addData("Tag ID", id);
                    telemetry.addData("Distance (cm)", distanceCm);

                    // Servo logic
                    double servoPos;

                    if (distanceCm >= FAR_CM) {
                        servoPos = 0.2;
                    }
                    else if (distanceCm >= MID_CM) {
                        servoPos = 0.55;
                    }
                    else {
                        servoPos = 0.8;
                    }

                    angleServo.setPosition(servoPos);
                    telemetry.addData("Servo Position", servoPos);
                }
            }
        }

        telemetry.update();
    }
}