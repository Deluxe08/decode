package org.firstinspires.ftc.teamcode.config.pedro;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.localizers.OTOSLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.3, 0.1211,0.0017011))
            .forwardZeroPowerAcceleration(-42.072)
            .lateralZeroPowerAcceleration(-67.58)
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1, 0, .01, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.01, 0))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.015,0,0.003,0))
            .useSecondaryDrivePIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true);

    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .useBrakeModeInTeleOp(true)
            .xVelocity(76.82)
            .yVelocity(61.54)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
//    public static OTOSConstants localizerConstants1 = new OTOSConstants()
//            .hardwareMapName("otos")
//            .linearUnit(DistanceUnit.INCH) //defualt is inchs and radians
//            .angleUnit(AngleUnit.RADIANS);
//            .linearScalar(multiplier);
//            .angularScalar(multiplier);
    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
        .forwardTicksToInches(.001989436789)
        .strafeTicksToInches(.001989436789)
        .turnTicksToInches(.001989436789)
            .leftPodY(4)
            .rightPodY(-4)
            .strafePodX(-6.5)
            .leftEncoder_HardwareMapName("leftFront")
            .rightEncoder_HardwareMapName("rightFront")
            .strafeEncoder_HardwareMapName("leftRear")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.95, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                //.OTOSLocalizer(localizerConstants1)
                .threeWheelLocalizer(localizerConstants)
                .build();
    }
}