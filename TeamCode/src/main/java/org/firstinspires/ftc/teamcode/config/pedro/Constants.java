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
            .mass(15)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.3, 0.1211,0.0017011))
            //41.02575982 + 43.2021 + 41.6109
            .forwardZeroPowerAcceleration(-((41.245272005489525 + 39.67044310097189 + 41.59345427820631)/3))
            // 50.08908269509616+ 57.677+ 63.63525
            .lateralZeroPowerAcceleration(-((59.419855854262515 + 56.607054930190664 + 65.94511178562782)/3))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.5, 0, .01, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.5, 0, 0.01, 0))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.035,0,0.0001,0.0))
            .useSecondaryDrivePIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(false);

    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .useBrakeModeInTeleOp(true)
            //67.81884748
            .xVelocity((80.6061791156236 + 78.462433354929717 + 79.59669796573121)/3)
            // 51.566236919319444
            .yVelocity((57.32757529427463 + 55.51439115710849 + 56.74369652783617)/3)
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
            .forwardTicksToInches((.0018787813584525783 + .0018789227352576687 + .0018766837971531463)/3)
            .strafeTicksToInches((.0017564746936741711 + .001746660028079416 + .001751366077302435)/3)
            .turnTicksToInches((.00192565185514933 + .0019258809766741032 + .001925651855149328)/3)
            .leftPodY(5.142716535)
            .rightPodY(-5.161653543)
            .strafePodX(-6.120590551)
            .leftEncoder_HardwareMapName("rightInTake")
            .rightEncoder_HardwareMapName("rightFront")
            .strafeEncoder_HardwareMapName("leftFront")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
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