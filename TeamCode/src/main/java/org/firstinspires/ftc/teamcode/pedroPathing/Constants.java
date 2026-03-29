package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static OTOSConstants localizerConstants = new OTOSConstants()
            .hardwareMapName("otos")
            .linearScalar(0.98688763333333333333333333333333)
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .offset(new SparkFunOTOS.Pose2D(0,0,(Math.toRadians(90))));

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(6.35029)
            .forwardZeroPowerAcceleration(-66)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .centripetalScaling(0.003)
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.08,0.05,0.04,0.3,0.0001))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.2,0.0,0.15,0.6,0.008))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.16, 0.001, 0.01, 0.008))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0.001, 0.001, 0.008))
            .headingPIDFCoefficients(new PIDFCoefficients(0.6, 0, 0.01, 0.016))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.38, 0.001, 0.05, 0.012));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(56.55)
            .yVelocity(51.03)
            .rightFrontMotorName("RightFrontDrive")
            .rightRearMotorName("RightBackDrive")
            .leftRearMotorName("LeftBackDrive")
            .leftFrontMotorName("LeftFrontDrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.3, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .OTOSLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}