package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rb")
            .leftRearMotorName("lb")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotor.Direction.REVERSE)
            .leftRearMotorDirection(DcMotor.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotor.Direction.FORWARD)
            .rightRearMotorDirection(DcMotor.Direction.FORWARD)
            .xVelocity(61.56)
            .yVelocity(44.22)
    ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.37)
            .strafePodX(0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
    ;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.8)
            .forwardZeroPowerAcceleration(-54.59)
            .lateralZeroPowerAcceleration(-63.72)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.025, 0, 0, 0.06))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.02, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(1.3,0,0.2,0.05))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.15, 0, 0, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0, 0.6, 0.06))
            .useSecondaryDrivePIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true)
    ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            0.85,
            1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
