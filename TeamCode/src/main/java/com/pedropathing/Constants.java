package com.pedropathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants();

    public static PathConstraints pathConstraints =
            new PathConstraints(0.99, 100, 1, 1);

    public static PinpointConstants pinpointConstants =
            new PinpointConstants()
                    .hardwareMapName("pinpoint")
                    .encoderResolution(
                            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
                    )
                    .forwardEncoderDirection(
                            GoBildaPinpointDriver.EncoderDirection.FORWARD
                    )
                    .strafeEncoderDirection(
                            GoBildaPinpointDriver.EncoderDirection.FORWARD
                    )
                    .forwardPodY(4.75)
                    .strafePodX(0.0);

    public static MecanumConstants mecanumConstants =
            new MecanumConstants()
                    .leftFrontMotorName("FL")
                    .leftRearMotorName("BL")
                    .rightFrontMotorName("FR")
                    .rightRearMotorName("BR")
                    .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .maxPower(0.25);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(pinpointConstants)
                .mecanumDrivetrain(mecanumConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}