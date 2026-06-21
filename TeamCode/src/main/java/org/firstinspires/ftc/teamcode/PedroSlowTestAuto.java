package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import com.pedropathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Disabled
@Autonomous(name = "Pedro Slow Test Auto", group = "Test")
@Configurable
public class PedroSlowTestAuto extends OpMode {

    private Follower follower;
    private TelemetryManager panelsTelemetry;
    private PathChain testPath;

    private int pathState = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        // Robot starts at X=0, Y=0, heading=0 degrees
        follower.setStartingPose(new Pose(8, 8, Math.toRadians(0)));

        // Extra slow speed
        follower.setMaxPower(0.20);

        testPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(8, 8),
                        new Pose(24, 8)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(0)
                )
                .build();

        telemetry.addLine("Ready: Pedro Slow Test Auto");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.followPath(testPath);
        pathState = 1;
    }

    @Override
    public void loop() {
        follower.update();

        if (pathState == 1 && !follower.isBusy()) {
            pathState = 2;
        }

        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading Deg", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Busy", follower.isBusy());
        telemetry.update();

        panelsTelemetry.debug("State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.update(telemetry);
    }
}