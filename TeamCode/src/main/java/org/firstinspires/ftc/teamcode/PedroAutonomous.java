package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import com.pedropathing.Constants;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
@Disabled
@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable
public class PedroAutonomous extends OpMode {

    private TelemetryManager panelsTelemetry;

    private Follower follower;

    private Paths paths;

    private int pathState = 0;

    @Override
    public void init() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        // MUST MATCH THE FIRST POINT OF THE PATH
        follower.setStartingPose(new Pose(8, 8, Math.toRadians(0)));
        follower.setMaxPower(0.50);

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {

        // Start following the path once Start is pressed
        follower.followPath(paths.MainChain);

        pathState = 1;
    }

    @Override
    public void loop() {

        follower.update();

        if (pathState == 1) {

            if (!follower.isBusy()) {
                pathState = 2;
            }
        }

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));

        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain MainChain;

        public Paths(Follower follower) {

            MainChain = follower.pathBuilder()

                    .addPath(
                            new BezierLine(
                                    new Pose(8, 8),
                                    new Pose(70, 70)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(90)
                    )

                    .build();
        }
    }
}