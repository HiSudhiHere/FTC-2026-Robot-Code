package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Autonomous(name = "Pedro Slow Drive Test", group = "Test")
public class PedroSlowDriveTest extends OpMode {

    private Follower follower;
    private PathChain testPath;

    private int pathState = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(101, 8, Math.toRadians(0)));

        follower.setMaxPower(0.25);

        testPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(101, 8),
                        new Pose(111, 8)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(90)
                )
                .build();

        telemetry.addLine("Ready");
        telemetry.addData("Start X", 101);
        telemetry.addData("Start Y", 8);
        telemetry.addData("Start Heading", "0 degrees");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();

        switch (pathState) {
            case 0:
                follower.followPath(testPath);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    pathState = 2;
                }
                break;

            case 2:
                follower.breakFollowing();
                break;
        }

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading Deg", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}