package org.firstinspires.ftc.teamcode;

import com.pedropathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Pedro Slow Auto")
public class PedroSimpleAuto extends LinearOpMode {

    private Follower follower;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);

        // Starting Pose
        follower.setStartingPose(new Pose(0, 0, 0));

        // VERY SLOW
        follower.setMaxPower(0.10);

        // Create path
        Path path = new Path(
                new BezierCurve(
                        new Pose(0, 0),
                        new Pose(43.15, 4),
                        new Pose(86.29, 8)
                )
        );

        // Keep heading at 0 degrees
        path.setConstantHeadingInterpolation(0);

        waitForStart();

        if (isStopRequested()) return;

        follower.followPath(path);

        while (opModeIsActive()) {

            follower.update();

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading",
                    Math.toDegrees(follower.getPose().getHeading()));

            telemetry.update();

            if (!follower.isBusy()) {
                break;
            }
        }
    }
}