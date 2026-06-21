package org.firstinspires.ftc.teamcode;

import com.pedropathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled
@Autonomous(name = "Go To (-60,0)")
public class GoToMinus60 extends LinearOpMode {

    private Follower follower;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);

        // Robot starts here
        follower.setStartingPose(new Pose(8, 8, 0));

        // Move very slowly
        follower.setMaxPower(0.10);

        // Straight path
        Path path = new Path(
                new BezierCurve(
                        new Pose(8, 8),
                        new Pose(-70, 8)
                )
        );

        path.setConstantHeadingInterpolation(0);

        waitForStart();

        if (isStopRequested()) return;

        follower.followPath(path);

        while (opModeIsActive() && follower.isBusy()) {

            follower.update();

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading",
                    Math.toDegrees(follower.getPose().getHeading()));

            telemetry.update();
        }
    }
}