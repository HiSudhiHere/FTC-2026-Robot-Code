package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

@TeleOp(name = "Pinpoint Odometry Test")
public class PinpointOdometryTest extends OpMode {

    private Follower follower;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
    }

    @Override
    public void loop() {
        follower.update();

        telemetry.addData("X inches", "%.2f", follower.getPose().getX());
        telemetry.addData("Y inches", "%.2f", follower.getPose().getY());
        telemetry.addData("Heading deg", "%.2f",
                Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addLine("Push robot forward: X should increase");
        telemetry.addLine("Push robot left: Y should increase");
        telemetry.addLine("Turn CCW: heading should increase");

        telemetry.update();
    }
}