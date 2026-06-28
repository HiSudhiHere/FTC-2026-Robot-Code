package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import com.pedropathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Autonomous(name = "RedDown7", group = "Autonomous")
@Configurable
public class RedDown7 extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private RedDown8.Paths paths;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private ServoSubsystem servos;
    private TurretSubsystem turret;
    private ElapsedTime waitTimer = new ElapsedTime();

    private static final double STOPPER_OPEN = 0.6;
    private static final double STOPPER_CLOSED = 0.3;
    private static final double SHOOT_TIME = 600;
    private static final double SHOOT_VELOCITY_READY = 1490;

    private static int value = 5;

    private boolean shootingStarted = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(101, 8, Math.toRadians(0)));

        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        servos = new ServoSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);

        turret.setFieldAngle(75);

        servos.setStopper(STOPPER_CLOSED);
        servos.setHudder(0.05);

        paths = new RedDown8.Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        follower.setMaxPower(0.65);

    }

    @Override
    public void loop() {
        follower.update();
        turret.update(Math.toDegrees(follower.getPose().getHeading()));
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", "%.2f", follower.getPose().getX());
        telemetry.addData("Y", "%.2f", follower.getPose().getY());
        telemetry.addData("Heading Deg", "%.2f", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addLine("===== SHOOTER =====");
        telemetry.addData("Left Velocity", "%.1f", shooter.getLeftVelocity());
        telemetry.addData("Right Velocity", "%.1f", shooter.getRightVelocity());
        telemetry.addData("Average Velocity", "%.1f", shooter.getAverageVelocity());
        telemetry.addData("Shooting Started", shootingStarted);

        telemetry.addLine("===== TURRET =====");
        telemetry.addData("Turret Position", turret.getPosition());
        telemetry.addData("Locked Field Angle", "%.1f", turret.getLockedFieldAngle());

        telemetry.update();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


    private void startShootPath(PathChain path, int nextState) {
        shooter.longShoot();
        servos.setStopper(STOPPER_CLOSED);
        intake.stop();
        shootingStarted = false;

        follower.followPath(path);
        pathState = nextState;
    }

    private void runShootSequence(int nextState) {
        shooter.longShoot();

        if (!shootingStarted) {
            servos.setStopper(STOPPER_CLOSED);
            intake.stop();

            if (shooter.getAverageVelocity() > SHOOT_VELOCITY_READY) {
                shootingStarted = true;
                waitTimer.reset();
            }

            return;
        }

        servos.setStopper(STOPPER_OPEN);
        intake.intakeOut();

        if (waitTimer.milliseconds() >= SHOOT_TIME) {
            servos.setStopper(STOPPER_CLOSED);
            intake.stop();
            shooter.stop();

            shootingStarted = false;
            pathState = nextState;
        }
    }

    private void startIntakePath(PathChain path, int nextState) {
        shooter.stop();
        servos.setStopper(STOPPER_CLOSED);

        intake.intakeOut();

        follower.followPath(path);
        pathState = nextState;
    }

    private void startNormalPath(PathChain  path, int nextState) {
        shooter.stop();
        intake.stop();
        servos.setStopper(STOPPER_CLOSED);

        follower.followPath(path);
        pathState = nextState;
    }
    public void autonomousPathUpdate() {
        switch (pathState) {

        }
    }
    public static class Paths {
        public PathChain intake1;
        public PathChain shoot2;
        public PathChain intake2;
        public PathChain shoot3;
        public PathChain intake3;
        public PathChain shoot4;
        public PathChain intake4;
        public PathChain shoot5;
        public PathChain intake5;
        public PathChain shoot6;
        public PathChain intake6;
        public PathChain shoot7;

        public Paths(Follower follower) {
            intake1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(86.000, 8.000),
                                    new Pose(85.000, 38.000),
                                    new Pose(126.000, 35.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .addPath(
                            new BezierLine(
                                    new Pose(95.000, 9.000),
                                    new Pose(108.000, 9.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            shoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(126.000, 35.000),
                                    new Pose(112.500, 24.000),
                                    new Pose(95.000, 9.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            intake2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(95.000, 9.000),
                                    new Pose(128.000, 9.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            shoot3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(128.000, 9.000),
                                    new Pose(95.000, 9.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            intake3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(95.000, 9.000),
                                    new Pose(127.000, 14.000),
                                    new Pose(128.000, 42.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(80))
                    .build();

            shoot4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(128.000, 42.000),
                                    new Pose(95.000, 9.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(80), Math.toRadians(0))
                    .build();

            intake4 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(95.000, 9.000),
                                    new Pose(130.000, 7.000),
                                    new Pose(128.000, 42.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();

            shoot5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(128.000, 42.000),
                                    new Pose(95.000, 9.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            intake5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(95.000, 9.000),
                                    new Pose(128.000, 9.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            shoot6 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(128.000, 9.000),
                                    new Pose(95.000, 9.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            intake6 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(95.000, 9.000),
                                    new Pose(99.000, 37.000),
                                    new Pose(126.000, 35.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            shoot7 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(126.000, 35.000),
                                    new Pose(95.000, 9.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }
}