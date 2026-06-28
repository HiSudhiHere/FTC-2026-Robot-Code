package org.firstinspires.ftc.teamcode;

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

@Autonomous(name = "RedDown4", group = "Autonomous")
@Configurable
public class RedDown4 extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;

    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private ServoSubsystem servos;
    private TurretSubsystem turret;

    private final ElapsedTime waitTimer = new ElapsedTime();

    private static final double STOPPER_OPEN = 0.6;
    private static final double STOPPER_CLOSED = 0.3;

    private static final double SHOOT_TIME = 1000;
    private static final double SHOOT_VELOCITY_READY = 1450;

    private static final double TURRET_ANGLE = 72;

    private boolean shootingStarted = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(101, 8, Math.toRadians(0)));
        follower.setMaxPower(0.65);

        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        servos = new ServoSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);

        turret.setFieldAngle(TURRET_ANGLE);

        servos.setStopper(STOPPER_CLOSED);
        servos.setHudder(0.3);

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
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
        turret.setFieldAngle(TURRET_ANGLE);
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
        intake.intakeIn();

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
        intake.intakeIn();

        follower.followPath(path);
        pathState = nextState;
    }

    private void startNormalPath(PathChain path, int nextState) {
        shooter.stop();
        intake.stop();
        servos.setStopper(STOPPER_CLOSED);

        follower.followPath(path);
        pathState = nextState;
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                startShootPath(paths.shoot1, 1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    shootingStarted = false;
                    pathState = 2;
                }
                break;

            case 2:
                runShootSequence(3);
                break;

            case 3:
                startIntakePath(paths.intake1, 4);
                break;

            case 4:
                if (!follower.isBusy()) {
                    intake.stop();
                    startShootPath(paths.shoot2, 5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    shootingStarted = false;
                    pathState = 6;
                }
                break;

            case 6:
                runShootSequence(7);
                break;

            case 7:
                startNormalPath(paths.pathTointake, 8);
                break;

            case 8:
                if (!follower.isBusy()) {
                    startIntakePath(paths.intake2, 9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    intake.stop();
                    startShootPath(paths.shoot3, 10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    shootingStarted = false;
                    pathState = 11;
                }
                break;

            case 11:
                runShootSequence(12);
                break;

            case 12:
                startNormalPath(paths.pathTointake2, 13);
                break;

            case 13:
                if (!follower.isBusy()) {
                    startIntakePath(paths.intake3, 14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    intake.stop();
                    startShootPath(paths.shoot4, 15);
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    shootingStarted = false;
                    pathState = 16;
                }
                break;

            case 16:
                runShootSequence(17);
                break;

            case 17:
                intake.stop();
                shooter.stop();
                turret.stop();
                servos.setStopper(STOPPER_CLOSED);
                break;
        }
    }

    public static class Paths {
        public PathChain shoot1;
        public PathChain intake1;
        public PathChain shoot2;
        public PathChain pathTointake;
        public PathChain intake2;
        public PathChain shoot3;
        public PathChain pathTointake2;
        public PathChain intake3;
        public PathChain shoot4;

        public Paths(Follower follower) {

            shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(101.000, 8.000),
                            new Pose(95.000, 9.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            intake1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(95.000, 9.000),
                            new Pose(93.000, 36.000),
                            new Pose(126.000, 35.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(126.000, 35.000),
                            new Pose(95.000, 9.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            pathTointake = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(95.000, 9.000),
                            new Pose(128.000, 15.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            intake2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(128.000, 15.000),
                            new Pose(128.000, 7.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(128.000, 7.000),
                            new Pose(95.000, 9.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            pathTointake2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(95.000, 9.000),
                            new Pose(126.000, 34.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            intake3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(126.000, 34.000),
                            new Pose(126.000, 13.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            shoot4 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(126.000, 13.000),
                            new Pose(94.000, 9.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }
}