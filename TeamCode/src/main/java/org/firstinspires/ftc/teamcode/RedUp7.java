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

@Autonomous(name = "RedUp7", group = "Autonomous")
@Configurable
public class RedUp7 extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;

    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private ServoSubsystem servos;
    private TurretSubsystem turret;
    private static double  value = 4;

    private ElapsedTime waitTimer = new ElapsedTime();

    private static final double STOPPER_OPEN = 0.6;
    private static final double STOPPER_CLOSED = 0.3;

    private static final double SHOOT_TIME = 1000;
    private static final double SHOOT_VELOCITY_READY = 1300;
    private static final double MAX_WAIT_FOR_SHOOTER = 1500;
    private static final double INTAKE_OPEN_WAIT = 1000;

    private boolean shootingStarted = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(110, 133, Math.toRadians(270)));

        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        servos = new ServoSubsystem(hardwareMap);

        turret = new TurretSubsystem(hardwareMap);
        turret.setFieldAngle(0);
        turret.setOffset(0);

        servos.setStopper(STOPPER_CLOSED);
        servos.setHudder(0.25);

        paths = new Paths(follower);

        follower.setMaxPower(0.95);

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
        telemetry.addData("Turret Field Lock", "%.1f", turret.getLockedFieldAngle());

        telemetry.update();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Turret Position", turret.getPosition());
        panelsTelemetry.debug("Turret Field Lock", turret.getLockedFieldAngle());
        panelsTelemetry.update(telemetry);
    }

    private void startShootPath(PathChain path, int nextState) {
        shooter.shootFast();
        servos.setStopper(STOPPER_CLOSED);
        intake.stop();
        shootingStarted = false;

        follower.followPath(path);
        pathState = nextState;
    }

    private void runShootSequence(int nextState) {
        shooter.shootFast();

        if (!shootingStarted) {
            servos.setStopper(STOPPER_CLOSED);
            intake.stop();

            if (shooter.getAverageVelocity() > SHOOT_VELOCITY_READY
                    || waitTimer.milliseconds() > MAX_WAIT_FOR_SHOOTER) {
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

    private void runIntakeOpenWait(int nextState) {
        shooter.stop();
        servos.setStopper(STOPPER_CLOSED);
        intake.intakeOut();

        if (waitTimer.milliseconds() >= INTAKE_OPEN_WAIT) {
            intake.stop();
            pathState = nextState;
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.setMaxPower(0.95);
                startShootPath(paths.shoot1, 1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    shootingStarted = false;
                    pathState = 2;
                }
                break;

            case 2:
                runShootSequence(3);
                break;

            case 3:
                follower.setMaxPower(0.80);
                startIntakePath(paths.intake1, 4);
                break;

            case 4:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    pathState = 5;
                }
                break;

            case 5:
                runIntakeOpenWait(6);
                break;

            case 6:
                follower.setMaxPower(0.95);
                startShootPath(paths.shoot2, 7);
                break;

            case 7:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    shootingStarted = false;
                    pathState = 8;
                }
                break;

            case 8:
                runShootSequence(9);
                break;

            case 9:
                startIntakePath(paths.intakeopen1, 10);
                break;

            case 10:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    pathState = 11;
                }
                break;

            case 11:
                runIntakeOpenWait(12);
                break;

            case 12:
                startShootPath(paths.shoot3, 13);
                break;

            case 13:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    shootingStarted = false;
                    pathState = 14;
                }
                break;

            case 14:
                runShootSequence(15);
                break;

            case 15:
                startIntakePath(paths.intakeopen2, 16);
                break;

            case 16:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    pathState = 17;
                }
                break;

            case 17:
                runIntakeOpenWait(18);
                break;

            case 18:
                startShootPath(paths.shoot4, 19);
                break;

            case 19:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    shootingStarted = false;
                    pathState = 20;
                }
                break;

            case 20:
                runShootSequence(21);
                break;

            case 21:
                startIntakePath(paths.intakeopen3, 22);
                break;

            case 22:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    pathState = 23;
                }
                break;

            case 23:
                runIntakeOpenWait(24);
                break;

            case 24:
                startShootPath(paths.shoot5, 25);
                break;

            case 25:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    shootingStarted = false;
                    pathState = 26;
                }
                break;

            case 26:
                runShootSequence(27);
                break;

            case 27:
                startIntakePath(paths.intakeopen4, 28);
                break;

            case 28:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    pathState = 29;
                }
                break;

            case 29:
                runIntakeOpenWait(30);
                break;

            case 30:
                startShootPath(paths.shoot6, 31);
                break;

            case 31:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    shootingStarted = false;
                    pathState = 32;
                }
                break;

            case 32:
                runShootSequence(33);
                break;

            case 33:
                startIntakePath(paths.intakeopen5, 34);
                break;

            case 34:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    pathState = 35;
                }
                break;

            case 35:
                runIntakeOpenWait(36);
                break;

            case 36:
                startShootPath(paths.shoot7, 37);
                break;

            case 37:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    shootingStarted = false;
                    pathState = 38;
                }
                break;

            case 38:
                runShootSequence(39);
                break;

            case 39:
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
        public PathChain intakeopen1;
        public PathChain shoot3;
        public PathChain intakeopen2;
        public PathChain shoot4;
        public PathChain intakeopen3;
        public PathChain shoot5;
        public PathChain intakeopen4;
        public PathChain shoot6;
        public PathChain intakeopen5;
        public PathChain shoot7;

        public Paths(Follower follower) {

            shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(110.000, 133.000),
                            new Pose(93.000, 92.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                    .build();

            intake1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(93.000, 92.000),
                            new Pose(77.000, 49.000),
                            new Pose(126.000-value, 59.500)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            shoot2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(126.000-value, 59.500),
                            new Pose(88.000, 31.000),
                            new Pose(81.000, 79.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intakeopen1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(81.000, 79.000),
                            new Pose(88.000, 31.000),
                            new Pose(130.000-value, 57.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            shoot3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(130.000-value, 57.000),
                            new Pose(88.000, 58.000),
                            new Pose(81.000, 79.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intakeopen2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(81.000, 79.000),
                            new Pose(88.000, 31.000),
                            new Pose(130.000-value, 57.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            shoot4 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(130.000-value, 57.000),
                            new Pose(88.000, 58.000),
                            new Pose(81.000, 79.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intakeopen3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(81.000, 79.000),
                            new Pose(88.000, 31.000),
                            new Pose(130.000-value, 57.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            shoot5 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(130.000-value, 57.000),
                            new Pose(88.000, 58.000),
                            new Pose(81.000, 79.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intakeopen4 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(81.000, 79.000),
                            new Pose(88.000, 31.000),
                            new Pose(130.000-value, 57.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            shoot6 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(130.000-value, 57.000),
                            new Pose(88.000, 58.000),
                            new Pose(81.000, 79.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            intakeopen5 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(81.000, 79.000),
                            new Pose(88.000, 31.000),
                            new Pose(130.000-value, 57.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();

            shoot7 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(130.000-value, 57.000),
                            new Pose(88.000, 58.000),
                            new Pose(81.000, 79.000)
                    ))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
        }
    }
}