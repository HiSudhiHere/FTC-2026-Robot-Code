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
@Disabled
@Autonomous(name = "BlueDown8", group = "Autonomous")
@Configurable
public class BlueDown8 extends OpMode {

    private TelemetryManager panelsTelemetry;

    public Follower follower;
    private int pathState = 0;
    private Paths paths;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private ServoSubsystem servos;
    private TurretSubsystem turret;
    private ElapsedTime waitTimer = new ElapsedTime();

    private static final double STOPPER_OPEN = 0.6;
    private static final double STOPPER_CLOSED = 0.3;
    private static final double SHOOT_TIME = 600;
    private static final double SHOOT_VELOCITY_READY = 1440;

    private static int value = 3;

    private boolean shootingStarted = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(40.500, 8.000, Math.toRadians(180)));

        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        servos = new ServoSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);

        turret.setFieldAngle(0);
        turret.setOffset(110);

        servos.setStopper(STOPPER_CLOSED);
        servos.setHudder(0.55);

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        follower.setMaxPower(0.95);
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
                startNormalPath(paths.up1, 4);
                break;

            case 4:
                if (!follower.isBusy()) {
                    startIntakePath(paths.upIntake, 5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    intake.stop();
                    startShootPath(paths.shoot2, 6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    shootingStarted = false;
                    pathState = 7;
                }
                break;

            case 7:
                runShootSequence(8);
                break;

            case 8:
                startIntakePath(paths.down1, 9);
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
                startIntakePath(paths.down2, 13);
                break;

            case 13:
                if (!follower.isBusy()) {
                    intake.stop();
                    startShootPath(paths.shoot4, 14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    shootingStarted = false;
                    pathState = 15;
                }
                break;

            case 15:
                runShootSequence(16);
                break;

            case 16:
                startIntakePath(paths.down3, 17);
                break;

            case 17:
                if (!follower.isBusy()) {
                    intake.stop();
                    startShootPath(paths.shoot5, 18);
                }
                break;

            case 18:
                if (!follower.isBusy()) {
                    shootingStarted = false;
                    pathState = 19;
                }
                break;

            case 19:
                runShootSequence(20);
                break;

            case 20:
                startIntakePath(paths.down4, 21);
                break;

            case 21:
                if (!follower.isBusy()) {
                    intake.stop();
                    startShootPath(paths.shoot6, 22);
                }
                break;

            case 22:
                if (!follower.isBusy()) {
                    shootingStarted = false;
                    pathState = 23;
                }
                break;

            case 23:
                runShootSequence(24);
                break;

            case 24:
                startIntakePath(paths.up2, 25);
                break;

            case 25:
                if (!follower.isBusy()) {
                    intake.stop();
                    startShootPath(paths.shoot7, 26);
                }
                break;

            case 26:
                if (!follower.isBusy()) {
                    shootingStarted = false;
                    pathState = 27;
                }
                break;

            case 27:
                runShootSequence(28);
                break;

            case 28:
                startIntakePath(paths.down5, 29);
                break;

            case 29:
                if (!follower.isBusy()) {
                    intake.stop();
                    startShootPath(paths.shoot8, 30);
                }
                break;

            case 30:
                if (!follower.isBusy()) {
                    shootingStarted = false;
                    pathState = 31;
                }
                break;

            case 31:
                runShootSequence(32);
                break;

            case 32:
                intake.stop();
                shooter.stop();
                turret.stop();
                servos.setStopper(STOPPER_CLOSED);
                break;
        }
    }

    public static class Paths {
        public PathChain shoot1;
        public PathChain up1;
        public PathChain upIntake;
        public PathChain shoot2;
        public PathChain down1;
        public PathChain shoot3;
        public PathChain down2;
        public PathChain shoot4;
        public PathChain down3;
        public PathChain shoot5;
        public PathChain down4;
        public PathChain shoot6;
        public PathChain up2;
        public PathChain shoot7;
        public PathChain down5;
        public PathChain shoot8;

        public Paths(Follower follower) {
            shoot1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(40.500, 8.000),
                                    new Pose(46.500, 8.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            up1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(46.500, 8.000),
                                    new Pose(44.500, 35.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            upIntake = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(44.500, 35.000),
                                    new Pose(15.500, 35.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(15.500, 35.000),
                                    new Pose(46.500, 8.715)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            down1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(46.500, 8.715),
                                    new Pose(8.500, 9.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(8.500, 9.000),
                                    new Pose(46.500, 9.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            down2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(46.500, 9.000),
                                    new Pose(25.500, 12.000),
                                    new Pose(8.500, 9.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(8.500, 9.000),
                                    new Pose(46.500, 9.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            down3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(46.500, 9.000),
                                    new Pose(41.500, 19.000),
                                    new Pose(8.500, 16.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(8.500, 16.000),
                                    new Pose(46.500, 9.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            down4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(46.500, 9.000),
                                    new Pose(8.500, 9.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot6 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(8.500, 9.000),
                                    new Pose(46.500, 9.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            up2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(46.500, 9.000),
                                    new Pose(40.500, 30.000),
                                    new Pose(8.500, 30.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot7 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(8.500, 30.000),
                                    new Pose(46.500, 9.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            down5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(46.500, 9.000),
                                    new Pose(8.500, 9.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot8 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(8.500, 9.000),
                                    new Pose(46.500, 9.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();
        }
    }
}