package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Autonomous(name = "RedUp-4", group = "Auto")
@Configurable
public class RedUp4 extends OpMode {

    private static Follower follower;

    private static ShooterSubsystem shooter;
    private static IntakeSubsystem intake;
    private static ServoSubsystem servos;
    private static TurretSubsystem turret;

    private Paths paths;
    private int pathState = 0;

    private final ElapsedTime actionTimer = new ElapsedTime();

    private static final double STOPPER_OPEN = 0.60;
    private static final double STOPPER_CLOSE = 0.30;

    private static final int FEED_TIME = 1400;
    private static final double SHOOTER_READY = 1280;

    private static double value = 4;

    // RedUp7-style fixed turret angles
    private static final double SHOOT1_TURRET = 55;
    private static final double SHOOT2_TURRET = 50;
    private static final double SHOOT3_TURRET = 50;
    private static final double SHOOT4_TURRET = 50;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(
                new Pose(110, 131, Math.toRadians(90))
        );

        follower.setMaxPower(0.90);

        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        servos = new ServoSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);

        servos.setHudder(0.40);
        servos.setStopper(STOPPER_CLOSE);

        turret.setFieldAngle(SHOOT1_TURRET);

        paths = new Paths(follower);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        follower.update();

        turret.update(
                Math.toDegrees(follower.getPose().getHeading())
        );

        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Shooter", shooter.getAverageVelocity());
        telemetry.addData("Turret Position", turret.getPosition());
        telemetry.update();
    }

    private void startShootPath(PathChain path, int nextState, double turretAngle) {
        turret.setFieldAngle(turretAngle);
        shooter.shootSlow();
        intake.stop();
        //servos.setStopper(STOPPER_CLOSE);

        follower.followPath(path);
        pathState = nextState;
    }

    private void startIntakePath(PathChain path, int nextState) {
        shooter.stop();
        servos.setStopper(STOPPER_CLOSE);
        intake.intakeIn();

        follower.followPath(path);
        pathState = nextState;
    }

    private void runShootSequence(int nextState) {
        shooter.shootSlow();

        if (shooter.getAverageVelocity() >= SHOOTER_READY) {
            servos.setStopper(STOPPER_OPEN);
            intake.intakeIn();
        }

        if (actionTimer.milliseconds() >= FEED_TIME) {
            servos.setStopper(STOPPER_CLOSE);
            intake.stop();
            shooter.stop();
            pathState = nextState;
        }
    }

    private void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                turret.setFieldAngle(SHOOT1_TURRET);
                shooter.shootSlow();
                follower.followPath(paths.shoot1);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    servos.setStopper(STOPPER_CLOSE);
                    intake.stop();
                    shooter.stop();
                    startIntakePath(paths.intake1, 2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    startShootPath(paths.shoot2, 3, SHOOT2_TURRET);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    actionTimer.reset();
                    pathState = 4;
                }
                break;

            case 4:
                runShootSequence(5);
                break;

            case 5:
                startIntakePath(paths.intake2, 6);
                break;

            case 6:
                if (!follower.isBusy()) {
                    startShootPath(paths.shoot3, 7, SHOOT3_TURRET);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    actionTimer.reset();
                    pathState = 8;
                }
                break;

            case 8:
                runShootSequence(9);
                break;

            case 9:
                startIntakePath(paths.intake3, 10);
                break;

            case 10:
                if (!follower.isBusy()) {
                    startShootPath(paths.shoot4, 11, SHOOT4_TURRET);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    actionTimer.reset();
                    pathState = 12;
                }
                break;

            case 12:
                runShootSequence(13);
                break;

            case 13:
                intake.stop();
                shooter.stop();
                turret.stop();
                servos.setStopper(STOPPER_CLOSE);
                break;
        }
    }

    public static class Paths {

        public PathChain shoot1;
        public PathChain intake1;
        public PathChain shoot2;
        public PathChain intake2;
        public PathChain shoot3;
        public PathChain intake3;
        public PathChain shoot4;

        public Paths(Follower follower) {

            shoot1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(110.000, 133.000),
                                    new Pose(95.000, 103.000),
                                    new Pose(96.000, 82.000)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(90)
                    )
                    .addParametricCallback(0.01, () -> {
                        turret.setFieldAngle(SHOOT1_TURRET);
                        servos.setHudder(0.22);
                        servos.setStopper(STOPPER_OPEN);
                    })
                    .addParametricCallback(0.25, () -> {
                        servos.setStopper(STOPPER_OPEN);
                        intake.intakeIn();
                    })
                    .addParametricCallback(0.8, () -> {
                        shooter.stop();
                        intake.stop();
                    })
                    .addParametricCallback(0.9, () -> {
                        follower.setMaxPower(0.9);
                    })
                    .build();

            intake1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(96.000, 82.000),
                                    new Pose(126.000 - value, 82.000)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .addParametricCallback(0.3, () -> {
                        follower.setMaxPower(0.7);
                    })
                    .addParametricCallback(0.9, () -> {
                        follower.setMaxPower(0.9);
                    })
                    .build();

            shoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(126.000 - value, 82.000),
                                    new Pose(84.000, 82.000)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();

            intake2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(84.000, 82.000),
                                    new Pose(80.000, 56.000),
                                    new Pose(126.000, 58.000)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .addParametricCallback(0.3, () -> {
                        follower.setMaxPower(0.7);
                    })
                    .addParametricCallback(0.9, () -> {
                        follower.setMaxPower(0.9);
                    })
                    .build();

            shoot3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(126.000, 58.000),
                                    new Pose(80.000, 56.000),
                                    new Pose(84.000, 82.000)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .build();

            intake3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(84.000, 82.000),
                                    new Pose(82.000, 34.000),
                                    new Pose(126.000, 33.000)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .addParametricCallback(0.3, () -> {
                        follower.setMaxPower(0.7);
                    })
                    .addParametricCallback(0.9, () -> {
                        follower.setMaxPower(0.9);
                    })
                    .build();

            shoot4 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(126.000, 35.000),
                                    new Pose(84.150, 33.970),
                                    new Pose(83.850, 81.383)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0)
                    )
                    .addParametricCallback(0.7, () -> {
                        servos.setStopper(STOPPER_OPEN);
                    } )
                    .build();
        }
    }
}