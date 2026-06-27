package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import com.pedropathing.Constants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;


import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RGBSubsystem;
import com.pedropathing.paths.callbacks.ParametricCallback;
@Disabled
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
    private RGBSubsystem light;

    private ElapsedTime waitTimer = new ElapsedTime();
    private static int value = 2;
    private static final double STOPPER_OPEN = 0.6;
    private static final double STOPPER_CLOSED = 0.3;
    private static final double SHOOT_TIME =850;
    private static final double OPEN_WAIT_TIME = 1000;
    private static final double INTAKE_WAIT_TIME = 800;

    //Turret Angles
    private static final double SHOOT1_TURRET = 65;
    private static final double SHOOT2_TURRET = 55;
    private static final double SHOOT3_TURRET = 55;


    private boolean firstShotStarted = false;
    private static final double SHOOT1_MOVING_SHOOT_TIME = 400;
    private boolean shoot1MovingShotActive = false;
    private ElapsedTime shoot1MovingShotTimer = new ElapsedTime();




    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(110, 131, Math.toRadians(90)));
        follower.setMaxPower(0.90);

        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        servos = new ServoSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        light = new RGBSubsystem(hardwareMap);

        turret.setFieldAngle(40);

        servos.setStopper(STOPPER_CLOSED);
        servos.setHudder(0.3);

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry );
        follower.setMaxPower(0.9);

    }

    @Override
    public void loop() {
        follower.update();
        turret.update(Math.toDegrees(follower.getPose().getHeading()));
        updateShoot1MovingShot();
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();

//        if((!firstShotStarted && y>=100) && (y<=120 && pathState == 1)){
//            firstShotStarted = true;
//            //light.green();
//            shooter.shootVERYSLOW();
//            servos.setStopper(STOPPER_OPEN);
//            intake.intakeIn();
//        }
//        if (pathState == 1 ||
//                pathState == 8 ||
//                pathState == 15 ||
//                pathState == 22 ||
//                pathState == 29 ||
//                pathState == 36 ||
//                pathState == 43) {
//
//            double distanceToShoot =
//                    Math.hypot(84.0 - x, 82.0 - y);
//
//            if (distanceToShoot < 12) {
//                follower.setMaxPower(0.4);
//            } else {
//                follower.setMaxPower(0.9);
//            }
//        }
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", "%.2f", follower.getPose().getX());
        telemetry.addData("Y", "%.2f", follower.getPose().getY());
        telemetry.addData("Heading Deg", "%.2f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Average Velocity", "%.1f", shooter.getAverageVelocity());
        telemetry.addData("Turret Position", turret.getPosition());
        telemetry.update();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    private void startShootPath(PathChain path, int nextState, double turretAngle) {
        turret.setFieldAngle(turretAngle);
        shooter.shootSlow();
        intake.stop();
        servos.setStopper(STOPPER_CLOSED);

        follower.followPath(path);
        pathState = nextState;
    }
    private void startShootPath1(PathChain path, int nextState, double turretAngle) {
        turret.setFieldAngle(turretAngle);
        shooter.shootVERYSLOW();
        intake.stop();
        servos.setStopper(STOPPER_CLOSED);

        shoot1MovingShotActive = false;

        follower.followPath(path);
        pathState = nextState;
    }
    private void startShoot1MovingShot() {
        shoot1MovingShotActive = true;
        shoot1MovingShotTimer.reset();

        shooter.shootVERYSLOW();
        servos.setStopper(STOPPER_OPEN);
        intake.intakeIn();
    }
    private void runShootSequence(int nextState, double h) {
        servos.setHudder(h);
        shooter.shootSlow();
        servos.setStopper(STOPPER_OPEN);

        // If this is physically opposite, change intakeIn() to intakeIn()
        intake.intakeIn();

        if (waitTimer.milliseconds() >= SHOOT_TIME) {
            servos.setStopper(STOPPER_CLOSED);
            intake.stop();
            shooter.stop();
            pathState = nextState;
        }
    }

    private void startIntakePath(PathChain path, int nextState) {
        shooter.stop();
        servos.setStopper(STOPPER_CLOSED);

        // If this is physically opposite, change intakeIn() to intakeIn()
        intake.intakeIn();

        follower.followPath(path);
        pathState = nextState;
    }

    private void startOpenPath(PathChain path, int nextState) {
        shooter.stop();
        intake.stop();
        servos.setStopper(STOPPER_CLOSED);

        follower.followPath(path);
        pathState = nextState;
    }
    private void updateShoot1MovingShot() {
        if (!shoot1MovingShotActive) {
            return;
        }

        if (shoot1MovingShotTimer.milliseconds() < SHOOT1_MOVING_SHOOT_TIME) {
            shooter.shootVERYSLOW();
            servos.setStopper(STOPPER_OPEN);
            intake.intakeIn();
        } else {
            shoot1MovingShotActive = false;

            servos.setStopper(STOPPER_CLOSED);
            intake.stop();
            shooter.stop();
        }
    }
    private void startPathToIntake(PathChain path, int nextState) {
        shooter.stop();
        intake.stop();
        servos.setStopper(STOPPER_CLOSED);

        follower.followPath(path);
        pathState = nextState;
    }

    private void waitWithIntakeOn(int nextState) {
        shooter.stop();
        servos.setStopper(STOPPER_CLOSED);
        intake.intakeIn();

        if (waitTimer.milliseconds() >= INTAKE_WAIT_TIME) {
            intake.stop();
            pathState = nextState;
        }
    }

    private void waitWithIntakeOff(int nextState) {
        shooter.stop();
        intake.stop();
        servos.setStopper(STOPPER_CLOSED);

        if (waitTimer.milliseconds() >= 200) {
            pathState = nextState;
        }
    }


    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                startShootPath1(paths.shoot1, 1, SHOOT1_TURRET);
                break;

            case 1:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    pathState = 2;
                }
                break;

            case 2:
                startIntakePath(paths.intake1, 3);
                break;


            case 3:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    pathState = 4;
                }
                break;

            case 4:
                startShootPath(paths.shoot2, 5, SHOOT2_TURRET);
                break;

            case 5:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    pathState = 6;
                }
                break;

            case 6:
                runShootSequence(7, 0.5);
                break;

            //cycle-1
            case 7:
                startPathToIntake(paths.pathTOintake1, 8);
                break;

            case 8:
                if (!follower.isBusy()) {
                    startIntakePath(paths.intake2, 9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    pathState = 10;
                }
                break;

            case 10:
                waitWithIntakeOn(11);
                break;

            case 11:
                startShootPath(paths.shoot3, 12, SHOOT3_TURRET);
                break;

            case 12:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    pathState = 13;
                }
                break;

            case 13:
                runShootSequence(14, 0.5);
                break;

            //cycle-2
            case 14:
                startPathToIntake(paths.pathTOintake1, 15);
                break;

            case 15:
                if (!follower.isBusy()) {
                    startIntakePath(paths.intake2, 16);
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    pathState = 17;
                }
                break;

            case 17:
                waitWithIntakeOn(18);
                break;

            case 18:
                startShootPath(paths.shoot3, 19, SHOOT3_TURRET);
                break;

            case 19:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    pathState = 20;
                }
                break;

            case 20:
                runShootSequence(21, 0.5);
                break;

            //normal cycle-3
            case 21:
                startPathToIntake(paths.pathTOintake1, 22);
                break;

            case 22:
                if (!follower.isBusy()) {
                    startIntakePath(paths.intake2, 23);
                }
                break;

            case 23:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    pathState = 24;
                }
                break;

            case 24:
                waitWithIntakeOn(25);
                break;

            case 25:
                startShootPath(paths.shoot3, 26, SHOOT3_TURRET);
                break;

            case 26:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    pathState = 27;
                }
                break;

            case 27:
                runShootSequence(28, 0.5);
                break;

            //normal cycle-4
            case 28:
                startPathToIntake(paths.pathTOintake1, 29);
                break;

            case 29:
                if (!follower.isBusy()) {
                    startIntakePath(paths.intake2, 30);
                }
                break;

            case 30:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    pathState = 31;
                }
                break;

            case 31:
                waitWithIntakeOn(32);
                break;

            case 32:
                startShootPath(paths.shoot3, 33, SHOOT3_TURRET);
                break;

            case 33:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    pathState = 34;
                }
                break;

            case 34:
                runShootSequence(35, 0.5);
                break;

            case 35:
                intake.stop();
                shooter.stop();
                turret.stop();
                servos.setStopper(STOPPER_CLOSED);
                break;
        }
    }

    public class Paths {
        public PathChain shoot1;
        public PathChain intake1;
        //        public PathChain open1;
        public PathChain shoot2;
        //        public PathChain intake2;
//        public PathChain open2;
//        public PathChain shoot3;
        public PathChain pathTOintake1;
        public PathChain intake2;
        public PathChain shoot3;

        public Paths(Follower follower) {

            shoot1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(108.000, 133.000),
                            new Pose(91.000,93.000),
                            new Pose(94.000,60.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .addParametricCallback(0.01, () -> {
                        turret.setFieldAngle(65);
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
                        follower.setMaxPower(0.6);

                    })
                    .build();

            intake1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(94.000,60.000),
                            new Pose(122.000, 60.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(122.000, 60.000),
                            new Pose(84.000, 82.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))
                    .build();

            pathTOintake1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(84.000, 82.000),
                            new Pose(98.000, 64.000),
                            new Pose(123.000, 65.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(-40), Math.toRadians(0))
                    .build();

            intake2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(123.000, 65.000),
                            new Pose(129, 60)
                    ))
                    .setLinearHeadingInterpolation(

                            Math.toRadians(-35),
                            Math.toRadians(39)

                    )
                    .build();

            shoot3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(124.000, 56.000),
                            new Pose(93.000, 64.500),
                            new Pose(84.000, 82.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30))
                    .build();

        }
    }
}