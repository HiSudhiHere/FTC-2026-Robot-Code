package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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

@Autonomous(name = "BlueUp7Test2", group = "Autonomous")
@Configurable
public class BlueUp7Test2 extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;
    private Paths paths;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private ServoSubsystem servos;
    private TurretSubsystem turret;

    private ElapsedTime waitTimer = new ElapsedTime();
    private static int value = 2;
    private static final double STOPPER_OPEN = 0.6;
    private static final double STOPPER_CLOSED = 0.3;
    private static final double SHOOT_TIME =900;
    private static final double OPEN_WAIT_TIME = 200;
    private static final double INTAKE_WAIT_TIME = 800;

    private Limelight3A limelight;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(107.000, 54.000, Math.toRadians(142)));

        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        servos = new ServoSubsystem(hardwareMap);

        turret = new TurretSubsystem(hardwareMap);
        turret.setFieldAngle(135);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(20);
        limelight.start();

        servos.setStopper(STOPPER_CLOSED);
        servos.setHudder(0.12);

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
        telemetry.addData("Average Velocity", "%.1f", shooter.getAverageVelocity());
        telemetry.addData("Turret Position", turret.getPosition());
        telemetry.update();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    private void startShootPath(PathChain path, int nextState) {
        shooter.shootSlow();
        intake.stop();
        servos.setStopper(STOPPER_CLOSED);

        follower.followPath(path);
        pathState = nextState;
    }

    private void runShootSequence(int nextState) {
        shooter.shootSlow();
        servos.setStopper(STOPPER_OPEN);

        // If this is physically opposite, change intakeOut() to intakeIn()
        intake.intakeOut();

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

        // If this is physically opposite, change intakeOut() to intakeIn()
        intake.intakeOut();

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
        intake.intakeOut();

        if (waitTimer.milliseconds() >= INTAKE_WAIT_TIME) {
            intake.stop();
            pathState = nextState;
        }
    }

    private void waitWithIntakeOff(int nextState) {
        shooter.stop();
        intake.stop();
        servos.setStopper(STOPPER_CLOSED);

        if (waitTimer.milliseconds() >= OPEN_WAIT_TIME) {
            pathState = nextState;
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                startShootPath(paths.shoot1, 1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    waitTimer.reset();
                    pathState = 2;
                }
                break;

            case 2:
                runShootSequence(3);
                break;

            /**case 3:
             startIntakePath(paths.intake1, 4);
             break;

             case 4:
             if (!follower.isBusy()) {
             startOpenPath(paths.open1, 5);
             }
             break;

             case 5:
             if (!follower.isBusy()) {
             waitTimer.reset();
             pathState = 6;
             }
             break;

             case 6:
             waitWithIntakeOff(7);
             break;

             case 7:
             startShootPath(paths.shoot2, 8);
             break;

             case 8:
             if (!follower.isBusy()) {
             waitTimer.reset();
             pathState = 9;
             }
             break;

             case 9:
             runShootSequence(10);
             break;

             case 10:
             startIntakePath(paths.intake2, 11);
             break;

             case 11:
             if (!follower.isBusy()) {
             startOpenPath(paths.open2, 12);
             }
             break;

             case 12:
             if (!follower.isBusy()) {
             waitTimer.reset();
             pathState = 13;
             }
             break;

             case 13:
             waitWithIntakeOff(14);
             break;

             case 14:
             startShootPath(paths.shoot3, 15);
             break;

             case 15:
             if (!follower.isBusy()) {
             waitTimer.reset();
             pathState = 16;
             }
             break;

             case 16:
             runShootSequence(17);
             break;

             case 17:
             startPathToIntake(paths.pathTOintake1, 18);
             break;

             case 18:
             if (!follower.isBusy()) {
             startIntakePath(paths.intake3, 19);
             }
             break;

             case 19:
             if (!follower.isBusy()) {
             waitTimer.reset();
             pathState = 20;
             }
             break;

             case 20:
             waitWithIntakeOn(21);
             break;

             case 21:
             startShootPath(paths.shoot4, 22);
             break;

             case 22:
             if (!follower.isBusy()) {
             waitTimer.reset();
             pathState = 23;
             }
             break;

             case 23:
             runShootSequence(24);
             break;

             case 24:
             startPathToIntake(paths.pathTOintake2, 25);
             break;

             case 25:
             if (!follower.isBusy()) {
             startIntakePath(paths.intake4, 26);
             }
             break;

             case 26:
             if (!follower.isBusy()) {
             waitTimer.reset();
             pathState = 27;
             }
             break;

             case 27:
             waitWithIntakeOn(28);
             break;

             case 28:
             startShootPath(paths.shoot5, 29);
             break;

             case 29:
             if (!follower.isBusy()) {
             waitTimer.reset();
             pathState = 30;
             }
             break;

             case 30:
             runShootSequence(31);
             break;

             case 31:
             startPathToIntake(paths.pathTOintake3, 32);
             break;

             case 32:
             if (!follower.isBusy()) {
             startIntakePath(paths.intake5, 33);
             }
             break;

             case 33:
             if (!follower.isBusy()) {
             waitTimer.reset();
             pathState = 34;
             }
             break;

             case 34:
             waitWithIntakeOn(35);
             break;

             case 35:
             startShootPath(paths.shoot6, 36);
             break;

             case 36:
             if (!follower.isBusy()) {
             waitTimer.reset();
             pathState = 37;
             }
             break;

             case 37:
             runShootSequence(38);
             break;

             case 38:
             startPathToIntake(paths.pathTOintake4, 39);
             break;

             case 39:
             if (!follower.isBusy()) {
             startIntakePath(paths.intake6, 40);
             }
             break;

             case 40:
             if (!follower.isBusy()) {
             waitTimer.reset();
             pathState = 41;
             }
             break;

             case 41:
             waitWithIntakeOn(42);
             break;

             case 42:
             startShootPath(paths.shoot7, 43);
             break;

             case 43:
             if (!follower.isBusy()) {
             waitTimer.reset();
             pathState = 44;
             }
             break;

             case 44:
             runShootSequence(45);
             break;

             case 45:
             intake.stop();
             shooter.stop();
             turret.stop();
             servos.setStopper(STOPPER_CLOSED);
             break;**/
        }
    }

    public static class Paths {
        public PathChain shoot1;
        public PathChain intake1;
        public PathChain open1;
        public PathChain shoot2;
        public PathChain intake2;
        public PathChain open2;
        public PathChain shoot3;
        public PathChain pathTOintake1;
        public PathChain intake3;
        public PathChain shoot4;
        public PathChain pathTOintake2;
        public PathChain intake4;
        public PathChain shoot5;
        public PathChain pathTOintake3;
        public PathChain intake5;
        public PathChain shoot6;
        public PathChain pathTOintake4;
        public PathChain intake6;
        public PathChain shoot7;

        public Paths(Follower follower) {
            shoot1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(107.000, 54.000),
                                    new Pose(58.500, 82.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(142))
                    .build();

            intake1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(58.500, 82.000),
                                    new Pose(15.500, 82.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(175), Math.toRadians(180))
                    .build();

            open1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(15.500, 82.000),
                                    new Pose(21.500, 78.000),
                                    new Pose(15.500, 74.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(15.500, 74.000),
                                    new Pose(57.500, 82.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(240))
                    .build();

            intake2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(57.500, 82.000),
                                    new Pose(57.500, 56.000),
                                    new Pose(15.500, 58.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(240), Math.toRadians(180))
                    .build();

            open2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(15.500, 58.000),
                                    new Pose(20.500, 61.000),
                                    new Pose(15.500, 63.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shoot3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(15.500, 63.000),
                                    new Pose(57.500, 82.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(220))
                    .setReversed()
                    .build();

            pathTOintake1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(57.500, 82.000),
                                    new Pose(43.500, 64.000),
                                    new Pose(18.500, 64.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(180))
                    .build();

            intake3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(18.500, 64.000),
                                    new Pose(13.500, 56.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                    .build();

            shoot4 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(13.500, 56.000),
                                    new Pose(48.688, 64.500),
                                    new Pose(57.500, 82.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(220))
                    .build();

            pathTOintake2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(57.500, 82.000),
                                    new Pose(43.500, 64.000),
                                    new Pose(18.500, 64.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(180))
                    .build();

            intake4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(18.500, 64.000),
                                    new Pose(13.500, 56.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                    .build();

            shoot5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(13.500, 56.000),
                                    new Pose(57.500, 82.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(220))
                    .build();

            pathTOintake3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(57.500, 82.000),
                                    new Pose(43.500, 64.000),
                                    new Pose(18.500, 64.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(180))
                    .build();

            intake5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(18.500, 64.000),
                                    new Pose(13.500, 56.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                    .build();

            shoot6 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(13.500, 56.000),
                                    new Pose(57.500, 82.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(220))
                    .build();

            pathTOintake4 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(57.500, 82.000),
                                    new Pose(43.500, 64.000),
                                    new Pose(18.500, 64.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(220), Math.toRadians(180))
                    .build();

            intake6 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(18.500, 64.000),
                                    new Pose(13.500, 56.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                    .build();

            shoot7 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(13.500, 56.000),
                                    new Pose(57.500, 82.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(220))
                    .build();
        }
    }
}