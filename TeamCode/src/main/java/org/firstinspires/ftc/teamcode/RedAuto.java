package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Autonomous(name = "Red Auto 8 Shot", group = "Autonomous")
@Configurable
public class RedAuto extends OpMode {
    private static final Pose START = new Pose(103, 9, Math.toRadians(90));
    private static final Pose SHOOT = new Pose(95, 9, 0);
    private static final Pose STACK_SETUP = new Pose(97, 35, 0);
    private static final Pose STACK_HIGH = new Pose(126, 35, 0);
    private static final Pose NEAR_BALL = new Pose(132, 9, 0);
    private static final Pose NEAR_BALL_DEEP = new Pose(133, 9, 0);
    private static final Pose MID_BALL = new Pose(133, 16, 0);
    private static final Pose FAR_BALL = new Pose(133, 30, 0);

    private static final double STOPPER_OPEN = 0.6;
    private static final double STOPPER_CLOSED = 0.3;
    private static final double HUDDER_READY = 0.55;
    private static final double CHASSIS_MAX_POWER = 0.85;

    private static final double TURRET_KP = 0.01;
    private static final double TURRET_MIN_POWER = 0.05;
    private static final double TURRET_MAX_POWER = 0.20;
    private static final double TURRET_DEADZONE_DEGREES = 2.0;
    private static final double TURRET_FILTER_OLD_WEIGHT = 0.95;
    private static final double TURRET_FILTER_NEW_WEIGHT = 0.05;
    private static final double SHOOT_TIME_MS = 1000;

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Paths paths;

    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private ServoSubsystem servos;
    private DcMotorEx turret;
    private Limelight3A limelight;

    private final ElapsedTime shotTimer = new ElapsedTime();

    private State state = State.SHOOT_PRELOAD;
    private boolean activeShot = false;
    private double filteredTx = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(START);
        follower.setMaxPower(CHASSIS_MAX_POWER);
        paths = new Paths(follower);

        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        servos = new ServoSubsystem(hardwareMap);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();

        servos.setStopper(STOPPER_CLOSED);
        servos.setHudder(HUDDER_READY);

        panelsTelemetry.debug("Status", "Red auto initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        followPath(paths.preloadToShoot);
        shotTimer.reset();
    }

    @Override
    public void loop() {
        follower.update();
        turretLockToTag();
        updateStateMachine();
        sendTelemetry();
    }

    @Override
    public void stop() {
        intake.stop();
        shooter.stop();
        turret.setPower(0);
        servos.setStopper(STOPPER_CLOSED);
    }

    private void updateStateMachine() {
        switch (state) {
            case SHOOT_PRELOAD:
                shootWhenArrived(State.DRIVE_TO_STACK_HIGH);
                break;

            case DRIVE_TO_STACK_HIGH:
                drive(paths.driveToStackSetup, false, State.COLLECT_STACK_HIGH);
                break;

            case COLLECT_STACK_HIGH:
                drive(paths.collectStackHigh, true, State.RETURN_FROM_STACK_HIGH);
                break;

            case RETURN_FROM_STACK_HIGH:
                driveToShoot(paths.returnFromStackHigh, State.SHOOT_STACK_HIGH);
                break;

            case SHOOT_STACK_HIGH:
                shootWhenArrived(State.COLLECT_NEAR_1);
                break;

            case COLLECT_NEAR_1:
                drive(paths.collectNear1, true, State.RETURN_NEAR_1);
                break;

            case RETURN_NEAR_1:
                driveToShoot(paths.returnNear1, State.SHOOT_NEAR_1);
                break;

            case SHOOT_NEAR_1:
                shootWhenArrived(State.COLLECT_NEAR_2);
                break;

            case COLLECT_NEAR_2:
                drive(paths.collectNear2, true, State.RETURN_NEAR_2);
                break;

            case RETURN_NEAR_2:
                driveToShoot(paths.returnNear2, State.SHOOT_NEAR_2);
                break;

            case SHOOT_NEAR_2:
                shootWhenArrived(State.COLLECT_MID);
                break;

            case COLLECT_MID:
                drive(paths.collectMid, true, State.RETURN_MID);
                break;

            case RETURN_MID:
                driveToShoot(paths.returnMid, State.SHOOT_MID);
                break;

            case SHOOT_MID:
                shootWhenArrived(State.COLLECT_FAR);
                break;

            case COLLECT_FAR:
                drive(paths.collectFar, true, State.RETURN_FAR);
                break;

            case RETURN_FAR:
                driveToShoot(paths.returnFar, State.SHOOT_FAR);
                break;

            case SHOOT_FAR:
                shootWhenArrived(State.COLLECT_FAR_REPEAT);
                break;

            case COLLECT_FAR_REPEAT:
                drive(paths.collectFarRepeat, true, State.RETURN_FAR_REPEAT);
                break;

            case RETURN_FAR_REPEAT:
                driveToShoot(paths.returnFarRepeat, State.SHOOT_FAR_REPEAT);
                break;

            case SHOOT_FAR_REPEAT:
                shootWhenArrived(State.COLLECT_NEAR_REPEAT);
                break;

            case COLLECT_NEAR_REPEAT:
                drive(paths.collectNearRepeat, true, State.RETURN_NEAR_REPEAT);
                break;

            case RETURN_NEAR_REPEAT:
                driveToShoot(paths.returnNearRepeat, State.SHOOT_NEAR_REPEAT);
                break;

            case SHOOT_NEAR_REPEAT:
                shootWhenArrived(State.FINISHED);
                break;

            case FINISHED:
                stop();
                break;
        }
    }

    private void drive(PathChain path, boolean runIntake, State nextState) {
        if (!activeShot && !follower.isBusy()) {
            if (runIntake) {
                intake.intakeIn();
            } else {
                intake.stop();
            }

            followPath(path);
            state = nextState;
        }
    }

    private void driveToShoot(PathChain path, State nextState) {
        if (!activeShot && !follower.isBusy()) {
            shooter.shootFast();
            servos.setStopper(STOPPER_CLOSED);
            intake.stop();

            followPath(path);
            state = nextState;
        }
    }

    private void followPath(PathChain path) {
        follower.followPath(path, CHASSIS_MAX_POWER, true);
    }

    private void shootWhenArrived(State nextState) {
        if (follower.isBusy()) {
            shooter.shootFast();
            servos.setStopper(STOPPER_CLOSED);
            return;
        }

        shooter.shootFast();

        if (!shooter.readyForFastShot()) {
            activeShot = false;
            shotTimer.reset();
            servos.setStopper(STOPPER_CLOSED);
            intake.stop();
            return;
        }

        if (!activeShot) {
            activeShot = true;
            shotTimer.reset();
        }

        servos.setStopper(STOPPER_OPEN);
        intake.intakeIn();

        if (shotTimer.milliseconds() >= SHOOT_TIME_MS) {
            activeShot = false;
            servos.setStopper(STOPPER_CLOSED);
            intake.stop();
            state = nextState;
        }
    }

    private void turretLockToTag() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            turret.setPower(0);
            return;
        }

        double tx = result.getTx();
        filteredTx = filteredTx * TURRET_FILTER_OLD_WEIGHT + tx * TURRET_FILTER_NEW_WEIGHT;

        if (Math.abs(filteredTx) <= TURRET_DEADZONE_DEGREES) {
            turret.setPower(0);
            return;
        }

        double turretPower = -filteredTx * TURRET_KP;

        if (Math.abs(turretPower) < TURRET_MIN_POWER) {
            turretPower = Math.signum(turretPower) * TURRET_MIN_POWER;
        }

        turretPower = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, turretPower));
        turret.setPower(turretPower);
    }

    private void sendTelemetry() {
        panelsTelemetry.debug("State", state);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addData("State", state);
        telemetry.addData("Turret Position", turret.getCurrentPosition());
        telemetry.addData("Shooter L", "%.1f", shooter.getLeftVelocity());
        telemetry.addData("Shooter R", "%.1f", shooter.getRightVelocity());
        telemetry.addData("Intake", "%.2f", intake.getPower());

        panelsTelemetry.update(telemetry);
    }

    private enum State {
        SHOOT_PRELOAD,
        DRIVE_TO_STACK_HIGH,
        COLLECT_STACK_HIGH,
        RETURN_FROM_STACK_HIGH,
        SHOOT_STACK_HIGH,
        COLLECT_NEAR_1,
        RETURN_NEAR_1,
        SHOOT_NEAR_1,
        COLLECT_NEAR_2,
        RETURN_NEAR_2,
        SHOOT_NEAR_2,
        COLLECT_MID,
        RETURN_MID,
        SHOOT_MID,
        COLLECT_FAR,
        RETURN_FAR,
        SHOOT_FAR,
        COLLECT_FAR_REPEAT,
        RETURN_FAR_REPEAT,
        SHOOT_FAR_REPEAT,
        COLLECT_NEAR_REPEAT,
        RETURN_NEAR_REPEAT,
        SHOOT_NEAR_REPEAT,
        FINISHED
    }

    private static class Paths {
        private final PathChain preloadToShoot;
        private final PathChain driveToStackSetup;
        private final PathChain collectStackHigh;
        private final PathChain returnFromStackHigh;
        private final PathChain collectNear1;
        private final PathChain returnNear1;
        private final PathChain collectNear2;
        private final PathChain returnNear2;
        private final PathChain collectMid;
        private final PathChain returnMid;
        private final PathChain collectFar;
        private final PathChain returnFar;
        private final PathChain collectFarRepeat;
        private final PathChain returnFarRepeat;
        private final PathChain collectNearRepeat;
        private final PathChain returnNearRepeat;

        private Paths(Follower follower) {
            preloadToShoot = line(follower, START, SHOOT);
            driveToStackSetup = line(follower, SHOOT, STACK_SETUP);
            collectStackHigh = line(follower, STACK_SETUP, STACK_HIGH);
            returnFromStackHigh = line(follower, STACK_HIGH, SHOOT);
            collectNear1 = line(follower, SHOOT, NEAR_BALL);
            returnNear1 = line(follower, NEAR_BALL, SHOOT);
            collectNear2 = curve(follower, SHOOT, new Pose(116, 12, 0), NEAR_BALL_DEEP);
            returnNear2 = line(follower, NEAR_BALL_DEEP, SHOOT);
            collectMid = curve(follower, SHOOT, new Pose(100, 19, 0), MID_BALL);
            returnMid = line(follower, MID_BALL, SHOOT);
            collectFar = curve(follower, SHOOT, new Pose(101, 30, 0), FAR_BALL);
            returnFar = line(follower, FAR_BALL, SHOOT);
            collectFarRepeat = line(follower, SHOOT, FAR_BALL);
            returnFarRepeat = line(follower, FAR_BALL, SHOOT);
            collectNearRepeat = line(follower, SHOOT, NEAR_BALL_DEEP);
            returnNearRepeat = line(follower, NEAR_BALL_DEEP, SHOOT);
        }

        private PathChain line(Follower follower, Pose start, Pose end) {
            return follower.pathBuilder()
                    .addPath(new BezierLine(start, end))
                    .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                    .build();
        }

        private PathChain curve(Follower follower, Pose start, Pose control, Pose end) {
            return follower.pathBuilder()
                    .addPath(new BezierCurve(start, control, end))
                    .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                    .build();
        }
    }
}