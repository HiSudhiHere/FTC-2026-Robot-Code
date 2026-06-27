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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Autonomous(name = "BlueUp", group = "Autonomous")
@Configurable
public class BlueUp extends OpMode {

    //========================================
    // TELEMETRY
    //========================================

    private TelemetryManager panelsTelemetry;

    //========================================
    // PEDRO
    //========================================

    private Follower follower;
    private Paths paths;

    //========================================
    // SUBSYSTEMS
    //========================================

    private static ShooterSubsystem shooter;
    private static IntakeSubsystem intake;
    private static ServoSubsystem servos;
    private static TurretSubsystem turret;

    //========================================
    // TIMERS
    //========================================

    private ElapsedTime actionTimer = new ElapsedTime();

    //========================================
    // STATE MACHINE
    //========================================

    private int pathState = 0;

    //========================================
    // CONSTANTS
    //========================================

    private static final double STOPPER_OPEN = 0.60;
    private static final double STOPPER_CLOSE = 0.30;

    private static final int INTAKE_WAIT = 1100;
    private static final int SHOOTER_WAIT = 200;
    private static final int FEED_TIME = 900;

    //========================================
    // INIT
    //========================================

    @Override
    public void init() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        // BLUE START POSITION
        follower.setStartingPose(
                new Pose(
                        31.5,
                        131,
                        Math.toRadians(90)
                )
        );

        follower.setMaxPower(0.90);

        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        servos = new ServoSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);

        servos.setStopper(STOPPER_CLOSE);

        servos.setHudder(0.40);

        // Change if your blue turret angle is different
        turret.setFieldAngle(-65);

        paths = new Paths(follower);

        telemetry.addLine("Blue Auto Initialized");
        telemetry.update();
    }

    //========================================
    // LOOP
    //========================================

    @Override
    public void loop() {

        follower.update();

        turret.update(
                Math.toDegrees(
                        follower.getPose().getHeading()
                )
        );

        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("Shooter", shooter.getAverageVelocity());
        telemetry.addData("Turret", turret.getPosition());

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading",
                Math.toDegrees(
                        follower.getPose().getHeading()));

        telemetry.update();
    }

    //========================================
    // FOLLOW PATH
    //========================================

    private void follow(PathChain path, int nextState) {

        follower.followPath(path);

        pathState = nextState;
    }

    //========================================
    // STOP ROBOT
    //========================================

    private void stopRobot() {

        shooter.stop();

        intake.stop();

        turret.stop();

        servos.setStopper(STOPPER_CLOSE);
    }
    private void autonomousPathUpdate() {

        switch (pathState) {

            //========================================
            // PRELOAD SHOOT
            //========================================

            case 0:

                shooter.ShortVelocity();
                follow(paths.shoot1, 1);
                break;

            case 1:

                if (!follower.isBusy()) {

                    servos.setStopper(STOPPER_OPEN);
                    intake.intakeIn();

                    actionTimer.reset();
                    pathState = 2;
                }

                break;

            case 2:

                if (actionTimer.milliseconds() >= FEED_TIME) {

                    servos.setStopper(STOPPER_CLOSE);

                    intake.stop();
                    shooter.stop();

                    follow(paths.intake1, 3);
                }

                break;

            //========================================
            // INTAKE 1 (NO WAIT)
            //========================================

            case 3:

                if (!follower.isBusy()) {

                    shooter.ShortVelocity();

                    follow(paths.shoot2, 4);
                }

                break;

            //========================================
            // SHOOT 2
            //========================================

            case 4:

                if (!follower.isBusy()) {

                    actionTimer.reset();

                    pathState = 5;
                }

                break;

            // Shooter Stabilize

            case 5:

                if (actionTimer.milliseconds() >= SHOOTER_WAIT) {

                    servos.setStopper(STOPPER_OPEN);

                    intake.intakeIn();

                    actionTimer.reset();

                    pathState = 6;
                }

                break;

            // Feed

            case 6:

                if (actionTimer.milliseconds() >= FEED_TIME) {

                    servos.setStopper(STOPPER_CLOSE);

                    intake.stop();

                    shooter.stop();

                    follow(paths.intake2, 7);
                }

                break;

            //========================================
            // INTAKE 2
            //========================================

            case 7:

                if (!follower.isBusy()) {

                    intake.intakeIn();

                    actionTimer.reset();

                    pathState = 8;
                }

                break;

            // Wait for Intake

            case 8:

                if (actionTimer.milliseconds() >= INTAKE_WAIT) {

                    shooter.ShortVelocity();

                    follow(paths.shoot3, 9);
                }

                break;

            //========================================
            // SHOOT 3
            //========================================

            case 9:

                if (!follower.isBusy()) {

                    actionTimer.reset();

                    pathState = 10;
                }

                break;

            // Shooter Stabilize

            case 10:

                if (actionTimer.milliseconds() >= SHOOTER_WAIT) {

                    servos.setStopper(STOPPER_OPEN);

                    intake.intakeIn();

                    actionTimer.reset();

                    pathState = 11;
                }

                break;

            // Feed

            case 11:

                if (actionTimer.milliseconds() >= FEED_TIME) {

                    servos.setStopper(STOPPER_CLOSE);

                    intake.stop();

                    shooter.stop();

                    follow(paths.intake3, 12);
                }

                break;

            //========================================
            // INTAKE 3
            //========================================

            case 12:

                if (!follower.isBusy()) {

                    intake.intakeIn();

                    actionTimer.reset();

                    pathState = 13;
                }

                break;

            // Wait for Intake

            case 13:

                if (actionTimer.milliseconds() >= INTAKE_WAIT) {

                    shooter.ShortVelocity();

                    follow(paths.shoot4, 14);
                }

                break;
            //========================================
            // SHOOT 4
            //========================================

            case 14:

                if (!follower.isBusy()) {

                    actionTimer.reset();

                    pathState = 15;
                }

                break;

            // Shooter Stabilize

            case 15:

                if (actionTimer.milliseconds() >= SHOOTER_WAIT) {

                    servos.setStopper(STOPPER_OPEN);

                    intake.intakeIn();

                    actionTimer.reset();

                    pathState = 16;
                }

                break;

            // Feed Balls

            case 16:

                if (actionTimer.milliseconds() >= FEED_TIME) {

                    servos.setStopper(STOPPER_CLOSE);

                    intake.stop();

                    shooter.stop();

                    follow(paths.intake4, 17);
                }

                break;

            //========================================
            // INTAKE 4
            //========================================

            case 17:

                if (!follower.isBusy()) {

                    intake.intakeIn();

                    actionTimer.reset();

                    pathState = 18;
                }

                break;

            // Wait at Intake

            case 18:

                if (actionTimer.milliseconds() >= INTAKE_WAIT) {

                    shooter.ShortVelocity();

                    follow(paths.shoot5, 19);
                }

                break;

            //========================================
            // SHOOT 5
            //========================================

            case 19:

                if (!follower.isBusy()) {

                    actionTimer.reset();

                    pathState = 20;
                }

                break;

            // Shooter Stabilize

            case 20:

                if (actionTimer.milliseconds() >= SHOOTER_WAIT) {

                    servos.setStopper(STOPPER_OPEN);

                    intake.intakeIn();

                    actionTimer.reset();

                    pathState = 21;
                }

                break;

            // Feed Balls

            case 21:

                if (actionTimer.milliseconds() >= FEED_TIME) {

                    servos.setStopper(STOPPER_CLOSE);

                    intake.stop();

                    shooter.stop();

                    follow(paths.intake5, 22);
                }

                break;

            //========================================
            // INTAKE 5
            //========================================

            case 22:

                if (!follower.isBusy()) {

                    intake.intakeIn();

                    actionTimer.reset();

                    pathState = 23;
                }

                break;

            // Wait at Intake

            case 23:

                if (actionTimer.milliseconds() >= INTAKE_WAIT) {

                    shooter.ShortVelocity();

                    follow(paths.shoot6, 24);
                }

                break;

            //========================================
            // SHOOT 6
            //========================================

            case 24:

                if (!follower.isBusy()) {

                    actionTimer.reset();

                    pathState = 25;
                }

                break;

            // Shooter Stabilize

            case 25:

                if (actionTimer.milliseconds() >= SHOOTER_WAIT) {

                    servos.setStopper(STOPPER_OPEN);

                    intake.intakeIn();

                    actionTimer.reset();

                    pathState = 26;
                }

                break;

            // Feed Balls

            case 26:

                if (actionTimer.milliseconds() >= FEED_TIME) {

                    servos.setStopper(STOPPER_CLOSE);

                    intake.stop();

                    shooter.stop();

                    stopRobot();

                    pathState = 27;
                }

                break;

            //========================================
            // END
            //========================================

            case 27:

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
        public PathChain intake4;
        public PathChain shoot5;
        public PathChain intake5;
        public PathChain shoot6;

        public Paths(Follower follower) {

            //========================================
            // SHOOT 1
            //========================================

            shoot1 = follower.pathBuilder()

                    .addPath(
                            new BezierLine(
                                    new Pose(31.500, 131.000),
                                    new Pose(47.500, 60.000)
                            )
                    )

                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(90)
                    )

                    .addParametricCallback(0.01, () -> {
                        turret.setFieldAngle(180-65);
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

                    .build();


            //========================================
            // INTAKE 1
            //========================================

            intake1 = follower.pathBuilder()

                    .addPath(
                            new BezierLine(
                                    new Pose(47.500, 60.000),
                                    new Pose(20.000, 57.500)
                            )
                    )

                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(180)
                    )

                    .addParametricCallback(0.05, () -> {

                        servos.setStopper(STOPPER_CLOSE);
                        //turret.setFieldAngle(65);

                        intake.intakeIn();

                    })

                    .build();


            //========================================
            // SHOOT 2
            //========================================

            shoot2 = follower.pathBuilder()

                    .addPath(
                            new BezierLine(
                                    new Pose(20.000, 57.500),
                                    new Pose(53.500, 77.000)
                            )
                    )

                    .setLinearHeadingInterpolation(
                            Math.toRadians(215),
                            Math.toRadians(215)
                    )

                    .addParametricCallback(0.1, () -> {
                        turret.setFieldAngle(180-55);
                    })

                    .addParametricCallback(0.90, () -> {

                        intake.stop();

                    })

                    .build();


            //========================================
            // INTAKE 2
            //========================================

            intake2 = follower.pathBuilder()

                    .addPath(
                            new BezierCurve(
                                    new Pose(53.500, 77.000),
                                    new Pose(32.500, 60.000),
                                    new Pose(9.500, 60.000)
                            )
                    )

                    .setLinearHeadingInterpolation(
                            Math.toRadians(218),
                            Math.toRadians(143)
                    )

                    .addParametricCallback(0.05, () -> {

                        intake.intakeIn();

                        shooter.stop();

                    })

                    .addParametricCallback(0.75, () -> {

                        shooter.ShortVelocity();

                    })

                    .build();


            //========================================
            // SHOOT 3
            //========================================

            shoot3 = follower.pathBuilder()

                    .addPath(
                            new BezierLine(
                                    new Pose(9.500, 60.000),
                                    new Pose(53.500, 77.000)
                            )
                    )

                    .setLinearHeadingInterpolation(
                            Math.toRadians(215),
                            Math.toRadians(215)
                    )

                    .addParametricCallback(0.1, () -> {
                        turret.setFieldAngle(180-55);
                    })

                    .addParametricCallback(0.90, () -> {

                        intake.stop();

                    })

                    .build();


            //========================================
            // INTAKE 3
            //========================================

            intake3 = follower.pathBuilder()

                    .addPath(
                            new BezierCurve(
                                    new Pose(53.500, 77.000),
                                    new Pose(32.500, 60.000),
                                    new Pose(9.500, 57.000)
                            )
                    )

                    .setLinearHeadingInterpolation(
                            Math.toRadians(215),
                            Math.toRadians(145)
                    )
                    .addParametricCallback(0.05, () -> {

                        intake.intakeIn();

                        shooter.stop();

                    })

                    .addParametricCallback(0.75, () -> {

                        shooter.ShortVelocity();

                    })

                    .build();
            //========================================
            // SHOOT 4
            //========================================

            shoot4 = follower.pathBuilder()

                    .addPath(
                            new BezierLine(
                                    new Pose(9.500, 57.000),
                                    new Pose(53.500, 77.000)
                            )
                    )

                    .setLinearHeadingInterpolation(
                            Math.toRadians(215),
                            Math.toRadians(215)
                    )

                    .addParametricCallback(0.1, () -> {
                        turret.setFieldAngle(180-55);
                    })

                    .addParametricCallback(0.90, () -> {

                        intake.stop();

                    })

                    .build();


            //========================================
            // INTAKE 4
            //========================================

            intake4 = follower.pathBuilder()

                    .addPath(
                            new BezierCurve(
                                    new Pose(53.500, 77.000),
                                    new Pose(32.500, 60.000),
                                    new Pose(9.500, 57.000)
                            )
                    )

                    .setLinearHeadingInterpolation(
                            Math.toRadians(215),
                            Math.toRadians(145)
                    )

                    .addParametricCallback(0.05, () -> {

                        intake.intakeIn();

                        shooter.stop();

                    })

                    .addParametricCallback(0.75, () -> {

                        shooter.ShortVelocity();

                    })

                    .build();


            //========================================
            // SHOOT 5
            //========================================

            shoot5 = follower.pathBuilder()

                    .addPath(
                            new BezierLine(
                                    new Pose(9.500, 57.000),
                                    new Pose(53.500, 77.000)
                            )
                    )

                    .setLinearHeadingInterpolation(
                            Math.toRadians(215),
                            Math.toRadians(215)
                    )

                    .addParametricCallback(0.1, () -> {
                        turret.setFieldAngle(180-55);
                    })

                    .addParametricCallback(0.90, () -> {

                        intake.stop();

                    })

                    .build();


            //========================================
            // INTAKE 5
            //========================================

            intake5 = follower.pathBuilder()

                    .addPath(
                            new BezierCurve(
                                    new Pose(53.500, 77.000),
                                    new Pose(33.432, 80.521),
                                    new Pose(17.599, 82.355)
                            )
                    )

                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(180)
                    )
                    .addParametricCallback(0.05, () -> {

                        intake.intakeIn();

                    })

                    .build();
            //========================================
            // SHOOT 6
            //========================================
            shoot6 = follower.pathBuilder()

                    .addPath(
                            new BezierLine(
                                    new Pose(17.599, 82.355),
                                    new Pose(53.305, 77.097)
                            )
                    )

                    .setLinearHeadingInterpolation(
                            Math.toRadians(180),
                            Math.toRadians(180)
                    )
                    .build();

        }
    }
}