package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.Constants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.Path;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Autonomous(name = "RedUp", group = "Auto")
@Configurable
public class RedUp extends OpMode {

    //-------------------------------
    // Hardware
    //-------------------------------

    private static Follower follower;

    private static ShooterSubsystem shooter;
    private static IntakeSubsystem intake;
    private static ServoSubsystem servos;
    private static TurretSubsystem turret;
    private Limelight3A limelight;

    private boolean limelightTracking = false;

    private static final double LIMELIGHT_KP = 0.08;


    //-------------------------------
    // Paths
    //-------------------------------

    private Paths paths;

    private double predictiveBrakingStrength = 1.0;
    //-------------------------------
    // State Machine
    //-------------------------------

    private int pathState = 0;

    private static final double SHOOTER_READY = 1280;

    //-------------------------------
    // Timer
    //-------------------------------

    private final ElapsedTime actionTimer = new ElapsedTime();

    //-------------------------------
    // Servo Positions
    //-------------------------------

    private static final double STOPPER_OPEN = 0.60;
    private static final double STOPPER_CLOSE = 0.30;

    //-------------------------------
    // Timing
    //-------------------------------

    private static final int INTAKE_WAIT = 1100;
    private static final int SHOOTER_WAIT = 200;
    private static final int FEED_TIME = 1000;

    //-------------------------------
    // Init
    //-------------------------------

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(
                new Pose(
                        110,
                        131,
                        Math.toRadians(90)
                ));

        follower.setMaxPower(0.90);

        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        servos = new ServoSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

        servos.setHudder(0.40);
        servos.setStopper(STOPPER_CLOSE);

        // turret.setFieldAngle(65);

        paths = new Paths(follower);

        telemetry.addLine("Initialized");
        telemetry.update();
    }
    public void setPredictiveBraking(double strength) {
        predictiveBrakingStrength = Math.max(0.0, Math.min(1.0, strength));
    }

    //-------------------------------
    // Loop
    //-------------------------------

    @Override
    public void loop() {

        follower.update();

        turret.update(
                Math.toDegrees(
                        follower.getPose().getHeading()
                )
        );

        if (limelightTracking) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                double correction = result.getTx() * LIMELIGHT_KP;

                turret.setFieldAngle(
                        turret.getLockedFieldAngle() - correction
                );
            }
        }


        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading",
                Math.toDegrees(
                        follower.getPose().getHeading()));

        telemetry.addData(
                "Shooter",
                shooter.getAverageVelocity());

        telemetry.update();
    }

    //-------------------------------
    // Helper Functions
    //-------------------------------

    private void follow(PathChain path, int nextState) {

        follower.followPath(path);
        pathState = nextState;
    }

    private void shoot() {

        shooter.ShortVelocity();

        if (shooter.getAverageVelocity() >= 1280) {

            servos.setStopper(STOPPER_OPEN);

            intake.intakeIn();
        }
    }

    private void intake() {

        shooter.stop();

        servos.setStopper(STOPPER_CLOSE);

        intake.intakeIn();
    }

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
                if (!follower.isBusy()) {

                    // Turret is already pointing approximately at the goal.
                    limelightTracking = true;

                    actionTimer.reset();
                    pathState = 5;
                }
                follow(paths.shoot1, 1);

                break;

            case 1:

                if (!follower.isBusy()) {
                    actionTimer.reset();
                    pathState = 2;
                }

                break;

            // Feed preload
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

                    // Robot reached shoot position

                    actionTimer.reset();

                    pathState = 5;
                }

                break;

            // Shooter Stabilize (300ms)

            case 5:

                if (actionTimer.milliseconds() >= SHOOTER_WAIT) {

                    servos.setStopper(STOPPER_OPEN);

                    intake.intakeIn();

                    actionTimer.reset();

                    pathState = 6;
                }

                break;

            // Feed Balls (700ms)

            case 6:

                if (actionTimer.milliseconds() >= FEED_TIME) {

                    servos.setStopper(STOPPER_CLOSE);

                    intake.stop();

                    shooter.stop();

                    follow(paths.intake2,7);

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

            // Wait 1 sec at intake

            case 8:

                if (actionTimer.milliseconds() >= INTAKE_WAIT) {

                    shooter.ShortVelocity();

                    follow(paths.shoot3,9);

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

            // Shooter Stabilize (300ms)

            case 10:

                if (actionTimer.milliseconds() >= SHOOTER_WAIT) {

                    servos.setStopper(STOPPER_OPEN);

                    intake.intakeIn();

                    actionTimer.reset();

                    pathState = 11;
                }

                break;

            // Feed Balls (700ms)

            case 11:

                if (actionTimer.milliseconds() >= FEED_TIME) {

                    servos.setStopper(STOPPER_CLOSE);

                    intake.stop();

                    shooter.stop();

                    follow(paths.intake3,12);

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

            // Wait 1 sec at intake

            case 13:

                if (actionTimer.milliseconds() >= INTAKE_WAIT) {

                    shooter.ShortVelocity();

                    follow(paths.shoot4,14);

                }

                break;
            //========================================
            // SHOOT 4
            //========================================

            case 14:

                if (!follower.isBusy()) {

                    // Reached shooting position

                    actionTimer.reset();

                    pathState = 15;
                }

                break;

            // Shooter Stabilize (300 ms)

            case 15:

                if (actionTimer.milliseconds() >= SHOOTER_WAIT) {

                    servos.setStopper(STOPPER_OPEN);

                    intake.intakeIn();

                    actionTimer.reset();

                    pathState = 16;
                }

                break;

            // Feed Balls (700 ms)

            case 16:

                if (actionTimer.milliseconds() >= FEED_TIME) {

                    servos.setStopper(STOPPER_CLOSE);

                    intake.stop();

                    shooter.stop();

                    follow(paths.intake4,17);

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

            // Wait 1 second at intake

            case 18:

                if (actionTimer.milliseconds() >= INTAKE_WAIT) {

                    shooter.ShortVelocity();

                    follow(paths.shoot5,19);

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

            // Shooter Stabilize (300 ms)

            case 20:

                if (actionTimer.milliseconds() >= SHOOTER_WAIT) {

                    servos.setStopper(STOPPER_OPEN);

                    intake.intakeIn();

                    actionTimer.reset();

                    pathState = 21;
                }

                break;

            // Feed Balls (700 ms)

            case 21:

                if (actionTimer.milliseconds() >= FEED_TIME) {

                    servos.setStopper(STOPPER_CLOSE);

                    intake.stop();

                    shooter.stop();

                    follow(paths.intake5,22);

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

            // Wait 1 second for intake

            //========================================
            // END
            //========================================

            //========================================
            // INTAKE 5 WAIT
            //========================================

            case 23:

                if (actionTimer.milliseconds() >= 100) {

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

            // Shooter Stabilize (300 ms)

            case 25:

                if (actionTimer.milliseconds() >= SHOOTER_WAIT) {

                    servos.setStopper(STOPPER_OPEN);

                    intake.intakeIn();

                    actionTimer.reset();

                    pathState = 26;
                }

                break;

            // Feed Balls (700 ms)

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

                // Autonomous complete
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

            //---------------------------------------------------
            // SHOOT 1
            //---------------------------------------------------

            shoot1 = follower.pathBuilder()

                    .addPath(new BezierLine(
                            new Pose(110, 131),
                            new Pose(94, 60)
                    ))

                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(90))

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
                    .build();

            //---------------------------------------------------
            // INTAKE 1
            //---------------------------------------------------

            intake1 = follower.pathBuilder()

                    .addPath(new BezierLine(

                            new Pose(94, 60),
                            new Pose(121, 60)

                    ))

                    .setLinearHeadingInterpolation(
                            Math.toRadians(0),
                            Math.toRadians(0))

                    .addParametricCallback(0.05, () -> {

                        servos.setStopper(STOPPER_CLOSE);
                        //turret.setFieldAngle(65);
                        follower.setMaxPower(0.85);
                        intake.intakeIn();

                    })

                    .addParametricCallback(0.75, () -> {
                        shooter.ShortVelocity();
                    })
                    .addParametricCallback(0.9, () -> {
                        follower.setMaxPower(0.6);
                    })

                    .build();

            //---------------------------------------------------
            // SHOOT 2
            //---------------------------------------------------

            shoot2 = follower.pathBuilder()

                    .addPath(new BezierLine(

                            new Pose(121, 60),
                            new Pose(87.5, 77)

                    ))

                    .setLinearHeadingInterpolation(

                            Math.toRadians(-35),
                            Math.toRadians(-35)

                    )
                    .addParametricCallback(0.1, () -> {
                        follower.setMaxPower(0.85);
                        turret.setFieldAngle(55);
                    })

                    .addParametricCallback(0.90, () -> {

                        intake.stop();

                    })
                    .addParametricCallback(0.9, () -> {
                        follower.setMaxPower(0.6);
                    })

                    .build();

            //---------------------------------------------------
            // INTAKE 2
            //---------------------------------------------------

            intake2 = follower.pathBuilder()

                    .addPath(new BezierCurve(

                            new Pose(87.5, 77),
                            new Pose(109, 60),
                            new Pose(130, 57)

                    ))

                    .setLinearHeadingInterpolation(

                            Math.toRadians(-35),
                            Math.toRadians(37)

                    )

                    .addParametricCallback(0.05, () -> {

                        intake.intakeIn();
                        follower.setMaxPower(0.85);
                        shooter.stop();
                    })
                    .addParametricCallback(0.9, () -> {
                        follower.setMaxPower(0.6);
                    })

                    .addParametricCallback(0.75, () -> {

                        shooter.ShortVelocity();

                    })
                    .build();

            //---------------------------------------------------
            // SHOOT 3
            //---------------------------------------------------

            shoot3 = follower.pathBuilder()

                    .addPath(new BezierLine(

                            new Pose(129, 60),
                            new Pose(87.5, 77)

                    ))

                    .setLinearHeadingInterpolation(

                            Math.toRadians(-35),
                            Math.toRadians(-35)

                    )
                    .addParametricCallback(0.1, () -> {
                        shooter.ShortVelocity();
                        follower.setMaxPower(0.85);
                        turret.setFieldAngle(55);
                    })

                    .addParametricCallback(0.90, () -> {
                        follower.setMaxPower(0.6);
                        intake.stop();

                    })

                    .build();

            //---------------------------------------------------
            // INTAKE 3
            //---------------------------------------------------

            intake3 = follower.pathBuilder()

                    .addPath(new BezierCurve(

                            new Pose(87.5, 77),
                            new Pose(109, 61),
                            new Pose(130, 61)

                    ))

                    .setLinearHeadingInterpolation(

                            Math.toRadians(-35),
                            Math.toRadians(39)

                    )

                    .addParametricCallback(0.05, () -> {

                        intake.intakeIn();
                        follower.setMaxPower(0.85);
                        shooter.stop();

                    })

                    .addParametricCallback(0.75, () -> {

                        shooter.ShortVelocity();

                    })
                    .addParametricCallback(0.9, () -> {

                        follower.setMaxPower(0.6);

                    })

                    .build();

            //---------------------------------------------------
            // SHOOT 4
            //---------------------------------------------------

            shoot4 = follower.pathBuilder()

                    .addPath(new BezierLine(

                            new Pose(129, 60),
                            new Pose(87.5, 77)

                    ))

                    .setLinearHeadingInterpolation(

                            Math.toRadians(-35),
                            Math.toRadians(-35)

                    )
                    .addParametricCallback(0.1, () -> {
                        turret.setFieldAngle(55);
                        follower.setMaxPower(0.85);
                    })

                    .addParametricCallback(0.90, () -> {

                        intake.stop();
                        follower.setMaxPower(0.6);

                    })

                    .build();

            //---------------------------------------------------
            // INTAKE 4
            //---------------------------------------------------

            intake4 = follower.pathBuilder()

                    .addPath(new BezierCurve(

                            new Pose(87.5, 77),
                            new Pose(109, 60),
                            new Pose(130, 59)

                    ))

                    .setLinearHeadingInterpolation(

                            Math.toRadians(-35),
                            Math.toRadians(39)

                    )

                    .addParametricCallback(0.05, () -> {

                        intake.intakeIn();
                        follower.setMaxPower(0.85);
                        shooter.stop();

                    })

                    .addParametricCallback(0.75, () -> {

                        shooter.ShortVelocity();

                    })
                    .addParametricCallback(0.9, () -> {
                        follower.setMaxPower(0.6);
                    })

                    .build();

            //---------------------------------------------------
            // SHOOT 5
            //---------------------------------------------------

            shoot5 = follower.pathBuilder()

                    .addPath(new BezierLine(

                            new Pose(129, 60),
                            new Pose(87.5, 77)

                    ))

                    .setLinearHeadingInterpolation(

                            Math.toRadians(-35),
                            Math.toRadians(-35)

                    )
                    .addParametricCallback(0.1, () -> {
                        turret.setFieldAngle(55);
                        follower.setMaxPower(0.85);
                    })

                    .addParametricCallback(0.90, () -> {
                        follower.setMaxPower(0.6);
                        intake.stop();

                    })

                    .build();

            //---------------------------------------------------
            // INTAKE 5
            //---------------------------------------------------

            intake5 = follower.pathBuilder()

                    .addPath(new BezierCurve(

                            new Pose(87.5, 77),
                            new Pose(108, 82),
                            new Pose(124, 85)

                    ))
                    .setLinearHeadingInterpolation(

                            Math.toRadians(0),
                            Math.toRadians(0)

                    )

                    .addParametricCallback(0.05, () -> {
                        follower.setMaxPower(0.85);
                        intake.intakeIn();

                    })
                    .addParametricCallback(0.9, () -> {
                        follower.setMaxPower(0.7);
                    })

                    .build();
            //---------------------------------------------------
            // SHOOT 6
            //---------------------------------------------------

            shoot6 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(124, 82),
                                    new Pose(87.5, 77)
                            )
                    )

                    .setLinearHeadingInterpolation(

                            Math.toRadians(0),
                            Math.toRadians(0)

                    )
                    .build();
        }
    }
}