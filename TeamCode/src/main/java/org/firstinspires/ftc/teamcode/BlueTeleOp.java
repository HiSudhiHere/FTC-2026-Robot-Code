package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ServoSubsystem;
@Disabled
@TeleOp(name = "Blue Teleop")
public class BlueTeleOp extends LinearOpMode {

    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private DcMotorEx turret;
    private Limelight3A limelight;
    private ServoSubsystem servos;

    private GoBildaPinpointDriver pinpoint;

    private Servo hooder;


    // Tune these

    // Servo Positions
    private static final double STOPPER_OPEN = 0.6;
    private static final double DRIVE_SPEED = 0.95;
    private static final double STOPPER_CLOSED = 0.3;
    private double filteredTx = 0;
    private double lastAimError = 0;
    private static final double CLOSE_KP = 0.010;
    private static final double FAR_KP = 0.025;
    private static final double KD = 0.010;
    private static final double CLOSE_MIN_POWER = 0.035;
    private static final double FAR_MIN_POWER = 0.08;
    private static final double CLOSE_MAX_POWER = 0.22;
    private static final double MAX_POWER = 0.45;
    private static final double DEADZONE = 1.5;
    private static final double SLOW_ZONE_DEGREES = 7.0;
    private static final double TX_FILTER_OLD_WEIGHT = 0.60;
    private static final double TX_FILTER_NEW_WEIGHT = 0.40;
    private static final double TICKS_PER_DEGREE = 7.47;
    private static final double TURRET_LEFT_LIMIT_DEG = -100.0;
    private static final double TURRET_RIGHT_LIMIT_DEG = 100.0;
    @Override
    public void runOpMode() {

        drive = new DriveSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        servos = new ServoSubsystem(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

// Select the pod type
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

// Set encoder directions (change if necessary)
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();


        turret = hardwareMap.get(DcMotorEx.class, "turret");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight.pipelineSwitch(0);
        limelight.start();
        servos.setStopper(STOPPER_CLOSED);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            pinpoint.update();

            Pose2D pose = pinpoint.getPosition();

            double robotX = pose.getX(DistanceUnit.INCH);
            double robotY = pose.getY(DistanceUnit.INCH);
            double heading = pose.getHeading(AngleUnit.DEGREES);
            // Drive
            drive.drive(
                    -gamepad1.left_stick_y * DRIVE_SPEED,
                    gamepad1.left_stick_x * DRIVE_SPEED,
                    gamepad1.right_stick_x * DRIVE_SPEED
            );

            // =========================
            // INTAKE CONTROL
            // =========================
            if (gamepad1.left_bumper) {
                intake.intakeOut();
            }
            else if (gamepad1.left_trigger > 0.1) {
                intake.intakeIn();
            }
            else {
                intake.stop();
            }

            // =========================
            // STOPPER SERVO CONTROL
            // =========================
            if (gamepad1.dpad_down) {
                servos.setStopper(STOPPER_CLOSED);
            }

            if (gamepad1.dpad_up) {
                servos.setStopper(STOPPER_OPEN);
            }

            // =========================
            // HOODER SERVO CONTROL
            // =========================
            if (gamepad2.right_bumper) {
                servos.setHudder(0.50);
            }

            if (gamepad2.right_trigger>0.1) {
                servos.setHudder(0.12);
            }
            /*
            // =========================
            // SHOOTER CONTROL
            // =========================
            if (gamepad1.right_bumper) {

                shooter.shootFast();

            }
            else if (gamepad1.right_trigger > 0.1) {

                shooter.reverse();

            }
            else if (gamepad1.y){
                shooter.shootSlow();
            }
            else {

                shooter.stop();
            }

             */
            // =========================
            // SHOOTER CONTROL & AUTOMATION (UPDATED)
            // =========================
            if (gamepad1.right_bumper) {

                // Motors ko 1600 RPM par spin karna shuru karein
                shooter.shootFast();

                // AUTOMATION HACK: Agar shooter ready hai aur aap D-PAD UP dabate hain,
                // toh teeno balls automatic perfect interval (delay) ke sath fire hongi.
                if (gamepad1.dpad_up && shooter.readyForFastShot()) {

                    // Ball 1 Launch
                    servos.setStopper(STOPPER_OPEN);
                    sleep(140); // Stopper khulne ka time
                    servos.setStopper(STOPPER_CLOSED);

                    // --- Recovery Gap ---
                    // Is 250ms ke break me motor wapas crash velocity (1300) se 1600 RPM recover kar lega
                    sleep(250);

                    // Ball 2 Launch
                    servos.setStopper(STOPPER_OPEN);
                    sleep(140);
                    servos.setStopper(STOPPER_CLOSED);

                    // --- Recovery Gap ---
                    sleep(250);

                    // Ball 3 Launch
                    servos.setStopper(STOPPER_OPEN);
                    sleep(140);
                    servos.setStopper(STOPPER_CLOSED);
                }
            }
            else if (gamepad1.right_trigger > 0.1) {
                shooter.reverse();
            }
            else if (gamepad1.y){
                shooter.shootSlow();
            }
            else {
                shooter.stop();
            }


            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {


                double tx = result.getTx();

                double x = result.getBotpose().getPosition().x;
                double y = result.getBotpose().getPosition().y;

                double distance = Math.hypot(x, y);

                filteredTx =
                        filteredTx * TX_FILTER_OLD_WEIGHT
                                + tx * TX_FILTER_NEW_WEIGHT;

                double turretPower = calculateTurretPower();

                turret.setPower(applyTurretWrapLimit(turretPower));

                telemetry.addData("Tag Visible", true);
                telemetry.addData("TX Raw", tx);
                telemetry.addData("TX Filtered", filteredTx);
                telemetry.addData("Turret Power", turretPower);

            } else {

                turret.setPower(0);
                lastAimError = 0;

                telemetry.addData("Tag Visible", false);
            }

            // Manual override
            if (gamepad2.dpad_left) {
                turret.setPower(applyTurretWrapLimit(0.3));
            }

            if (gamepad2.dpad_right) {
                turret.setPower(applyTurretWrapLimit(-0.3));
            }

            telemetry.addData("Turret Position",
                    turret.getCurrentPosition());

            telemetry.addLine("===== SHOOTER =====");

            telemetry.addData("Left Velocity",
                    "%.1f", shooter.getLeftVelocity());

            telemetry.addData("Right Velocity",
                    "%.1f", shooter.getRightVelocity());
            telemetry.addData("Servo position", "%.2f", servos.getHudderPosition());

            telemetry.addLine("===== PINPOINT =====");
            telemetry.addData("X (in)", "%.1f", robotX);
            telemetry.addData("Y (in)", "%.1f", robotY);
            telemetry.addData("Heading", "%.1f", heading);
            telemetry.update();
        }

        turret.setPower(0);
        drive.stop();
    }

    private double calculateTurretPower() {
        double absTx = Math.abs(filteredTx);

        if (absTx <= DEADZONE) {
            lastAimError = 0;
            return 0;
        }

        double error = -filteredTx;
        double derivative = error - lastAimError;
        lastAimError = error;

        boolean closeToCenter = absTx < SLOW_ZONE_DEGREES;
        double kP = closeToCenter ? CLOSE_KP : FAR_KP;
        double minPower = closeToCenter ? CLOSE_MIN_POWER : FAR_MIN_POWER;
        double maxPower = closeToCenter ? CLOSE_MAX_POWER : MAX_POWER;

        double power = error * kP + derivative * KD;

        if (Math.abs(power) < minPower) {
            power = Math.signum(power) * minPower;
        }

        return Math.max(-maxPower, Math.min(maxPower, power));
    }

    private double applyTurretWrapLimit(double requestedPower) {
        double turretDeg = turret.getCurrentPosition() / TICKS_PER_DEGREE;

        if (turretDeg >= TURRET_RIGHT_LIMIT_DEG && requestedPower > 0) {
            return -MAX_POWER;
        }

        if (turretDeg <= TURRET_LEFT_LIMIT_DEG && requestedPower < 0) {
            return MAX_POWER;
        }

        return requestedPower;
    }
}