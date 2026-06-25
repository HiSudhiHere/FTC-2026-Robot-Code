package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;

import com.qualcomm.hardware.limelightvision.Limelight3A;

public class BLTurret {

    private final DcMotor turret;

    // 537.7 CPR * (100/20)

    private double filteredTx = 0;
    private double lastAimError = 0;


    private boolean tagFound = false;
    private double tagTx = 0;
    private double tagDistance = 0;

    private static final double CLOSE_KP = 0.010;
    private static final double FAR_KP = 0.025;
    private static final double KD = 0.005;

    private static final double CLOSE_MIN_POWER = 0.025;
    private static final double FAR_MIN_POWER = 0.08;

    private static final double TX_OFFSET = 8;

    private static final double CLOSE_MAX_POWER = 0.22;
    private static final double MAX_POWER = 0.45;

    private static final double DEADZONE = 2.5;
    private static final double SLOW_ZONE_DEGREES = 7.0;

    private static final double TX_FILTER_OLD_WEIGHT = 0.60;
    private static final double TX_FILTER_NEW_WEIGHT = 0.40;

    private static final double TICKS_PER_DEGREE = 7.47;

    private static final double MIN_ANGLE = -120.0;
    private static final double MAX_ANGLE = 120.0;

    private static final double VISION_KP = 0.018;
    private static final double VISION_KD = 0.001;

    private double lastTx = 0;

    // PIDF
    private double kP = 0.013;
    private double kI = 0.0000;
    private double kD = 0.00001;
    private double kF = 0.020;

    private double integral = 0;
    private double lastError = 0;

    /*
    private double lockedFieldAngle = 0;
    private double turretOffsetDeg = 0;

     */

    private double lastPosition = 0;
    private double turretVelocity = 0;

    private Limelight3A limelight;


    private final ElapsedTime pidTimer = new ElapsedTime();

    public BLTurret(DcMotorEx turret) {
        this.turret = turret;
    }

    private enum TurretState {
        SEARCH,
        TRACK,
        HOLD
    }

    private TurretState state = TurretState.SEARCH;

    private ElapsedTime tagLostTimer = new ElapsedTime();

    private int holdEncoder = 0;

    private int searchDirection = 1;

    public BLTurret(HardwareMap hardwareMap) {

        turret = hardwareMap.get(DcMotorEx.class, "turret");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        turret.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);


        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pidTimer.reset();
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

        double turretDeg =
                turret.getCurrentPosition() / TICKS_PER_DEGREE;

        if (turretDeg >= 100 && requestedPower > 0) {
            return -MAX_POWER;
        }

        if (turretDeg <= -100 && requestedPower < 0) {
            return MAX_POWER;
        }

        return requestedPower;
    }

    public void trackAprilTag() {

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            tagFound = true;

            tagTx = result.getTx() - TX_OFFSET;

            double x = result.getBotpose().getPosition().x;
            double y = result.getBotpose().getPosition().y;

            tagDistance = Math.hypot(x, y);

            filteredTx =
                    filteredTx * TX_FILTER_OLD_WEIGHT
                            + tagTx * TX_FILTER_NEW_WEIGHT;

            double turretPower = calculateTurretPower();

            turret.setPower(applyTurretWrapLimit(turretPower));

        } else {

            tagFound = false;

            turret.setPower(0);

            lastAimError = 0;
        }
    }

    /**
     * Field-centric turret lock
     */
    /*



    public void update(double robotHeadingDeg) {

        double turretTargetDeg =
                lockedFieldAngle
                        - robotHeadingDeg
                        + turretOffsetDeg;

        turretTargetDeg = Math.max(
                MIN_ANGLE,
                Math.min(MAX_ANGLE, turretTargetDeg));

        int targetTicks =
                (int) (turretTargetDeg * TICKS_PER_DEGREE);

        int currentTicks =
                turret.getCurrentPosition();

        double error =
                targetTicks - currentTicks;

        // Deadband to prevent oscillation
        if (Math.abs(error) < 8) {
            turret.setPower(0);
            integral = 0;
            lastError = error;
            return;
        }

        double dt = pidTimer.seconds();

        pidTimer.reset();

        turretVelocity = (currentTicks - lastPosition) / Math.max(dt, 0.001);
        lastPosition = currentTicks;

        integral += error * dt;

        integral = Math.max(
                -5000,
                Math.min(5000, integral));

        double derivative =
                (error - lastError)
                        / Math.max(dt, 0.001);

        derivative = Math.max(-4000, Math.min(4000, derivative));

        double power =
                (kP * error)
                        + (kI * integral)
                        + (kD * derivative)
                        - (0.00025 * turretVelocity);

        if (Math.abs(error) > 200) {
            power += Math.signum(error) * 0.10;
        }
        else if (Math.abs(error) > 80) {
            power += Math.signum(error) * 0.06;
        }
        else if (Math.abs(error) > 20) {
            power += Math.signum(error) * 0.03;
        }

        power = Math.max(
                -1.0,
                Math.min(1.0, power));

        double maxPower;

        if (Math.abs(error) > 300) {
            maxPower = 1.00;
        }
        else if (Math.abs(error) > 120) {
            maxPower = 0.70;
        }
        else if (Math.abs(error) > 40) {
            maxPower = 0.45;
        }
        else {
            maxPower = 0.18;
        }

        power = Math.max(-maxPower, Math.min(maxPower, power));
        turret.setPower(power);

        lastError = error;
    }

     */

    /**
     * D-pad left
     */
    /*
    public void aimLeft() {
        lockedFieldAngle += 1;
    }

     */

    /**
     * D-pad right
     */
    /*
    public void aimRight() {
        lockedFieldAngle -= 1;
    }

     */

    /**
     * Direct field angle set
     */
    /*
    public void setFieldAngle(double angle) {
        lockedFieldAngle = angle;
    }

     */

    /**
     * Zero turret lock
     */
    /*
    public void resetLock() {
        lockedFieldAngle = 0;
    }

     */

    /**
     * Encoder reset
     */
    public void resetEncoder() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * PID tuning
     */
    public void setPIDF(
            double p,
            double i,
            double d,
            double f) {

        kP = p;
        kI = i;
        kD = d;
        kF = f;
    }
    /*

    public void setOffset(double offsetDeg) {
        turretOffsetDeg = offsetDeg;
    }

     */

    public void stop() {
        turret.setPower(0);
    }

    public int getPosition() {
        return turret.getCurrentPosition();
    }

    /*
    public double getLockedFieldAngle() {
        return lockedFieldAngle;
    }

     */

    public double getKP() {
        return kP;
    }

    public double getKD() {
        return kD;
    }

    public double getKF() {
        return kF;
    }
    public boolean isTagFound() {
        return tagFound;
    }

    public double getTx() {
        return tagTx;
    }

    public double getDistance() {
        return tagDistance;
    }
}