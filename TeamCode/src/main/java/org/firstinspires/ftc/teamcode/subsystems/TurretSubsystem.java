package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TurretSubsystem {

    private final DcMotorEx turret;

    private static final double TICKS_PER_DEGREE = 6.603;

    private static final double MIN_ANGLE = -120.0;
    private static final double MAX_ANGLE = 120.0;

    private double kP = 0.013;
    private double kI = 0.0;
    private double kD = 0.00001;

    private double integral = 0;
    private double lastError = 0;

    private double lockedFieldAngle = 0;
    private double turretOffsetDeg = 0;

    private double targetTurretDeg = 0;
    private double targetTicks = 0;

    private double lastPosition = 0;
    private double turretVelocity = 0;

    private final ElapsedTime pidTimer = new ElapsedTime();

    public TurretSubsystem(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pidTimer.reset();
    }

    public void update(double robotHeadingDeg) {

        double rawTarget =
                lockedFieldAngle
                        - robotHeadingDeg
                        + turretOffsetDeg;

        targetTurretDeg = angleWrap(rawTarget);

        targetTurretDeg = Math.max(
                MIN_ANGLE,
                Math.min(MAX_ANGLE, targetTurretDeg)
        );

        targetTicks = targetTurretDeg * TICKS_PER_DEGREE;

        int currentTicks = turret.getCurrentPosition();
        double error = targetTicks - currentTicks;

        if (Math.abs(error) < 8) {
            turret.setPower(0);
            integral = 0;
            lastError = error;
            return;
        }

        double dt = Math.max(pidTimer.seconds(), 0.001);
        pidTimer.reset();

        turretVelocity = (currentTicks - lastPosition) / dt;
        lastPosition = currentTicks;

        integral += error * dt;
        integral = Math.max(-5000, Math.min(5000, integral));

        double derivative = (error - lastError) / dt;
        derivative = Math.max(-4000, Math.min(4000, derivative));

        double power =
                (kP * error)
                        + (kI * integral)
                        + (kD * derivative)
                        - (0.00025 * turretVelocity);

        if (Math.abs(error) > 200) {
            power += Math.signum(error) * 0.10;
        } else if (Math.abs(error) > 80) {
            power += Math.signum(error) * 0.06;
        } else if (Math.abs(error) > 20) {
            power += Math.signum(error) * 0.03;
        }

        double maxPower;

        if (Math.abs(error) > 300) {
            maxPower = 1.00;
        } else if (Math.abs(error) > 120) {
            maxPower = 0.70;
        } else if (Math.abs(error) > 40) {
            maxPower = 0.45;
        } else {
            maxPower = 0.18;
        }

        power = Math.max(-maxPower, Math.min(maxPower, power));

        turret.setPower(power);
        lastError = error;
    }

    private double angleWrap(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    public void setFieldAngle(double angle) {
        lockedFieldAngle = angle;
    }

    public void setOffset(double offsetDeg) {
        turretOffsetDeg = offsetDeg;
    }

    public void resetLock() {
        lockedFieldAngle = 0;
    }

    public void resetEncoder() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPIDF(double p, double i, double d, double f) {
        kP = p;
        kI = i;
        kD = d;
    }

    public void stop() {
        turret.setPower(0);
    }
    public void setPower() {
        turret.setPower(0.5);
    }

    public int getPosition() {
        return turret.getCurrentPosition();
    }

    public double getLockedFieldAngle() {
        return lockedFieldAngle;
    }

    public double getTargetTurretDeg() {
        return targetTurretDeg;
    }

    public double getTargetTicks() {
        return targetTicks;
    }

    public double getTurretAngleDeg() {
        return turret.getCurrentPosition() / TICKS_PER_DEGREE;
    }
}