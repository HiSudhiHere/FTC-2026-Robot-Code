package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem {

    private final DcMotorEx shooterL;
    private final DcMotorEx shooterR;

    // === TUNED FOR 10FT TARGET (UPDATED) ===
    private static final double FAST_VELOCITY = 1390; // Target updated to 1600
    private static final double SLOW_VELOCITY = 1180;//1260
    private static final double SHORT_VELOCITY = 1300;//1260
    private static final double SPINUP_BOOST_POWER = 1.0;
    private static final double SPINUP_BOOST_ERROR = 250; // Boost window increased for fast start
    private static final double READY_TOLERANCE = 40;   // Strict tolerance for perfect consistency

    public ShooterSubsystem(HardwareMap hardwareMap) {

        shooterL = hardwareMap.get(DcMotorEx.class, "ShooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "ShooterR");

        shooterL.setDirection(DcMotor.Direction.REVERSE);
        shooterR.setDirection(DcMotor.Direction.FORWARD);

        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // === NEW TUNED PIDF COEFFICIENTS (UPDATED) ===
        // P=120, I=0, D=18, F=24.2 for lock and fast recovery
        shooterL.setVelocityPIDFCoefficients(125, 0, 5, 15);
        shooterR.setVelocityPIDFCoefficients(125, 0, 5, 15);
    }

    public void shootFast() {
        shootVelocity(FAST_VELOCITY);
    }

    public void shootVERYSLOW() {
        shootVelocity(SHORT_VELOCITY);
    }

    public void shootSlow() {
        shootVelocity(SLOW_VELOCITY);
    }

    public void shootVelocity(double velocity) {
        double currentVel = getAverageVelocity();

        if (currentVel < velocity - SPINUP_BOOST_ERROR) {
            shooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            shooterL.setPower(1.0);
            shooterR.setPower(1.0);
        } else {
            if (shooterL.getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                shooterL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooterR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                // Coefficients ko hardware par force-refresh karein
                shooterL.setVelocityPIDFCoefficients(120, 0, 18, 24.2);
                shooterR.setVelocityPIDFCoefficients(120, 0, 18, 24.2);
            }
            // Ab PIDF speed ko 1600 par tight lock rakhega
            shooterL.setVelocity(velocity);
            shooterR.setVelocity(velocity);
        }
    }

    public void stop() {
        // Stop karte waqt hamesha zero power dein taaki motor turant free ho jaye
        shooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterL.setPower(0);
        shooterR.setPower(0);
    }

    public void reverse() {
        shooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterL.setPower(-0.5);
        shooterR.setPower(-0.5);
    }

    public boolean readyForFastShot() {
        return getAverageVelocity() >= FAST_VELOCITY - READY_TOLERANCE;
    }

    public boolean readyForSlowShot() {
        return getAverageVelocity() >= SLOW_VELOCITY - READY_TOLERANCE;
    }

    public double getCurrentVelocity(){
        return getCurrentVelocity();
    }

    public double getAverageVelocity() {
        return (Math.abs(shooterL.getVelocity())
                + Math.abs(shooterR.getVelocity())) / 2.0;
    }

    public double getLeftVelocity() {
        return shooterL.getVelocity();
    }

    public double getRightVelocity() {
        return shooterR.getVelocity();
    }
}