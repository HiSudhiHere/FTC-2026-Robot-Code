package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystemRedUpAuto {

    private final DcMotorEx shooterL;
    private final DcMotorEx shooterR;

    private static final double FAST_VELOCITY = 1450;
    private static final double SLOW_VELOCITY = 1240;

    public ShooterSubsystemRedUpAuto(HardwareMap hardwareMap) {

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

        shooterL.setVelocityPIDFCoefficients(125, 0, 25, 27.5);//125, 0, 25, 27.5 - PIDF for UP
        shooterR.setVelocityPIDFCoefficients(125, 0, 25, 27.5);
    }

    public void shootFast() {

        shooterL.setVelocity(FAST_VELOCITY);
        shooterR.setVelocity(FAST_VELOCITY);
    }

    public void shootSlow() {

        shooterL.setVelocity(SLOW_VELOCITY);
        shooterR.setVelocity(SLOW_VELOCITY);
    }

    public void shootVelocity(double velocity) {

        shooterL.setVelocity(velocity);
        shooterR.setVelocity(velocity);
    }

    public void reverse() {

        shooterL.setPower(-0.5);
        shooterR.setPower(-0.5);
    }

    public void stop() {

        shooterL.setVelocity(0);
        shooterR.setVelocity(0);
    }

    public boolean readyForFastShot() {

        return Math.abs(getAverageVelocity() - FAST_VELOCITY) < 50;
    }

    public boolean readyForSlowShot() {

        return Math.abs(getAverageVelocity() - SLOW_VELOCITY) < 50;
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