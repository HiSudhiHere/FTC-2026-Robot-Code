package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {

    private final DcMotorEx intake;

    public IntakeSubsystem(HardwareMap hardwareMap) {

        intake = hardwareMap.get(DcMotorEx.class, "I");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotor.Direction.REVERSE);
    }

    public void intakeIn() {
        intake.setPower(-0.7);
    }

    public void intakeOut() {
        intake.setPower(0.7);
    }

    public void stop() {
        intake.setPower(0);
    }

    public double getPower() {
        return intake.getPower();
    }

    public double getVelocity() {
        return intake.getVelocity();
    }

}