package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Turret Angle Test", group = "Test")
public class TurretAngleTest extends LinearOpMode {

    private DcMotorEx turret;

    // GoBILDA Yellow Jacket 312 RPM
    private static final double TICKS_PER_REV = 537.7;

    // Turret Gear Ratio
    private static final double GEAR_RATIO = 4.2;

    // Total encoder ticks for one full turret revolution
    private static final double TICKS_PER_TURRET_REV = TICKS_PER_REV * GEAR_RATIO;

    private int zeroTicks = 0;

    @Override
    public void runOpMode() {

        turret = hardwareMap.get(DcMotorEx.class, "turret");

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Press START");
        telemetry.update();

        waitForStart();

        zeroTicks = turret.getCurrentPosition();

        while (opModeIsActive()) {

            // Manual turret control
            double power = -gamepad1.left_stick_x * 0.4;
            turret.setPower(power);

            // Reset zero
            if (gamepad1.a) {
                zeroTicks = turret.getCurrentPosition();
            }

            int ticks = turret.getCurrentPosition() - zeroTicks;

            double angle = (ticks / TICKS_PER_TURRET_REV) * 360.0;

            telemetry.addLine("===== TURRET =====");
            telemetry.addData("Encoder", ticks);
            telemetry.addData("Angle", "%.2f°", angle);
            telemetry.addData("Motor Power", "%.2f", power);

            telemetry.addLine();
            telemetry.addLine("A = Set Current Position as 0°");
            telemetry.addLine("Left Stick X = Rotate Turret");

            telemetry.update();
        }
    }
}