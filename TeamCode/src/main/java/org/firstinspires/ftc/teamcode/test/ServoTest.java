package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test", group = "Test")
public class ServoTest extends LinearOpMode {

    private Servo hservo;

    @Override
    public void runOpMode() {

        hservo = hardwareMap.get(Servo.class, "hservo");


        telemetry.addLine("Servo Test Ready");
        telemetry.addLine("A = Position 0.0");
        telemetry.addLine("B = Position 0.5");
        telemetry.addLine("Y = Position 1.0");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                hservo.setPosition(0.0);
            }

            if (gamepad1.b) {
                hservo.setPosition(0.3);
            }

            if (gamepad1.y) {
                hservo.setPosition(1.0);
            }

            telemetry.addData("Servo Position", hservo.getPosition());
            telemetry.update();
        }
    }
}