package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoSubsystem {

    private final Servo stopperServo;
    private final Servo hudderServo;


    public ServoSubsystem(HardwareMap hardwareMap) {

        stopperServo = hardwareMap.get(Servo.class, "Sservo");
        hudderServo = hardwareMap.get(Servo.class, "hservo");
    }

    // Stopper Servo
    public void setStopper(double position) {

        stopperServo.setPosition(position);
    }

    public double getStopperPosition() {
        return stopperServo.getPosition();
    }

    // Hudder Servo
    public void setHudder(double position) {
        hudderServo.setPosition(position);
    }

    public double getHudderPosition() {
        return hudderServo.getPosition();
    }
}