package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RGBSubsystem {

    private final Servo rgb;

    public RGBSubsystem(HardwareMap hardwareMap) {
        rgb = hardwareMap.get(Servo.class, "rgb");
    }

    public void red() {
        rgb.setPosition(0.28);
    }

    public void blue() {
        rgb.setPosition(0.61);
    }
    public void yellow() { rgb.setPosition(0.388);}

    public void green() {
        rgb.setPosition(0.50);
    }

    public void purple() {
        rgb.setPosition(0.72);
    }

    public void off() {
        rgb.setPosition(0.00);
    }
}