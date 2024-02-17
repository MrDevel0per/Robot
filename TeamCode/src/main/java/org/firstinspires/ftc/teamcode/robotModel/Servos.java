package org.firstinspires.ftc.teamcode.robotModel;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.C;

public class Servos {
    public Servo leftClawServo;
    public Servo rightClawServo;

    public CRServo clawRotatorRight;
    public CRServo clawRotatorLeft;

    public Servo droneLauncher;

    public Servos(HardwareMap hardwareMap) {
        this.leftClawServo = hardwareMap.get(Servo.class, "claw_servo");
        leftClawServo.setDirection(Servo.Direction.FORWARD);
        this.rightClawServo = hardwareMap.get(Servo.class, "claw_servo_two");
        rightClawServo.setDirection(Servo.Direction.REVERSE);
        this.clawRotatorRight = hardwareMap.get(CRServo.class, "claw_rotator_right");
        clawRotatorRight.setDirection(CRServo.Direction.FORWARD);
        this.clawRotatorLeft = hardwareMap.get(CRServo.class, "claw_rotator_left");
        clawRotatorLeft.setDirection(CRServo.Direction.REVERSE);
        this.droneLauncher = hardwareMap.get(Servo.class, "droneLauncher");
        droneLauncher.setDirection(Servo.Direction.FORWARD);
    }

}
