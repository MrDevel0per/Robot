package org.firstinspires.ftc.teamcode.robotModel;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Servos {
    public Servo leftClawServo;
    public Servo rightClawServo;

    public Servos(HardwareMap hardwareMap) {
        this.leftClawServo = hardwareMap.get(Servo.class, "claw_servo");
        leftClawServo.setDirection(Servo.Direction.FORWARD);
        this.rightClawServo = hardwareMap.get(Servo.class, "claw_servo_two");
        rightClawServo.setDirection(Servo.Direction.REVERSE);
    }

}
