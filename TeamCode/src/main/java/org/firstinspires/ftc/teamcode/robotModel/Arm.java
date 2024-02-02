package org.firstinspires.ftc.teamcode.robotModel;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {
    // Motors for controlling rotate forward/backward
    //private DcMotor leftRotator;
    //private DcMotor rightRotator;

    // Motor for controlling up/down
    private DcMotor upDownMotor;
    private Servo clawServo = null;
    private Servo clawServoTwo = null;

    private CRServo clawRotator = null;

    // Motor for controlling open/close of the claw
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public Arm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.upDownMotor = hardwareMap.get(DcMotor.class, "up_down_motor");
        upDownMotor.setDirection(DcMotor.Direction.REVERSE);
        this.clawRotator = hardwareMap.get(CRServo.class, "claw_rotator");
        clawRotator.setDirection(CRServo.Direction.FORWARD);
        this.clawServo = hardwareMap.get(Servo.class, "claw_servo");
        clawServo.setDirection(Servo.Direction.FORWARD);
        this.clawServoTwo = hardwareMap.get(Servo.class, "claw_servo_two");
        clawServoTwo.setDirection(Servo.Direction.REVERSE);

    }


        public void grip(){
            clawServo.setPosition(1.0);
            clawServoTwo.setPosition(1.0);
        }

        public void unGrip() {
            clawServo.setPosition(0.0);
            clawServoTwo.setPosition(0.0);
        }

        public void rotate(double power){
        clawRotator.setPower(power);
        }
    /*public void rotate(double power) {
        leftRotator.setPower(power);
        rightRotator.setPower(power);
    }*/

    public void upDown(double power) {

        upDownMotor.setPower(power);
    }


}



