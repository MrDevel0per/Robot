package org.firstinspires.ftc.teamcode.robotModel;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {
    private DcMotor leftRotator;
    private DcMotor rightRotator;
    private Servo clawServo = null;
    private Servo clawServoTwo = null;
    private CRServo clawRotator = null;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    public boolean isHolding = true;

    public Arm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.leftRotator = hardwareMap.get(DcMotor.class, "left_rotator");
        leftRotator.setDirection(DcMotor.Direction.FORWARD);
        this.rightRotator = hardwareMap.get(DcMotor.class, "right_rotator");
        rightRotator.setDirection(DcMotor.Direction.REVERSE);
        this.clawRotator = hardwareMap.get(CRServo.class, "claw_rotator");
        clawRotator.setDirection(CRServo.Direction.FORWARD);
        this.clawServo = hardwareMap.get(Servo.class, "claw_servo");
        clawServo.setDirection(Servo.Direction.FORWARD);
        this.clawServoTwo = hardwareMap.get(Servo.class, "claw_servo_two");
        clawServoTwo.setDirection(Servo.Direction.REVERSE);
    }

    public void grip() {
        clawServo.setPosition(1.0);
        clawServoTwo.setPosition(1.0);
    }

    public void unGrip() {
        clawServo.setPosition(0.0);
        clawServoTwo.setPosition(0.0);
    }

    public int getArmRotation() {
        return (this.leftRotator.getCurrentPosition() + this.rightRotator.getCurrentPosition()) / 2;
    }

    public enum Position {
        GROUND(10),
        FIRST_LINE(20),
        SECOND_LINE(30),
        TRANSPORT(40);

        public final int DEGREES_OF_360;

        private Position(int degrees) {
            this.DEGREES_OF_360 = degrees;
        }
    }

    public void rotateArm(Position position) {
        isHolding = false;
        int currentPosition = getArmRotation();
        int targetPosition = position.DEGREES_OF_360;
        int difference = targetPosition - currentPosition;
        int direction = 1;
        if (Math.abs(difference) > 180) {
            direction = -1;
            difference = (360 - currentPosition) + targetPosition;
        }
        while (Math.abs(difference) > 60) {
            int power = (int) Math.signum(difference) * (Math.min(Math.abs(difference), 1));
            rightRotator.setPower(power * direction * 0.3);
            leftRotator.setPower(power * direction * 0.3);
            difference = getArmRotation() - targetPosition;
        }
        rightRotator.setPower(0);
        leftRotator.setPower(0);
        isHolding = true;
        hold();
    }

    private void rotateArmToDesiredPos(int desiredPosition) {
        double TICKS_PER_360_DEGREES = 537.7;
        int allowedError = 1;
        int currentMotorPosition = this.getArmRotation();
        int difference = desiredPosition - currentMotorPosition;
        double power = 0.25;
        if (difference > allowedError) {
            rightRotator.setPower(power);
            leftRotator.setPower(power);
            rotateArmToDesiredPos(desiredPosition);
        } else if (difference < -allowedError) {
            rightRotator.setPower(-power);
            leftRotator.setPower(-power);
            rotateArmToDesiredPos(desiredPosition);
        } else {
            rightRotator.setPower(0);
            leftRotator.setPower(0);
        }
    }

    public void hold() {
        long CHECK_EVERY_MILLISECONDS = 100;
        int startingMotorPosition = getArmRotation();
        long currentTime = System.nanoTime();
        while (isHolding) {
            int currentMotorPosition = getArmRotation();
            if (Math.abs(currentMotorPosition - startingMotorPosition) > 1) {
                rotateArmToDesiredPos(startingMotorPosition);
            }
            while (System.nanoTime() - currentTime < CHECK_EVERY_MILLISECONDS) {
                if (!isHolding) {
                    return;
                }
            }
            currentTime = System.nanoTime();
        }
    }

    public void clawRotate(double power) {
        clawRotator.setPower(power);
    }

    public void armRotate(double power) {
        leftRotator.setPower(power);
        rightRotator.setPower(power);
    }
}