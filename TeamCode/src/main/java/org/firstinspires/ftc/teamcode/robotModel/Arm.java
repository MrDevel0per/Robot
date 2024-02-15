package org.firstinspires.ftc.teamcode.robotModel;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    private final Motors motors;
    private final Servos servos;
    public boolean isHolding = true;

    public Arm(HardwareMap hardwareMap) {
        this.motors = new Motors(hardwareMap);
        this.servos = new Servos(hardwareMap);
    }

    public void grip() {
        servos.rightClawServo.setPosition(1.0);
        servos.leftClawServo.setPosition(1.0);
    }

    public void unGrip() {
        servos.rightClawServo.setPosition(0.0);
        servos.leftClawServo.setPosition(0.0);
    }

    public int getArmRotation() {
        return (motors.leftRotator.getCurrentPosition() + motors.rightRotator.getCurrentPosition()) / 2;
    }

    public enum Position {
        GROUND(10),
        FIRST_LINE(20),
        SECOND_LINE(30),
        TRANSPORT(40);

        public final int DEGREES_OF_360;

        Position(int degrees) {
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
            motors.rightRotator.setPower(power * direction * 0.3);
            motors.leftRotator.setPower(power * direction * 0.3);
            difference = getArmRotation() - targetPosition;
        }
        motors.rightRotator.setPower(0);
        motors.leftRotator.setPower(0);
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
            motors.rightRotator.setPower(power);
            motors.leftRotator.setPower(power);
            rotateArmToDesiredPos(desiredPosition);
        } else if (difference < -allowedError) {
            motors.rightRotator.setPower(-power);
            motors.leftRotator.setPower(-power);
            rotateArmToDesiredPos(desiredPosition);
        } else {
            motors.rightRotator.setPower(0);
            motors.leftRotator.setPower(0);
        }
    }

  public void hold() {
    new Thread(() -> {
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
    }).start();
}

    public void clawRotate(double power) {
        // TODO: Use the actual motor
        motors.clawRotator.setPower(power);
    }

    public void armRotate(double power) {
        motors.leftRotator.setPower(power);
        motors.rightRotator.setPower(power);
    }
}