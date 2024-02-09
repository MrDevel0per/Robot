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
    //private DcMotor upDownMotor;
    private DcMotor leftRotator;
    private DcMotor rightRotator;
    private Servo clawServo = null;
    private Servo clawServoTwo = null;

    private CRServo clawRotator = null;

    // Motor for controlling open/close of the claw
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public boolean isHolding = true;

    public Arm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        //this.upDownMotor = hardwareMap.get(DcMotor.class, "up_down_motor");
        //upDownMotor.setDirection(DcMotor.Direction.REVERSE);
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
        // TODO: Update to correct degrees

        GROUND(1),
        FIRST_LINE(2),
        SECOND_LINE(3),
        TRANSPORT(4);

        public final int DEGREES_OF_360;

        private Position(int degrees) {
            this.DEGREES_OF_360 = degrees;
        }
        }
    public void rotateArm(Position position) {
        isHolding = false;
        // TODO: First, set current position to the bottom position
        // TODO: Then, calculate necessary to get to ground
        // 1. Set current position:
        int currentPosition = getArmRotation();

        // 2. Calculate necessary movement:
        int targetPosition = position.DEGREES_OF_360;
        int difference = targetPosition - currentPosition;

        // 3. Handle direction and movement:
        int direction = 1; // Assume clockwise initially
        if (Math.abs(difference) > 180) { // Check for shorter counter-clockwise rotation
            direction = -1;
            difference = (360 - currentPosition) + targetPosition;
            System.out.println("Difference: " + difference);
        }

        // 4. Move motors smoothly:
        while (Math.abs(difference) > 60) {// Adjust the threshold as needed
            int power = (int) Math.signum(difference) * (Math.min(Math.abs(difference), 1)); // Scale power smoothly
            System.out.println("Direction: " + direction);
            System.out.println("Power: " + power);
            rightRotator.setPower(power * direction * 0.3);
            leftRotator.setPower(power * direction * 0.3);
            difference = getArmRotation() - targetPosition; // Update difference
        }

        // 5. Final adjustment:
        rightRotator.setPower(0);
        leftRotator.setPower(0);
        rotateArmToDesiredPos(targetPosition); // Ensure final position accuracy

    }



    private void rotateArmToDesiredPos(int desiredPosition) {
        // Calculate movement
        // Calculate movement direction
        // It's ok if the motor is off by [MAX_CLICKS_OFF] clicks
        double TICKS_PER_360_DEGREES = 537.7;
//        int allowedErrorDegrees = 4/360;
        int allowedError = 1;
        int currentMotorPosition = this.getArmRotation();
        int difference = desiredPosition - currentMotorPosition;
        // TODO: Make more powerful if needed
        double power = 0.5;
        if (difference > allowedError) {
            // Move up
            rightRotator.setPower(power);
            leftRotator.setPower(power);
        } else if (difference < -allowedError) {
            // Move down
            rightRotator.setPower(-power);
            leftRotator.setPower(-power);
        } else {
            // We are at the right position
            rightRotator.setPower(0);
            leftRotator.setPower(0);
        }

    }

    /*
    Hold the claw in place
     */
    public void hold() {
        long CHECK_EVERY_MILLISECONDS = 100;
        int startingMotorPosition = getArmRotation();
        long currentTime = System.nanoTime();
        while (isHolding) {
            int currentMotorPosition = getArmRotation();
            if (currentMotorPosition != startingMotorPosition) {
                // Oh noes! The motor has moved!
                // We need to move the motor back
                rotateArmToDesiredPos(startingMotorPosition);
            }

            while (System.nanoTime() - currentTime < CHECK_EVERY_MILLISECONDS) {
                if (!isHolding) {
                    return;
                }
                continue;
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

    /*public void upDown(double power) {

        upDownMotor.setPower(power);
    }*/


}



