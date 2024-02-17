package org.firstinspires.ftc.teamcode.robotModel;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * This class represents a chassis of a robot that does not use encoders.
 */
public class NoEncoderChassis {
    private final Motors motors;

    /**
     * Constructor for the NoEncoderChassis class.
     *
     * @param hardwareMap The hardware map of the robot.
     */
    public NoEncoderChassis(HardwareMap hardwareMap) {
        motors = new Motors(hardwareMap);
    }

    /**
     * Sets the power for each of the drive motors.
     *
     * @param leftFrontPower  The power for the left front motor.
     * @param rightFrontPower The power for the right front motor.
     * @param leftRearPower   The power for the left rear motor.
     * @param rightRearPower  The power for the right rear motor.
     */
    public void setDriveMotorPowers(double leftFrontPower, double rightFrontPower, double leftRearPower, double rightRearPower) {
        motors.leftFrontChassis.setPower(leftFrontPower);
        motors.rightFrontChassis.setPower(rightFrontPower);
        motors.leftRearChassis.setPower(leftRearPower);
        motors.rightRearChassis.setPower(rightRearPower);
    }

    /**
     * Handles the driving of the robot based on the input from the gamepad.
     *
     * @param gamepad  The gamepad that controls the robot.
     * @param maxPower The maximum power that can be applied to the motors.
     */
    public void handleDriving(Gamepad gamepad, double maxPower) {
        if (gamepad.left_stick_y == 0 && gamepad.right_stick_x == 0 && gamepad.left_stick_x == 0) {
            stop();
        } else {
            double drive = gamepad.left_stick_y;
            double turn = gamepad.right_stick_x;
            double strafe = gamepad.left_stick_x;
            double leftFrontPower = Range.clip(drive - turn - strafe, -maxPower, maxPower);
            double rightFrontPower = Range.clip(drive + turn + strafe, -maxPower, maxPower);
            double leftBackPower = Range.clip(drive - turn + strafe, -maxPower, maxPower);
            double rightBackPower = Range.clip(drive + turn - strafe, -maxPower, maxPower);
            setDriveMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        }
    }

    /**
     * Handles the driving of the robot based on the input from the gamepad.
     * This method uses a default max power of 0.25.
     *
     * @param gamepad The gamepad that controls the robot.
     */
    public void handleDriving(Gamepad gamepad) {
        handleDriving(gamepad, 0.50);
    }

    /**
     * Stops the robot by setting all motor powers to 0.
     */
    public void stop() {
        motors.leftFrontChassis.setPower(0);
        motors.rightFrontChassis.setPower(0);
        motors.leftRearChassis.setPower(0);
        motors.rightRearChassis.setPower(0);
    }
}