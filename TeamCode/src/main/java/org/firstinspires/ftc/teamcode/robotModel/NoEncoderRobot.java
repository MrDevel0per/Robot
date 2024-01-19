package org.firstinspires.ftc.teamcode.robotModel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class NoEncoderRobot {
    public NoEncoderChassis chassis;
    private Telemetry telemetry;
    private Arm arm;
    private Launcher launcher;

    private HardwareMap hardwareMap;

    public NoEncoderRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.chassis = new NoEncoderChassis(hardwareMap, telemetry);
        this.telemetry=telemetry;
        this.arm = arm;
        this.launcher = launcher;
    }

    public void driveStraight(double power) {
        chassis.driveStraight(power);
    }

    public void turn(double power) {
        chassis.turn(power);
    }

    public void strafeLeft(double power) {
        chassis.strafeLeft(power);
    }

    public void strafeRight(double power) {
        chassis.strafeRight(power);
    }

    public void setMotorPowers(double leftFrontPower, double rightFrontPower, double leftRearPower, double rightRearPower) {
        chassis.setMotorPowers(leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
    }

    public void stop() {
        chassis.stop();
    }

    public void rotateArm(double power) {
        arm.rotate(power);
    }

    public void upDownArm(double power) {
        arm.upDown(power);
    }

    public void shovelArm(double power) {
        arm.shovel(power);
    }
}
