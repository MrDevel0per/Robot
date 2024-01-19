package org.firstinspires.ftc.teamcode.robotModel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class NoEncoderChassis {
    private Telemetry telemetry;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

    private HardwareMap hardwareMap;

    public NoEncoderChassis(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        leftFront = hardwareMap.get(DcMotor.class,"left_front");
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront = hardwareMap.get(DcMotor.class,"right_front");
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear = hardwareMap.get(DcMotor.class,"left_rear");
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear = hardwareMap.get(DcMotor.class,"right_rear");
        rightRear.setDirection(DcMotorEx.Direction.FORWARD);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void driveStraight(double power) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);
    }

    public void turn(double power) {
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftRear.setPower(power);
        rightRear.setPower(-power);
    }

    public void strafeLeft(double power) {
        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(-power);
    }

    public void strafeRight(double power) {
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftRear.setPower(-power);
        rightRear.setPower(power);
    }

    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    public void setMotorPowers(double leftFrontPower, double rightFrontPower, double leftRearPower, double rightRearPower) {
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);
    }
}
