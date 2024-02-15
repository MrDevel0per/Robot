package org.firstinspires.ftc.teamcode.robotModel;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
Stores all of the motors on the robot
 */
public class Motors {
    // MARK: Chassis Motors
    public DcMotor leftFrontChassis;
    public DcMotor rightFrontChassis;
    public DcMotor leftRearChassis;
    public DcMotor rightRearChassis;
    // MARK: Arm Motors
    public DcMotor leftRotator;
    public DcMotor rightRotator;
    public DcMotor clawRotator;

    public Motors(@NonNull HardwareMap hardwareMap) {
        this.leftFrontChassis = hardwareMap.get(DcMotor.class, "left_front");
        this.rightFrontChassis = hardwareMap.get(DcMotor.class, "right_front");
        this.leftRearChassis = hardwareMap.get(DcMotor.class, "left_rear");
        this.rightRearChassis = hardwareMap.get(DcMotor.class, "right_rear");
        this.leftRotator = hardwareMap.get(DcMotor.class, "left_rotator");
        this.rightRotator = hardwareMap.get(DcMotor.class, "right_rotator");
        this.clawRotator = hardwareMap.get(DcMotor.class, "claw_rotator");
        leftFrontChassis = hardwareMap.get(DcMotor.class, "left_front");
        leftFrontChassis.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontChassis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontChassis = hardwareMap.get(DcMotor.class, "right_front");
        rightFrontChassis.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontChassis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearChassis = hardwareMap.get(DcMotor.class, "left_rear");
        leftRearChassis.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRearChassis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearChassis = hardwareMap.get(DcMotor.class, "right_rear");
        rightRearChassis.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearChassis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftRotator = hardwareMap.get(DcMotor.class, "left_rotator");
        leftRotator.setDirection(DcMotor.Direction.FORWARD);
        this.rightRotator = hardwareMap.get(DcMotor.class, "right_rotator");
        rightRotator.setDirection(DcMotor.Direction.REVERSE);
        this.clawRotator = hardwareMap.get(DcMotor.class, "claw_rotator");
        clawRotator.setDirection(CRServo.Direction.FORWARD);
    }
}
