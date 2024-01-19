package org.firstinspires.ftc.teamcode.robotModel;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    // Motors for controlling rotate forward/backward
    private DcMotorEx leftRotator;
    private DcMotorEx rightRotator;

    // Motor for controlling up/down
    private DcMotorEx upDownMotor;

    // Motor for controlling forward/backward of shovel
    private CRServo leftFrontShovel;
    private CRServo rightFrontShovel;
    private CRServo leftRearShovel;
    private CRServo rightRearShovel;

    private HardwareMap hardwareMap;

    public Arm(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.leftRotator = hardwareMap.get(DcMotorEx.class, "left_rotator");
        this.rightRotator = hardwareMap.get(DcMotorEx.class, "right_rotator");
        this.upDownMotor = hardwareMap.get(DcMotorEx.class, "up_down_motor");
        this.leftFrontShovel = hardwareMap.get(CRServo.class, "left_front_shovel");
        this.rightFrontShovel = hardwareMap.get(CRServo.class, "right_front_shovel");
        this.leftRearShovel = hardwareMap.get(CRServo.class, "left_rear_shovel");
        this.rightRearShovel = hardwareMap.get(CRServo.class, "right_rear_shovel");
    }

    public void rotate(double power) {
        leftRotator.setPower(power);
        rightRotator.setPower(power);
    }

    public void upDown(double power) {
        upDownMotor.setPower(power);
    }

    public void shovel(double power) {
        // The input power is from -1 to 1
        // Convert it to 0 to 1 - -1 is now 0, 0 is now .5, and 1 is now 1
        power = (power + 1) / 2;
        // Continuously rotate servos
        leftFrontShovel.setPower(power);
        rightFrontShovel.setPower(power);
        leftRearShovel.setPower(power);
        rightRearShovel.setPower(power);

    }
}
