package org.firstinspires.ftc.teamcode.robotModel;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class NoEncoderRobot {
    public NoEncoderChassis chassis;
    private final Arm arm;


    public NoEncoderRobot(HardwareMap hardwareMap) {
        this.chassis = new NoEncoderChassis(hardwareMap);
        this.arm = new Arm(hardwareMap);
    }

    public void handleDriving(Gamepad gamepad) {
        chassis.handleDriving(gamepad);
    }

    public void stop() {
        chassis.stop();
    }

    public void rotateArmPlayer(double power) {
        arm.armRotate(power);
    }

    public void rotateArmButton1() {
        arm.rotateArm(Arm.Position.GROUND);
    }

    public void rotateArmButton2() {
        arm.rotateArm(Arm.Position.TRANSPORT);
    }

    public void rotateArmButton3() {
        arm.rotateArm(Arm.Position.FIRST_LINE);
    }

    public void rotateArmButton4() {
        arm.rotateArm(Arm.Position.SECOND_LINE);
    }

    public void grip() {
        arm.grip();
    }

    public void unGrip() {
        arm.unGrip();
    }

    public void clawRotator(double power) {
        arm.clawRotate(power);
    }
}
