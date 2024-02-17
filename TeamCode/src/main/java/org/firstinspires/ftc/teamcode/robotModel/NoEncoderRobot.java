package org.firstinspires.ftc.teamcode.robotModel;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class NoEncoderRobot {
    public NoEncoderChassis chassis;
    private final Arm arm;
    private Telemetry telemetry;


    public NoEncoderRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.chassis = new NoEncoderChassis(hardwareMap);
        this.arm = new Arm(hardwareMap, telemetry);
    }

    public void handleDriving(Gamepad gamepad) {
        chassis.handleDriving(gamepad);
    }

    public void stop() {
        chassis.stop();
        arm.stop();
    }

    public void rotateArmPlayer(double power) {
        arm.armRotate(power);
    }


    public void droneLaunch(){
        arm.droneLaunch();
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
