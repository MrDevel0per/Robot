package org.firstinspires.ftc.teamcode.robotModel;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotModel.Chassis;
import org.firstinspires.ftc.teamcode.robotModel.Intake;
import org.firstinspires.ftc.teamcode.robotModel.Launcher;

public class Robot {

    //attributes
    private Chassis chassis;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private Intake intake;
    private Launcher launcher;
    private DcMotor linearSlide;

    //contstructor

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.chassis = new Chassis(hardwareMap,telemetry);
        this.telemetry=telemetry;
        this.hardwareMap = hardwareMap;
        this.intake = intake;
        this.launcher = launcher;
        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");

    }

    //instance methods - things that robot can do
    public void driveStraight(double distance, double power){
        chassis.driveStraight(distance, power);
    }
    public void pointTurn(int angle, double power) {
        chassis.pointTurn(angle, power);
    }
}
