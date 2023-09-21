package org.firstinspires.ftc.teamcode.robotModel;

import org.firstinspires.ftc.teamcode.robotModel.Chassis;
import org.firstinspires.ftc.teamcode.robotModel.Intake;
import org.firstinspires.ftc.teamcode.robotModel.Launcher;

public class Robot {

    //attributes
    private Chassis chassis;
    private Intake intake;
    private Launcher launcher;

    //contstructor

    public Robot() {
        this.chassis = new Chassis();
        this.intake = intake;
        this.launcher = launcher;
    }

    //instance methods - things that robot can do
    public void driveStraight(double distance, double power){
        chassis.driveStraight(distance, power);
    }

}
