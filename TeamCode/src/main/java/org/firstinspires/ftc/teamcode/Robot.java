package org.firstinspires.ftc.teamcode;

import org.checkerframework.checker.units.qual.C;

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
