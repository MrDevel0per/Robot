package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotModel.Robot;

@Autonomous(name="Simple OpMode", group="Linear OpMode")
//@Disabled
public class SimpleAuto extends LinearOpMode {
    public Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        runtime.reset();

        // Call our loop
        while (opModeIsActive()) {
            robot.driveStraight(10,.8);
        }

    }

}
