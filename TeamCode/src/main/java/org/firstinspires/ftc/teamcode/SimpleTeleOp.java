package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SimpleTeleOp extends LinearOpMode {

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
