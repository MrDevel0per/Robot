package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotModel.Robot;

@Autonomous(name = "Simple Auto", group = "Linear OpMode")
//@Disabled
public class SimpleAuto extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry);

        waitForStart();
        runtime.reset();

        // Call our loop
        while (opModeIsActive()) {
//            // Drive forward for 3 seconds
//            robot.driveStraight(0.5);
//            sleep(3000);
//            robot.stop();
        }

    }

}
