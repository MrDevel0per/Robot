package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotModel.Robot;

@Autonomous(name="Simple Auto", group="Linear OpMode")
//@Disabled
public class SimpleAuto extends LinearOpMode {
    public Robot robot = new Robot(hardwareMap, telemetry);
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
