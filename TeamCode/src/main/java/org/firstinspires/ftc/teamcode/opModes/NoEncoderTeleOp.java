package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotModel.NoEncoderRobot;

/**
 * This class represents the teleoperation mode of the robot.
 */
@TeleOp(name = "USE THIS ONE (STATES)", group = "Linear OpMode")
public class NoEncoderTeleOp extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private NoEncoderRobot robot;
    Boolean alreadyRotated = false;

    @Override
    public void runOpMode() {
        this.robot = new NoEncoderRobot(hardwareMap, telemetry);

        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            handleRobotMovement();
            handleArmMovement();
        }
        robot.stop();

    }

    /**
     * Handles the movement of the robot based on GamePad 1 input.
     */
    void handleRobotMovement() {
        robot.handleDriving(gamepad1);
    }

    /**
     * Handles the movement of the robot arm based on GamePad 2 input.
     */
    void handleArmMovement() {
        // Use the y value of the right stick to determine rotation
        double armRotation = gamepad2.right_stick_y;
        // Use function 1 - e^(1/1-t)
//        armRotation = 1 - Math.pow(Math.E, 1 / (1 - armRotation));
        double MAX_UP_POWER = 0.32;
        double MAX_DOWN_POWER = 0.32 * -1;
        double armRotationPower = Range.clip(armRotation, MAX_DOWN_POWER, MAX_UP_POWER);
        if (armRotationPower != 0) {
            telemetry.addData("Power: ", armRotationPower);
            telemetry.update();
            if (!alreadyRotated) {
                alreadyRotated = true;
            }
            robot.rotateArmPlayer(armRotationPower);
        } else {
            if (alreadyRotated) {
                robot.rotateArmPlayer(0);
            }
        }

        // Have set positions where the arm holds its location

        // Control claw angle with the y value of the left stick
        double clawRotation = gamepad2.left_stick_y;
        double clawRotationPower = Range.clip(clawRotation, -1.0, 1.0);
        robot.clawRotator(clawRotationPower);

        // Servos open/close controlled by X and B
        boolean grip = gamepad2.left_bumper;
        boolean unGrip = gamepad2.right_bumper;

        if (grip) {
            robot.grip();
        }
        if (unGrip) {
            robot.unGrip();
        }
    }
}