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

    @Override
    public void runOpMode() {
        this.robot = new NoEncoderRobot(hardwareMap);

        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            handleRobotMovement();
            handleArmMovement();
        }
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
        double armRotationPower = Range.clip(armRotation, -1.0, 1.0);
        robot.rotateArmPlayer(armRotationPower);

        // Have set positions where the arm holds its location
        boolean armGround = gamepad2.x;
        boolean armTransport = gamepad2.y;
        boolean armDrop1stLine = gamepad2.a;
        boolean armDrop2ndLine = gamepad2.b;

        if (armGround) {
            robot.rotateArmButton1();
        }
        if (armTransport) {
            robot.rotateArmButton2();
        }
        if (armDrop1stLine) {
            robot.rotateArmButton3();
        }
        if (armDrop2ndLine) {
            robot.rotateArmButton4();
        }

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