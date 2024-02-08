package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotModel.NoEncoderRobot;
import org.firstinspires.ftc.teamcode.robotModel.Robot;

@TeleOp(name="USE THIS ONE (January 18)", group="Linear OpMode")

public class NoEncoderTeleOp extends LinearOpMode {
    private NoEncoderRobot robot;
    private ElapsedTime runtime = new ElapsedTime();



    @Override
            public void runOpMode() {

        this.robot = new NoEncoderRobot(hardwareMap, telemetry);

    waitForStart();
    runtime.reset();

    // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            handleRobotMovement();
            handleArmMovement();

    }
    }

    void handleRobotMovement() {
        if (gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0 && gamepad1.left_stick_x == 0) {
            robot.stop();
        } else {
            double drive = gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            double leftFrontPower    = Range.clip(drive - turn -strafe, -0.75, 0.75) ;
            double  rightFrontPower   = Range.clip(drive + turn + strafe, -0.75, 0.75) ;
            double leftBackPower    = Range.clip(drive - turn + strafe, -0.75, 0.75) ;
            double rightBackPower   = Range.clip(drive + turn - strafe, -0.75, 0.75) ;
            robot.setMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
        }
    }

    void handleArmMovement() {

        //Gamepad 1 Controls

        // Use the y value of the right stick to determine rotation
        double armRotation = gamepad1.right_stick_y;
        double armRotationPower = Range.clip(armRotation, -1.0, 1.0);
        robot.rotateArm(armRotationPower);


        // Use the y button to move up and x button to move down
        boolean yValue = gamepad1.y;
        boolean aValue = gamepad1.a;
        if (yValue ^ aValue) {
            if (yValue) {
                robot.upDownArm(1);
            } else {
                robot.upDownArm(-1);
            }
        } else {
            robot.upDownArm(0);
        }



        /*boolean clawRotationForward = gamepad1.x;
        boolean clawRotationBackwards = gamepad1.b;
        if (clawRotationForward){
            robot.forward();
        }
        if (clawRotationBackwards){
            robot.backwards();
        }*/



        //Gamepad 2 Controls

        //Have set positions where the arm holds its location
        //Posibility of having the claw and the arm both rotate with one button press.
        boolean armGround = gamepad2.;
        boolean armTranport = gamepad2.;
        boolean armDrop1stLine = gamepad2.;
        boolean armDrop2ndLine =gamepad2.;

        //Control claw angle with the y value of the left stick
        double clawRotation = gamepad2.left_stick_y;
        double clawRotationPower = Range.clip(clawRotation, -1.0, 1.0);
        robot.clawRotator(clawRotationPower);

        // Servos open/close controlled by X and B
        boolean grip = gamepad2.x;
        boolean unGrip = gamepad2.b;

        if (grip){
            robot.grip();
        }
        if (unGrip){
            robot.unGrip();
        }




        /*boolean xValue = gamepad1.x;
        boolean bValue = gamepad1.b;

        if (xValue ^ bValue) {
            if (xValue) {
                robot.shovelArm(1);
            } else {
                robot.shovelArm(-1);
            }
        } else {
            robot.shovelArm(0);
        }*/
    }
}
