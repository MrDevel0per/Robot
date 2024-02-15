package org.firstinspires.ftc.teamcode.robotModel;

import static org.firstinspires.ftc.teamcode.util.OutputUtils.print;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class Chassis {

    //MARK: Attributes
    final double TICKS_PER_INCH = 45.28479;
    private final Telemetry telemetry;
    private final DcMotor leftFront;
    private final DcMotor rightFront;
    private final DcMotor leftRear;
    private final DcMotor rightRear;

    private final Encoder leftEncoder;
    private final Encoder rightEncoder;
    private final Encoder sidewaysEncoder;


    //MARK: Constructors

    public Chassis(HardwareMap hardwareMap, Telemetry telemetry, Encoder leftEncoder, Encoder rightEncoder, Encoder sidewaysEncoder) {
        this.telemetry = telemetry;
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // NOTE: We don't need to set the direction of the encoders because we already did that in the Robot class
        this.rightEncoder = rightEncoder;
        this.leftEncoder = leftEncoder;
        this.sidewaysEncoder = sidewaysEncoder;
    }


    //MARK: Instance Methods - Things that the Chassis class can do

    // MARK: Positioning Methods with Encoders
    int getCurrentPosition() {
        return (leftEncoder.getCurrentPosition() + rightEncoder.getCurrentPosition()) / 2;
    }

    int getSidewaysPosition() {
        return sidewaysEncoder.getCurrentPosition();
    }

    double getSidewaysMovement(int startingPosition) {
        return getSidewaysPosition() - startingPosition;
    }

    double getStraightMovement(int startingPosition) {
        return getCurrentPosition() - startingPosition;
    }


    /**
     * @param distance: distance in inches
     * @param power:    power from -1 to 1
     * @see org.firstinspires.ftc.teamcode.robotModel.Robot
     */
    public void driveStraight(double distance, double power, boolean shouldStop) {
        // First, get the current position
        int startingPosition = getCurrentPosition();
        //definitely not finished!!!
        //get current ticks
//        int ticksStart = rightRear.getCurrentPosition();
//        int ticksToGo = (int) (distance * TICKS_PER_INCH);
        //start motors
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);
        //enter a loop that checks for current distance
        int ticksTravelled = 0;
        // TODO: Refactor to reduce overhead with calling getStraightMovement?
        while (getStraightMovement(startingPosition) > distance) {
            print("Distance Travelled", getStraightMovement(startingPosition));
        }

        //stop motors
        stop(shouldStop);

    }


    public void driveStraight(double distance, double power) {
        driveStraight(distance, power, true);
    }

    public void pointTurn(int angle, double power, boolean shouldStop) {
        double diameter = Math.sqrt(Math.pow(15, 2) + Math.pow(15.2, 2));
        double distance = ((diameter * Math.PI / 360) * angle);
        int startingPosition = rightEncoder.getCurrentPosition();
        //start motors
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftRear.setPower(power);
        rightRear.setPower(-power);
        //enter a loop that checks for current distance
        // TODO: Refractor to abstract logic
        int amountTraveled = 0;
        while (amountTraveled < distance) {
            amountTraveled = Math.abs(rightRear.getCurrentPosition() - startingPosition);
            //change telemetry to angle turned
            print("Distance Travelled (ft)", amountTraveled / 12);
        }
        //stop motors
        stop(shouldStop);
    }

    public void pointTurn(int angle, double power) {
        pointTurn(angle, power, true);
    }

    public void strafeRight(double distance, double power, boolean shouldStop) {
        int startingPosition = getSidewaysPosition();

        //definitely not finished!!!
        //start motors
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftRear.setPower(-power);
        rightRear.setPower(power);
        //enter a loop that checks for current distance
        // TODO: Update more or less based on encoder orientation (if forward if left or right)
        // TODO: Refactor to reduce overhead with calling getStraightMovement?
        while (getSidewaysMovement(startingPosition) < distance) {
            print("Distance Travelled", getSidewaysMovement(startingPosition));
        }

        //stop motors
        stop(shouldStop);
    }

    public void strafeRight(double distance, double power) {
        strafeRight(distance, power, true);
    }

    public void strafeLeft(double distance, double power, boolean shouldStop) {
        //definitely not finished!!!
        int startingPosition = getSidewaysPosition();
        //start motor
        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(-power);
        //enter a loop that checks for current distance
        int ticksTravelled = 0;
        // TODO: Update more or less based on encoder orientation (if forward if left or right)
        // TODO: Refactor to reduce overhead with calling getStraightMovement?
        while (getSidewaysMovement(startingPosition) < distance) {
            print("Distance Travelled", getSidewaysMovement(startingPosition));
        }
        stop(shouldStop);
    }

    public void strafeLeft(double distance, double power) {
        strafeLeft(distance, power, true);
    }

    private void stop(boolean shouldStop) {
        if (shouldStop) {
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
        }
    }

}
