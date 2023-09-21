package org.firstinspires.ftc.teamcode.robotModel;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Chassis {

    //MARK: Attributes
    final double TICKS_PER_INCH = 45.28479;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

    //MARK: Constructors

    public Chassis(HardwareMap hardwareMap, Telemetry telemetry) {
        leftFront = hardwareMap.get(DcMotor.class,"left_front");
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront = hardwareMap.get(DcMotor.class,"right_front");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear = hardwareMap.get(DcMotor.class,"left_rear");
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear = hardwareMap.get(DcMotor.class,"right_rear");
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    //MARK: Instance Methods - Things that the Chassis class can do

    public void driveStraight(double distance,double power){
        //definitely not finished!!!
        //get current ticks
        int ticksSoFar = rightRear.getCurrentPosition();
        int ticksToGo = (int) (distance*TICKS_PER_INCH);
        //start motors
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);
        //enter a loop that checks for current distance
        while(ticksToGo>(rightRear.getCurrentPosition()-ticksSoFar)){
            telemetry.addData("distance travelled",(rightRear.getCurrentPosition()-ticksSoFar)/TICKS_PER_INCH);
        }

        //stop motors
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

    }

}
