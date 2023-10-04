package org.firstinspires.ftc.teamcode.robotModel;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.utils.OutputUtils.print;

public class Chassis {

    //MARK: Attributes
    final double TICKS_PER_INCH = 45.28479;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;

    //MARK: Constructors

    public Chassis(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        leftFront = hardwareMap.get(DcMotor.class,"left_front");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront = hardwareMap.get(DcMotor.class,"right_front");
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear = hardwareMap.get(DcMotor.class,"left_rear");
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear = hardwareMap.get(DcMotor.class,"right_rear");
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    //MARK: Instance Methods - Things that the Chassis class can do

    public void driveStraight(double distance,double power){
        //definitely not finished!!!
        //get current ticks
        int ticksStart = rightRear.getCurrentPosition();
        int ticksToGo = (int) (distance*TICKS_PER_INCH);
        //start motors
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);
        //enter a loop that checks for current distance
        int ticksTravelled=0;
        while(ticksToGo>ticksTravelled) {
            ticksTravelled = Math.abs(rightRear.getCurrentPosition() - ticksStart);
            telemetry.addData("Distance Travelled",(ticksTravelled / TICKS_PER_INCH) / 12);
            telemetry.update();
        }

        //stop motors
        stop();

    }
    public void pointTurn(int angle, double power){
        double diameter = Math.sqrt(Math.pow(15 , 2)+Math.pow(15.2 , 2));
        double distance = ((diameter * Math.PI/360) * angle);
        int ticksStart = rightRear.getCurrentPosition();
        int ticksToGo = (int) (distance * TICKS_PER_INCH);
        //start motors
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftRear.setPower(power);
        rightRear.setPower(-power);
        //enter a loop that checks for current distance
        int ticksTravelled=0;
        while(ticksToGo>ticksTravelled) {
            ticksTravelled = Math.abs(rightRear.getCurrentPosition() - ticksStart);
            //change telemetry to angle turned
            telemetry.addData("Distance Turned",(ticksTravelled / TICKS_PER_INCH) / 12);
            telemetry.update();
    }
        //stop motors
        stop();
}
    public void strafeRight(double distance,double power) {
        //definitely not finished!!!
        //get current ticks
        int ticksStart = rightRear.getCurrentPosition();
        int ticksToGo = (int) (distance * TICKS_PER_INCH);
        //start motors
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftRear.setPower(-power);
        rightRear.setPower(power);
        //enter a loop that checks for current distance
        int ticksTravelled = 0;
        while (ticksToGo > ticksTravelled) {
            ticksTravelled = Math.abs(rightRear.getCurrentPosition() - ticksStart);
            telemetry.addData("Distance Travelled", (ticksTravelled / TICKS_PER_INCH) / 12);
            telemetry.update();
        }

        //stop motors
        stop();
    }
    public void strafeLeft(double distance,double power) {
        //definitely not finished!!!
        //get current ticks
        int ticksStart = rightRear.getCurrentPosition();
        int ticksToGo = (int) (distance * TICKS_PER_INCH);
        //start motor
        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(-power);
        //enter a loop that checks for current distance
        int ticksTravelled = 0;
        while (ticksToGo > ticksTravelled) {
            ticksTravelled = Math.abs(rightRear.getCurrentPosition() - ticksStart);
            telemetry.addData("Distance Travelled", (ticksTravelled / TICKS_PER_INCH) / 12);
            telemetry.update();
        }

        //stop motors
        stop();
    }
    private void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

}
