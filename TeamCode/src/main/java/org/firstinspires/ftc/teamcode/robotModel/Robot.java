package org.firstinspires.ftc.teamcode.robotModel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Encoder;

public class Robot {

    //attributes
    private Chassis chassis;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private Arm arm;
    private Launcher launcher;
    private DcMotor linearSlide;

    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private Encoder sidewaysEncoder;

    //contstructor

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        // TODO: Get the proper encoders
        DcMotorEx left_EncoderEx = hardwareMap.get(DcMotorEx.class, "left_front_encoder");
        leftEncoder = new Encoder(left_EncoderEx);
        // Set direction to forward
        // TODO: Correct direction?
        leftEncoder.setDirection(Encoder.Direction.FORWARD);

        DcMotorEx right_EncoderEx = hardwareMap.get(DcMotorEx.class, "right_front_encoder");
        rightEncoder = new Encoder(right_EncoderEx);
        // Set direction to forward
        // TODO: Correct direction?
        rightEncoder.setDirection(Encoder.Direction.FORWARD);

        DcMotorEx sideways_EncoderEx = hardwareMap.get(DcMotorEx.class, "sideways_encoder");
        sidewaysEncoder = new Encoder(sideways_EncoderEx);
        // Set direction to forward
        // TODO: Correct direction?
        sidewaysEncoder.setDirection(Encoder.Direction.FORWARD);

        this.chassis = new Chassis(hardwareMap,telemetry, leftEncoder, rightEncoder, sidewaysEncoder);
        this.telemetry=telemetry;
        this.hardwareMap = hardwareMap;
        this.arm = arm;
        this.launcher = launcher;
        linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    //instance methods - things that robot can do

    /**
    * This method will drive the robot straight for a given distance at a given power.
    * @param distance - distance in inches
    * @param power - power from 0 to 1
     */
    public void driveStraight(double distance, double power){
        chassis.driveStraight(distance, power);
    }
    public void pointTurn(int angle, double power) {
        chassis.pointTurn(angle, power);
    }
    public void strafeRight(double distance, double power){
        chassis.strafeRight(distance, power);
    }
    //darn this gosh darn darn darn darn darn push stuff >:(
    public void strafeLeft(double distance, double power){
        chassis.strafeLeft(distance, power);
    }
}
