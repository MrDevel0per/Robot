package org.firstinspires.ftc.teamcode.robotModel;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class Arm {
    private final Motors motors;
    private final Servos servos;
    private final Lock lock = new ReentrantLock();

    private boolean isScheduled = false;

    public boolean isHolding = true;
    private volatile boolean shouldStop = false;

    private ScheduledExecutorService execService;
    private Telemetry telemetry;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.motors = new Motors(hardwareMap);
        this.servos = new Servos(hardwareMap);
        this.telemetry = telemetry;
    }

    public void grip() {
        servos.rightClawServo.setPosition(1.0);
        servos.leftClawServo.setPosition(1.0);
    }

    public void unGrip() {
        servos.rightClawServo.setPosition(0.0);
        servos.leftClawServo.setPosition(0.0);
    }

    public int getArmRotation() {
        return (motors.leftRotator.getCurrentPosition() + motors.rightRotator.getCurrentPosition()) / 2;
    }

    public void stop() {
        motors.leftRotator.setPower(0);
        motors.rightRotator.setPower(0);
    }

    public void clawRotate(double power) {
        servos.clawRotator
    }

    public enum Position {
        GROUND(10),
        FIRST_LINE(20),
        SECOND_LINE(30),
        TRANSPORT(40);

        public final int DEGREES_OF_360;

        Position(int degrees) {
            this.DEGREES_OF_360 = degrees;
        }
    }

     void rotateArmToDesiredPos(int desiredPosition) {
        double TICKS_PER_360_DEGREES = 537.7;
        int allowedError = (int) ((5 / 360) * (TICKS_PER_360_DEGREES));
        int currentMotorPosition = this.getArmRotation();
        int difference = desiredPosition - currentMotorPosition;
        // Found as good holding power
        double power = 0.2;
        if (difference > allowedError) {
            motors.rightRotator.setPower(power);
            motors.leftRotator.setPower(power);
            rotateArmToDesiredPos(desiredPosition);
        } else if (difference < -allowedError) {
            motors.rightRotator.setPower(-power);
            motors.leftRotator.setPower(-power);
            rotateArmToDesiredPos(desiredPosition);
        } else {
            motors.rightRotator.setPower(0);
            motors.leftRotator.setPower(0);
        }
    }

    public void _hold() {
        if (!isHolding || shouldStop) {
            execService.shutdown();
            isScheduled = false;
            return;
        }
        telemetry.addData("DOING IT!!", "Working now");
        telemetry.update();
        int startingMotorPosition = getArmRotation();
        int currentMotorPosition = getArmRotation();
        rotateArmToDesiredPos(currentMotorPosition);

    }

    public void hold() {
        lock.lock();
        try {
            if (this.execService == null || this.execService.isShutdown()) {
                this.execService = Executors.newSingleThreadScheduledExecutor();
            }
            _hold();
            if (!isScheduled) {
                execService.scheduleAtFixedRate(this::_hold, 0, 500, TimeUnit.MILLISECONDS);
                isScheduled = true;
            }
        } finally {
            lock.unlock();
        }
    }


    public void start(){
        servos.droneLauncher.setPosition(0.5);
    }
    public void droneLaunch(){
        servos.droneLauncher.setPosition(1);
    }
    public void armRotate(double power) {

        motors.leftRotator.setPower(power);
        motors.rightRotator.setPower(power);
        if (power == 0) {
            motors.leftRotator.setPower(-0.15);
            motors.rightRotator.setPower(-0.15);
        }
    }
    }
