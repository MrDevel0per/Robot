# Motor Names
## Wheel Motors
- Left Front: `left_front`: Port `2` on the Expansion Hub
- Right Front: `right_front`: Port `3` on the Control Hub
- Left Rear: `left_rear`: Port `0` on the Expansion Hub
- Right Rear: `right_rear`: Port `1` on the Expansion Hub
## Arm Motors
See this code:
```java
this.leftRotator = hardwareMap.get(DcMotorEx.class, "left_rotator");
        this.rightRotator = hardwareMap.get(DcMotorEx.class, "right_rotator");
        this.upDownMotor = hardwareMap.get(DcMotorEx.class, "up_down_motor");
        this.leftFrontShovel = hardwareMap.get(Servo.class, "left_front_shovel");
        this.rightFrontShovel = hardwareMap.get(Servo.class, "right_front_shovel");
        this.leftRearShovel = hardwareMap.get(Servo.class, "left_rear_shovel");
        this.rightRearShovel = hardwareMap.get(Servo.class, "right_rear_shovel");
```
- Left Rotator: `left_rotator`: Port `2` on the Control Hub
- Right Rotator: `right_rotator`: Port `1` on the Control Hub
- Up Down Motor: `up_down_motor`: Port `3` on the Expansion Hub
## Shovel Servos
- Left Front Shovel Servo: `left_front_shovel`
- Right Front Shovel Servo: `right_front_shovel`
- Left Rear Shovel Servo: `left_rear_shovel`
- Right Rear Shovel Servo: `right_rear_shovel`