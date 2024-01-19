# Motor Names
## Wheel Motors
- Left Front: `left_front`
- Right Front: `right_front`
- Left Rear: `left_rear`
- Right Rear: `right_rear`
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
``
- Left Rotator: `left_rotator`
- Right Rotator: `right_rotator`
- Up Down Motor: `up_down_motor`
- Left Front Shovel Servo: `left_front_shovel`
- Right Front Shovel Servo: `right_front_shovel`
- Left Rear Shovel Servo: `left_rear_shovel`
- Right Rear Shovel Servo: `right_rear_shovel`