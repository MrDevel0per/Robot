package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class OutputUtils {

    /**
     * `print` outputs the given `caption` and `value` to the robot's telemetry. We also implimented another verison of
     * `soloPrint`, which has no caption.
     *
     * @param caption The caption to give the output in the console.
     * @param value   The actual value to be outputted.
     */
    public static void print(String caption, Object value) {
        telemetry.addData(caption, value);
        telemetry.update();
    }

    public static void soloPrint(Object value) {
        telemetry.addData("", value);
        telemetry.update();
    }


}