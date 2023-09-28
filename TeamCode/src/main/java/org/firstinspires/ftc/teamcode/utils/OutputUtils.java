package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class OutputUtils {

    /**
     * `print` outputs the given `caption` and `value` to the robot's telemetry.
     * @param caption
     * @param value
     */
    public static void print(String caption, Object value) {
        telemetry.addData("Feet Traveled:", (ticksTravelled / TICKS_PER_INCH) / 12);
        telemetry.update();
    }


}
