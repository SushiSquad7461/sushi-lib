package SushiFrcLib.Math;

import java.lang.Math;

/**
 * Linearize helpers.
 */
public class Normalization {
    public static double cube(double triggerVal) {
        return Math.pow(triggerVal, 3);
    }

    /**
     * Apply deadzone.
     */
    public static double linearDeadzone(double triggerVal, double deadzone) {
        return Math.abs(triggerVal) < deadzone ? 0 : (
            (triggerVal - ((triggerVal < 0 ? -1 : 1) * deadzone)) 
            / (1 - deadzone));
    }

    public static double logistic(double triggerVal, double k) {
        return 2 / (1 + (Math.pow((Math.E), -k * (triggerVal)))) - 1;
    }
}