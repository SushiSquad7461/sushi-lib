package libraries.SushiFrcLib.Math;

public class Conversion {
    // Convert ticks to meters
    public static double ticksToMeters(double ticks, double wheelDiamater) {
        return (ticks / 2048.0) * ((wheelDiamater * Math.PI) / 39.37);
    }

    // Return a value of currAngle between -180 and 180
    public static double normalizeGyro(double currAngle) {
        if (currAngle < -180) {
            return 180 + (currAngle + 180);
        } else if (currAngle > 180) {
            return -180 + (currAngle - 180);
        } else {
            return currAngle;
        }
    }

    // Normalize angle between 0 360
    public static double normalizeAngle(double currAngle) {
        if (currAngle < 0) {
            return 360 + currAngle;
        } else if (currAngle > 360) {
            return currAngle - 360;
        } else {
            return currAngle;
        }
    }
}
