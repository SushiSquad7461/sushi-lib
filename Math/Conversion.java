package SushiFrcLib.Math;

public class Conversion {
    // Convert ticks to meters
    public static double ticksToMeters(double ticks, double wheelDiamater) {
        return (ticks / 2048.0) * ((wheelDiamater * Math.PI) / 39.37);
    }

    // the unit of measurement for Talon FX encoder velocity is known as the "Tran"
    // encoder ticks per 100ms
    public static double convertRPMtoTrans(double RPM) {
        return RPM * 2048.0 / 600.0;
    }

    public static double convertTransToRPM(double trans) {
        return trans * 600.0 / 2048.0;
    }

    // used for unit conversions for ff constants on talon fx
    public static double voltageToPercent(double voltage) {
        return voltage / 12.0;
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
        double modules = currAngle % 360;
        return modules < 0 ? 360 + modules : modules;
    }

    public static double degreesToRadians(double angle) {
        return (angle * Math.PI) / 180.0;
    }

    public static double radiansToDegrees(double angle) {
        return (angle * 180.0) / Math.PI;
    }
}
