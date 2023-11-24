package SushiFrcLib.Math;

public class Conversion {
    // Convert ticks to meters
    public static double ticksToMeters(double ticks, double wheelDiamater) {
        return (ticks / 2048.0) * ((wheelDiamater * Math.PI) / 39.37);
    }

    /**
     * Converts RPM to Tran
     * @param RPM motor rpm
     * @return the unit of measurement for Talon FX encoder velocity is known as the "Tran" (encoder ticks per 100ms)
     */
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

    /**
     * @param counts Falcon Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double falconToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Counts
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        double ticks = degrees / (360.0 / (gearRatio * 2048.0));
        return ticks;
    }

    /**
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param RPM RPM of mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * @param velocitycounts Falcon Velocity Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio){
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    public static double falconToM(double distance, double circumference, double gearRatio) {
        double rotations = distance / 2048.0;
        double axleRotations = rotations / gearRatio;
        return axleRotations * circumference;
    }
}
