package SushiFrcLib.Sensors.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Defines an structure for Gyroscope classes to follow.
 */
public abstract class Gyro {
    public abstract Rotation2d getAngle();

    public abstract void zeroGyro();

    public abstract void setAngle(Rotation2d newAngle);

    public abstract double getRoll();

    public abstract double getPitch();

    public abstract double getYaw();

}
