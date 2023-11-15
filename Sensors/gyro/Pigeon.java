package SushiFrcLib.Sensors.gyro;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Encapsulates pigeon gyroscope.
 */
public class Pigeon extends Gyro {
    private final Pigeon2 gyro;
    private final boolean inversion;

    /**
     * Set id and inversion of pigeon.
     */
    public Pigeon(int id, boolean inversion) {
        gyro = new Pigeon2(id);
        gyro.configFactoryDefault();
        this.inversion = inversion;
    }

    /**
     * Set id, inversion, and canivore name of pigeon.
     */
    public Pigeon(int id, boolean inversion, String canBusName) {
        gyro = new Pigeon2(id, canBusName);
        gyro.configFactoryDefault();
        this.inversion = inversion;
    }

    @Override
    public Rotation2d getAngle() {
        double yaw = getYaw();

        return inversion 
            ? Rotation2d.fromDegrees(360 - yaw)
            : Rotation2d.fromDegrees(yaw);
    }

    private double getYaw() {
        return gyro.getYaw();
    }

    // TODO why aren't these in the base class?
    public double getPitch() {
        return gyro.getPitch();
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    @Override
    public void zeroGyro() {
        gyro.setYaw(0);
    }

    @Override
    public void setAngle(Rotation2d newAngle) {
        gyro.setYaw(newAngle.getDegrees()); 
    }

}
