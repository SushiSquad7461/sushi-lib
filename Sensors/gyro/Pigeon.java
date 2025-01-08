package SushiFrcLib.Sensors.gyro;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
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
        this(id, inversion, "");
    }

    /**
     * Set id, inversion, and canivore name of pigeon.
     */
    public Pigeon(int id, boolean inversion, String canBusName) {
        gyro = new Pigeon2(id, canBusName);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        this.inversion = inversion;
    }

    @Override
    public Rotation2d getAngle() {
        double yaw = getYaw();

        return inversion 
            ? Rotation2d.fromDegrees(MathUtil.inputModulus(360 - yaw, 0, 360))
            : Rotation2d.fromDegrees(MathUtil.inputModulus(yaw, 0, 360));
    }

    public double getYaw() {
        return gyro.getYaw().getValueAsDouble();
    }

    public double getPitch() {
        return gyro.getPitch().getValueAsDouble();
    }

    public double getRoll() {
        return gyro.getRoll().getValueAsDouble();
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
