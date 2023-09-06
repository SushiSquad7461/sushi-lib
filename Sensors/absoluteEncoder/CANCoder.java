package SushiFrcLib.Sensors.absoluteEncoder;

import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class CanCoder {
    private double offset;
    private WPI_CANCoder encoder;

    public CanCoder(int id, double offset, boolean inverted) {
        this.offset = offset;
        this.encoder = new WPI_CANCoder(id);

        this.encoder.configFactoryDefault();
        this.encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        this.encoder.configSensorDirection(inverted);
        this.encoder.configSensorInitializationStrategy(
            SensorInitializationStrategy.BootToAbsolutePosition
        );
    }

    public Rotation2d getPosition() {
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition() - offset);
    }
}
