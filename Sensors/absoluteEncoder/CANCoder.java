package SushiFrcLib.Sensors.absoluteEncoder;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;


public class CANCoder {
    private CANcoder encoder;

    public CANCoder(int id, Rotation2d offset, SensorDirectionValue inverted) {
        this.encoder = new CANcoder(id);

        CANcoderConfiguration config = new CANcoderConfiguration();
        
        //setting discontinuity point to 1.0 for unsigned [0,1) range
        config.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(1.0)
                          .withSensorDirection(inverted)
                          .withMagnetOffset(-offset.getRotations());

        encoder.getConfigurator().apply(config);
    }

    public Rotation2d getPosition() {
        return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValue().in(Units.Rotations));
    }
}
