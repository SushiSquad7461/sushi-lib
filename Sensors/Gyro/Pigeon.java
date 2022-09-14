package SushiFrcLib.Sensors.Gyro;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import SushiFrcLib.Constants.SushiConstants;
import SushiFrcLib.Math.Conversion;

public class Pigeon extends Gyro {
    private final WPI_Pigeon2 gyro;
    private static Pigeon instance;

    public static Pigeon getInstance() {
        if (instance == null) {
            instance = new Pigeon();
        }
        return instance;
    }

    private Pigeon() {
        gyro = new WPI_Pigeon2(SushiConstants.Ports.PIGEON);
    }

    @Override
    public void zero() {
        gyro.setYaw(0);
    }

    @Override
    public double getAngle() {
        return Conversion.normalizeAngle(gyro.getYaw());
    }
}
