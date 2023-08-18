package SushiFrcLib.Sensors.gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;

public class Navx extends Gyro {
    private final AHRS gyro;

    public Navx() {
        gyro = new AHRS();
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(-gyro.getYaw());
    }

    @Override
    public void zeroGyro() {
        gyro.zeroYaw();
    }

    @Override
    public void setAngle(Rotation2d newAngle) {
        gyro.zeroYaw();
        gyro.setAngleAdjustment(newAngle.getDegrees());
    }
    
}
