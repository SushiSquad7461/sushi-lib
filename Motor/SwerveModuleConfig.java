package SushiFrcLib.Motor;

public class SwerveModuleConfig {
    public final MotorConfig driveMotor;
    public final MotorConfig turnMotor;
    public final int encoderId;
    public final double angleOfset;

    public SwerveModuleConfig(MotorConfig driveMotor, MotorConfig turnMotor, int encoderId, double ofset) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.encoderId = encoderId;
        this.angleOfset = ofset;
    }
}