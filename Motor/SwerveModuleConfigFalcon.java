package SushiFrcLib.Motor;

public class SwerveModuleConfigFalcon {
    public final MotorConfigFalcon driveMotor;
    public final MotorConfigFalcon turnMotor;
    public final int encoderId;
    public final double angleOfset;

    public SwerveModuleConfigFalcon(MotorConfigFalcon driveMotor, MotorConfigFalcon turnMotor, int encoderId, double ofset) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.encoderId = encoderId;
        this.angleOfset = ofset;
    }
}