package SushiFrcLib.SwerveModule;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import SushiFrcLib.Motor.SwerveModuleConfig;

public class SwerveModuleFalcon extends SwerveModule {
    private final WPI_TalonFX drive;
    private final WPI_TalonFX turn;
    private final TalonFXInvertType defaultInversion;

    public SwerveModuleFalcon(SwerveModuleConfig config) {
        super(config);
        drive = config.driveMotor.getFalconMotor();
        turn = config.turnMotor.getFalconMotor();
        defaultInversion = config.turnMotor.inversion.getFalconDirection();
    }

    public SwerveModuleFalcon(SwerveModuleConfig config, double errorBound, double turnSpeed) {
        super(config, errorBound, turnSpeed);
        drive = config.driveMotor.getFalconMotor();
        turn = config.turnMotor.getFalconMotor();
        defaultInversion = config.turnMotor.inversion.getFalconDirection();
    }

    @Override
    protected void drive(double velocity) {
        drive.set(ControlMode.Velocity, velocity); 
    }

    @Override
    protected void turn(double angle, int inversion) {
        turn.setInverted(inversion == -1 ? (defaultInversion == TalonFXInvertType.Clockwise ? TalonFXInvertType.CounterClockwise : TalonFXInvertType.Clockwise) : defaultInversion);
        turn.set(ControlMode.Position, angle);
    }
}
