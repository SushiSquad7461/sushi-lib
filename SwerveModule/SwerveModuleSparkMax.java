package SushiFrcLib.SwerveModule;

import com.revrobotics.CANSparkMax;

import SushiFrcLib.Math.Conversion;
import SushiFrcLib.Motor.SwerveModuleConfig;
import edu.wpi.first.math.controller.PIDController;

public class SwerveModuleSparkMax extends SwerveModule {
    private final CANSparkMax drive;
    private final CANSparkMax turn;
    private final PIDController pidController;
    private final boolean defaultInversion;

    public SwerveModuleSparkMax(SwerveModuleConfig config) {
        super(config);
        drive = config.driveMotor.getSparkMax();
        turn = config.turnMotor.getSparkMax();
        defaultInversion = turn.getInverted();
        pidController = config.turnMotor.getWpiPid();
    }

    public SwerveModuleSparkMax(SwerveModuleConfig config, double errorBound, double turnSpeed) {
        super(config, errorBound, turnSpeed);
        drive = config.driveMotor.getSparkMax();
        turn = config.turnMotor.getSparkMax();
        defaultInversion = turn.getInverted();
        pidController = config.turnMotor.getWpiPid();
    }

    @Override
    protected void drive(double velocity) {
        drive.getPIDController().setReference(velocity, CANSparkMax.ControlType.kVelocity, 0 /* pid value slot */, drive.getPIDController().getFF() /** feed forward? double check that this is correct */);
    }

    @Override
    protected void turn(double angle, int inversion) {
        turn.setInverted(inversion == -1 ? !defaultInversion : defaultInversion);
        double error = Conversion.normalizeAngle(angle - getModuleAngle());
        double output = pidController.calculate(error, 0);
        turn.set(output);
    }

    @Override
    protected void setTurn(double speed) {
        turn.set(0);        
    }

    @Override
    protected void setDrive(double speed) {
        turn.set(0);        
    }
}
