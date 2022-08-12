package SushiFrcLib.SwerveModule;

import javax.management.relation.RelationException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import SushiFrcLib.Control.FeedForwardController;
import SushiFrcLib.Math.Conversion;
import SushiFrcLib.Motor.SwerveModuleConfig;
import edu.wpi.first.math.controller.PIDController;

public class SwerveModuleSparkMax extends SwerveModule {
    private final CANSparkMax drive;
    private final CANSparkMax turn;
    private final PIDController turnPidController;
    private final PIDController drivePidController;
    private final RelativeEncoder driveEncoder;
    private final boolean defaultInversion;
    private final FeedForwardController driveFeedForward;

    public SwerveModuleSparkMax(SwerveModuleConfig config) {
        super(config);
        drive = config.driveMotor.getSparkMax();
        turn = config.turnMotor.getSparkMax();
        defaultInversion = turn.getInverted();
        turnPidController = config.turnMotor.getWpiPid();
        drivePidController = config.driveMotor.getWpiPid();
        driveEncoder = drive.getEncoder();
        driveFeedForward = config.driveMotor.getFeedForward();
    }

    public SwerveModuleSparkMax(SwerveModuleConfig config, double errorBound, double turnSpeed) {
        super(config, errorBound, turnSpeed);
        drive = config.driveMotor.getSparkMax();
        turn = config.turnMotor.getSparkMax();
        defaultInversion = turn.getInverted();
        drivePidController = config.driveMotor.getWpiPid();
        turnPidController = config.turnMotor.getWpiPid();
        driveEncoder = drive.getEncoder();
        driveFeedForward = config.driveMotor.getFeedForward();
    }

    @Override
    protected void drive(double velocity) {
        drive.set(
            drivePidController.calculate(driveEncoder.getVelocity(), velocity) + 
            driveFeedForward.calculate(velocity));
    }

    @Override
    protected void turn(double angle, int inversion) {
        turn.setInverted(inversion == -1 ? !defaultInversion : defaultInversion);
        double error = Conversion.normalizeAngle(angle - getModuleAngle());
        double output = turnPidController.calculate(error, 0);
        turn.set(output);
    }

    @Override
    protected void setTurn(double speed) {
        turn.set(speed);        
    }

    @Override
    protected void setDrive(double speed) {
        turn.set(speed);        
    }
}
