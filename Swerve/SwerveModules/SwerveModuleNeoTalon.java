package SushiFrcLib.Swerve.SwerveModules;
import SushiFrcLib.SmartDashboard.PIDTuning;
import SushiFrcLib.Swerve.SwerveConstants.SwerveModuleConstants;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleNeoTalon extends SwerveModule {
    private final TalonFX driveMotor;
    private final CANSparkMax angleMotor;
    private final RelativeEncoder angleEncoder;
    private final SparkPIDController anglePID;

    private VelocityVoltage drivePID;

    public SwerveModuleNeoTalon(SwerveModuleConstants constants) {
        super(constants);
        driveMotor = constants.getDriveFalcon(); 

        angleMotor = constants.getAngleNeo();
        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getPIDController();

        resetToAbsolute();

        drivePID = new VelocityVoltage(0,0,false,0,0,false,false,false); 
        lastAngle = getPose().angle;
    }


    @Override
    public void resetToAbsolute() {
        angleEncoder.setPosition(getCanCoder().getRotations()); 
        lastAngle = getCanCoder();
    }

    @Override
    public void log() {
        if (swerveModuleConstants.swerveTuningMode) {
            SmartDashboard.putNumber("Angle Current " + swerveModuleConstants.moduleNumber, angleMotor.getOutputCurrent());
            SmartDashboard.putNumber("Current Reltaive Encoder Angle " + swerveModuleConstants.moduleNumber, MathUtil.inputModulus(angleEncoder.getPosition(), 0, 360));
            SmartDashboard.putNumber("Current Reltaive Encoder Angle Non Mod " + swerveModuleConstants.moduleNumber, angleEncoder.getPosition());
            SmartDashboard.putNumber("Current Drive Velocity" + swerveModuleConstants.moduleNumber, driveMotor.getVelocity().getValue());
            SmartDashboard.putNumber("CanCoder Angle" + swerveModuleConstants.moduleNumber, getAbsoluteAngleDegrees());
        }
    }

    @Override
    protected void applySwerveModuleState(double velocityMPS, Rotation2d angleRadians) {
        driveMotor.setControl(drivePID.withVelocity(velocityMPS));
        anglePID.setReference(angleRadians.getDegrees(), CANSparkMax.ControlType.kPosition);

        if (swerveModuleConstants.swerveTuningMode) {
            SmartDashboard.putNumber("Target Drive Velocity: " + swerveModuleConstants.moduleNumber, velocityMPS);
            SmartDashboard.putNumber("Target Relative Encoder Angle " + swerveModuleConstants.moduleNumber, angleRadians.getDegrees());
        }
    }

    @Override
    protected Rotation2d getEncoderAngle() {
        return Rotation2d.fromDegrees(angleEncoder.getPosition());
    }

    @Override
    protected double getPositionMeter() {
        return driveMotor.getPosition().getValue();
    }

    @Override
    protected double getVelocityMeter() {
        return driveMotor.getVelocity().getValue();
    }

    @Override
    public void updatePID(PIDTuning angle, PIDTuning drive) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updatePID'");
    }
}