package SushiFrcLib.Swerve.SwerveModules;
import SushiFrcLib.SmartDashboard.PIDTuning;
import SushiFrcLib.Swerve.SwerveConstants.SwerveModuleConstants;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleNeoTalon extends SwerveModule {
    private final TalonFX driveMotor;
    private final SparkMax angleMotor;
    private final RelativeEncoder angleEncoder;
    private final SparkClosedLoopController anglePID;

    private VelocityVoltage drivePID;

    public SwerveModuleNeoTalon(SwerveModuleConstants constants) {
        super(constants);
        driveMotor = constants.getDriveFalcon(); 

        angleMotor = constants.getAngleNeo();
        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getClosedLoopController();

        resetToAbsolute();

        drivePID = new VelocityVoltage(0); 
        lastAngle = getPose().angle;
    }


    @Override
    public void resetToAbsolute() {
        angleEncoder.setPosition(getCanCoder().getDegrees()); 
        lastAngle = getCanCoder();
    }

    @Override
    public void log() {
        if (swerveModuleConstants.swerveTuningMode) {
            SmartDashboard.putNumber("Angle Current " + swerveModuleConstants.moduleNumber, angleMotor.getOutputCurrent());
            SmartDashboard.putNumber("Current Reltaive Encoder Angle " + swerveModuleConstants.moduleNumber, MathUtil.inputModulus(angleEncoder.getPosition(), 0, 360));
            SmartDashboard.putNumber("Current Reltaive Encoder Angle Non Mod " + swerveModuleConstants.moduleNumber, angleEncoder.getPosition());
            SmartDashboard.putNumber("Current Drive Velocity" + swerveModuleConstants.moduleNumber, driveMotor.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("CanCoder Angle" + swerveModuleConstants.moduleNumber, getAbsoluteAngleDegrees());
        }
    }

    @Override
    protected void applySwerveModuleState(double velocityMPS, Rotation2d angleRadians) {
        driveMotor.setControl(drivePID.withVelocity(velocityMPS));
        anglePID.setReference(angleRadians.getDegrees(), SparkMax.ControlType.kPosition);

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
        return driveMotor.getPosition().getValueAsDouble();
    }

    @Override
    protected double getVelocityMeter() {
        return driveMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void updatePID(PIDTuning angle, PIDTuning drive) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updatePID'");
    }
}