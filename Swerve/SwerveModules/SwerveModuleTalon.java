package SushiFrcLib.Swerve.SwerveModules;

import SushiFrcLib.Math.Conversion;
import SushiFrcLib.SmartDashboard.PIDTuning;
import SushiFrcLib.Swerve.SwerveConstants.SwerveModuleConstants;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Falcon Swerve Module.
 */
public class SwerveModuleTalon extends SwerveModule {
    private TalonFX angleMotor;
    private TalonFX driveMotor;

    private VelocityDutyCycle drivePID;
    private PositionDutyCycle anglePID;

    public SwerveModuleTalon(SwerveModuleConstants moduleConstants) {
        super(moduleConstants);

        angleMotor = moduleConstants.getAngleFalcon();
        driveMotor = moduleConstants.getDriveFalcon();

        resetToAbsolute();

        lastAngle = getPose().angle;

        drivePID = new VelocityDutyCycle(0); 
        anglePID = new PositionDutyCycle(0); 
    }

    @Override
    public void resetToAbsolute() {
        angleMotor.setPosition(getCanCoder().getDegrees());
        lastAngle = getCanCoder();
    }

    @Override
    public void log() {
        if (swerveModuleConstants.swerveTuningMode) {
            SmartDashboard.putNumber("Current Relative Encoder Angle " + swerveModuleConstants.moduleNumber, MathUtil.inputModulus(angleMotor.getPosition().getValueAsDouble(), 0, 360));
            SmartDashboard.putNumber("Current Relative Encoder Angle Non Mod " + swerveModuleConstants.moduleNumber, angleEncoder.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("Current Drive Velocity" + swerveModuleConstants.moduleNumber, Conversion.RPSToMPS(driveMotor.getVelocity().getValue().in(Units.RotationsPerSecond), SwerveModuleConstants.wheelCircumference));
        }

    }

    @Override
    protected void applySwerveModuleState(double velocityMPS, Rotation2d angle) {
        driveMotor.setControl(drivePID.withVelocity(Conversion.MPSToRPS(velocityMPS, SwerveModuleConstants.wheelCircumference))); 
        angleMotor.setControl(anglePID.withPosition(angle.getDegrees()));

        
        if (swerveModuleConstants.swerveTuningMode) {
            SmartDashboard.putNumber("Target Drive Velocity: " + swerveModuleConstants.moduleNumber, velocityMPS);
            SmartDashboard.putNumber("Target Relative Encoder Angle " + swerveModuleConstants.moduleNumber, angle.getDegrees());
        }
    }

    @Override
    protected Rotation2d getEncoderAngle() {
        return Rotation2d.fromDegrees(angleMotor.getPosition().getValue().in(Units.Degrees));
    }

    @Override
    protected double getPositionMeter() {
        return Conversion.rotationsToM(driveMotor.getPosition().getValue().in(Units.Rotations), SwerveModuleConstants.wheelCircumference);
    }

    @Override
    protected double getVelocityMeter() {
        return Conversion.RPSToMPS(driveMotor.getVelocity().getValue().in(Units.RotationsPerSecond), SwerveModuleConstants.wheelCircumference);
    }

    @Override
    public void updatePID(PIDTuning angle, PIDTuning drive) {
        angle.updatePID(angleMotor);
        drive.updatePID(driveMotor);
    }
}
