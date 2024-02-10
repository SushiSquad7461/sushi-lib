package SushiFrcLib.Swerve.SwerveModules;

import SushiFrcLib.Math.Conversion;
import SushiFrcLib.Swerve.SwerveConstants.SDSModules;
import SushiFrcLib.Swerve.SwerveConstants.SwerveModuleConstants;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Falcon Swerve Module.
 */
public class SwerveModuleTalon extends SwerveModule {
    private TalonFX angleMotor;
    private TalonFX driveMotor;

    private VelocityVoltage drivePID;
    private DutyCycleOut driveD;
    private PositionVoltage anglePID;

    public SwerveModuleTalon(SwerveModuleConstants moduleConstants) {
        super(moduleConstants);

        angleMotor = moduleConstants.getAngleFalcon();
        driveMotor = moduleConstants.getDriveFalcon();

        resetToAbsolute();

        lastAngle = getPose().angle;

        driveD = new DutyCycleOut(0);
        drivePID = new VelocityVoltage(0, 0, false, 0,0, false, false, false); 
        anglePID = new PositionVoltage(0,0,false,0,0,false,false,false); 
    }

    @Override
    public void resetToAbsolute() {
        angleMotor.setPosition(getCanCoder().getDegrees());
        lastAngle = getCanCoder();
    }

    @Override
    public void log() {
        if (swerveModuleConstants.swerveTuningMode) {
            SmartDashboard.putNumber("Current Reltaive Encoder Angle " + swerveModuleConstants.moduleNumber, MathUtil.inputModulus(angleMotor.getPosition().getValue(), 0, 360));
            SmartDashboard.putNumber("Current Reltaive Encoder Angle Non Mod " + swerveModuleConstants.moduleNumber, angleEncoder.getPosition().getValue());
            SmartDashboard.putNumber("Current Drive Velocity" + swerveModuleConstants.moduleNumber, Conversion.RPSToMPS(driveMotor.getVelocity().getValue(), SwerveModuleConstants.wheelCircumference));
            SmartDashboard.putNumber("CanCoder Angle" + swerveModuleConstants.moduleNumber, getAbsoluteAngleDegrees());
            SmartDashboard.putNumber("PID Error " + swerveModuleConstants.moduleNumber,driveMotor.getClosedLoopError().getValue());
        }

    }

    @Override
    protected void applySwerveModuleState(double velocityMPS, Rotation2d angle) {
        driveMotor.setControl(drivePID.withVelocity(Conversion.MPSToRPS(velocityMPS, SwerveModuleConstants.wheelCircumference))); 
                // driveMotor.setControl(driveD.withOutput(1.0));   
  
        angleMotor.setControl(anglePID.withPosition(angle.getDegrees()));

        if (swerveModuleConstants.swerveTuningMode) {
            SmartDashboard.putNumber("Target Drive Velocity: " + swerveModuleConstants.moduleNumber, velocityMPS);
            SmartDashboard.putNumber("Target Relative Encoder Angle " + swerveModuleConstants.moduleNumber, angle.getDegrees());
        }
    }

    @Override
    protected Rotation2d getEncoderAngle() {
        return Rotation2d.fromDegrees(angleMotor.getPosition().getValue());
    }

    @Override
    protected double getPositionMeter() {
        return Conversion.rotationsToM(driveMotor.getPosition().getValue(), SwerveModuleConstants.wheelCircumference);
    }

    @Override
    protected double getVelocityMeter() {
        return Conversion.RPSToMPS(driveMotor.getVelocity().getValue(), SwerveModuleConstants.wheelCircumference);
    }

    @Override
    public void updatePID() {
        angleTuning.updatePID(angleMotor);
        driveTuning.updatePID(driveMotor);
    }
}
