package SushiFrcLib.Swerve.SwerveModules;

import SushiFrcLib.Math.Conversion;
import SushiFrcLib.Swerve.SwerveConstants.SwerveModuleConstants;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Falcon Swerve Module.
 */
public class SwerveModuleTalon extends SwerveModule {
    private TalonFX angleMotor;
    private TalonFX driveMotor;

    private VelocityVoltage drivePID;
    private PositionVoltage anglePID;

    public SwerveModuleTalon(SwerveModuleConstants moduleConstants) {
        super(moduleConstants);

        angleMotor = moduleConstants.getAngleFalcon();
        driveMotor = moduleConstants.getDriveFalcon();

        resetToAbsolute();

        lastAngle = getPose().angle;

        drivePID = new VelocityVoltage(0,0,false,0,0,false,false,false); 
        anglePID = new PositionVoltage(0,0,false,0,0,false,false,false); 
    }

    @Override
    public void resetToAbsolute() {
        angleMotor.setPosition(getCanCoder().getRotations() * swerveModuleConstants.moduleInfo.angleGearRatio);
        lastAngle = getCanCoder();
    }

    @Override
    protected void applySwerveModuleState(double velocityMPS, Rotation2d angle) {
        driveMotor.setControl(
            drivePID.withVelocity(
                Conversion.MPSToFalcon(
                    velocityMPS, 
                    SwerveModuleConstants.wheelCircumference,
                    swerveModuleConstants.moduleInfo.driveGearRatio
                ) 
            )
        );

        angleMotor.setControl(
            anglePID.withPosition(
                angle.getRotations() * swerveModuleConstants.moduleInfo.angleGearRatio
            )
        );

    }

    @Override
    protected Rotation2d getEncoderAngle() {
        return Rotation2d.fromRotations(
            angleMotor.getPosition().getValue() / swerveModuleConstants.moduleInfo.angleGearRatio
        );
    }

    @Override
    protected double getPositionMeter() {
        return Conversion.rotationsToM(
            driveMotor.getPosition().getValue(), 
            SwerveModuleConstants.wheelCircumference,
            swerveModuleConstants.moduleInfo.driveGearRatio
        );
    }

    @Override
    protected double getVelocityMeter() {
        return Conversion.RPSToMPS(
            driveMotor.getVelocity().getValue(), 
            SwerveModuleConstants.wheelCircumference,
            swerveModuleConstants.moduleInfo.driveGearRatio
        );
    }
}
