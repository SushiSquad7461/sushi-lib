package SushiFrcLib.Swerve.SwerveModules;

import SushiFrcLib.Math.Conversion;
import SushiFrcLib.Swerve.CTREModuleState;
import SushiFrcLib.Swerve.SwerveConstants.SwerveModuleConstants;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Falcon Swerve Module.
 */
public class SwerveModuleTalon extends SwerveModule {
    private TalonFX angleMotor;
    private TalonFX driveMotor;

    private Rotation2d lastAngle;

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

    /**
     * Set the state of the swerve module.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = CTREModuleState.optimize(desiredState, lastAngle);

        driveMotor.setControl(
            drivePID.withVelocity(
                Conversion.MPSToFalcon(
                    desiredState.speedMetersPerSecond, 
                    SwerveModuleConstants.wheelCircumference,
                    swerveModuleConstants.moduleInfo.driveGearRatio
                ) 
            )
        );

        // TODO: Test anti jitter code
        if (desiredState.speedMetersPerSecond < swerveModuleConstants.moduleInfo.maxSpeed * 0.02) {
            desiredState.angle = lastAngle;
        }


        angleMotor.setControl(
            anglePID.withPosition(
                Conversion.degreesToFalcon(
                    desiredState.angle.getDegrees(), 
                    swerveModuleConstants.moduleInfo.angleGearRatio
                )
            )
        );

        lastAngle = desiredState.angle;
    }

    @Override
    public void resetToAbsolute() {
        angleMotor.setPosition(getCanCoder().getRotations());
    }

    @Override
    public SwerveModulePosition getPose() {
        double distance = Conversion.rotationsToM(
            driveMotor.getPosition().getValue(), SwerveModuleConstants.wheelCircumference,
            swerveModuleConstants.moduleInfo.driveGearRatio
        );
        Rotation2d angle = Rotation2d.fromRotations(
            angleMotor.getPosition().getValue() / swerveModuleConstants.moduleInfo.angleGearRatio
        );

        return new SwerveModulePosition(distance, angle);
    }
}
