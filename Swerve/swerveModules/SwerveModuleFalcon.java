package SushiFrcLib.Swerve.swerveModules;

import SushiFrcLib.Math.Conversion;
import SushiFrcLib.Swerve.CTREModuleState;
import SushiFrcLib.Swerve.SwerveModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Falcon Swerve Module.
 */
public class SwerveModuleFalcon extends SwerveModule {
    private WPI_TalonFX angleMotor;
    private WPI_TalonFX driveMotor;

    private Rotation2d lastAngle;

    public SwerveModuleFalcon(SwerveModuleConstants moduleConstants) {
        super(moduleConstants);

        angleMotor = moduleConstants.getAngleFalcon();
        driveMotor = moduleConstants.getDriveFalcon();

        resetToAbsolute();

        lastAngle = getPose().angle;
    }

    /**
     * Set the state of the swerve module.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = CTREModuleState.optimize(desiredState, lastAngle);

        driveMotor.set(ControlMode.Velocity, Conversion.MPSToFalcon(
            desiredState.speedMetersPerSecond, 
            SwerveModuleConstants.wheelCircumference,
            swerveModuleConstants.driveGearRatio
        ));
       
        // TODO: Test anti jitter code
        if (desiredState.speedMetersPerSecond < swerveModuleConstants.maxSpeed * 0.02) {
            desiredState.angle = lastAngle;
        }

        angleMotor.set(
            ControlMode.Position, 
            Conversion.degreesToFalcon(
                desiredState.angle.getDegrees(), 
                swerveModuleConstants.angleGearRatio
            )
        );

        lastAngle = desiredState.angle;
    }

    @Override
    public void resetToAbsolute() {
        double absolutePosition = Conversion.degreesToFalcon(getAngle(), swerveModuleConstants.angleGearRatio);
        angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    @Override
    public SwerveModulePosition getPose() {
        double distance = Conversion.falconToM(
            driveMotor.getSelectedSensorPosition(), SwerveModuleConstants.wheelCircumference,
            swerveModuleConstants.driveGearRatio
        );
        Rotation2d angle = Rotation2d.fromDegrees(
            Conversion.falconToDegrees(
                angleMotor.getSelectedSensorPosition(), swerveModuleConstants.angleGearRatio
            )
        );
        return new SwerveModulePosition(distance, angle);
    }
}
