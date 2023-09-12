package SushiFrcLib.Swerve.swerveModules;

import SushiFrcLib.Math.Conversion;
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

    private double lastAngle;

    public SwerveModuleFalcon(SwerveModuleConstants moduleConstants) {
        super(moduleConstants);

        angleMotor = moduleConstants.getAngleFalcon();
        driveMotor = moduleConstants.getDriveFalcon();

        resetToAbsolute();

        lastAngle = getPose().angle.getDegrees();
    }

    /**
     * Set the state of the swerve module.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // TODO: test optimization code
        double targetAngle = placeInAppropriate0To360Scope(
            lastAngle, desiredState.angle.getDegrees()
        );

        double targetSpeed = desiredState.speedMetersPerSecond;

        double delta = targetAngle - lastAngle;

        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);        
        }

        driveMotor.set(ControlMode.Velocity, Conversion.MPSToFalcon(
            targetSpeed, 
            SwerveModuleConstants.wheelCircumference,
            swerveModuleConstants.driveGearRatio
        ));
       
        // TODO: Test anti jitter code
        // anit jitter code
        // if (targetSpeed < moduleConstants.MAX_SPEED * 0.02) {
        //     targetAngle = lastAngle;
        // }

        angleMotor.set(
            ControlMode.Position, 
            Conversion.degreesToFalcon(
                targetAngle, 
                swerveModuleConstants.angleGearRatio
            )
        );

        lastAngle = targetAngle;
    }

    // Stolen from CTREModuleState.java in SushiFrcLib
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
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
