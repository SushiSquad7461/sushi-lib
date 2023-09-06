package SushiFrcLib.Swerve.swerveModules;

import SushiFrcLib.Math.Conversion;
import SushiFrcLib.Motor.MotorHelper;
import SushiFrcLib.Swerve.SwerveModuleConstants;
import SushiFrcLib.Swerve.SwerveModulePosition;
import SushiFrcLib.Swerve.SwerveModuleState;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Falcon Swerve Module.
 */
public class SwerveModuleFalcon extends SwerveModule {
    private TalonFX angleMotor;
    private TalonFX driveMotor;
    private CANCoder angleEncoder;
    private double lastAngle;
    SwerveModuleConstants moduleConstants;

    /**
     * Instantiate a swerve module with a number from 1-4.
     */
    public SwerveModuleFalcon(int moduleNumber, SwerveModuleConstants moduleConstants) {
        super(moduleNumber, moduleConstants);

        this.moduleConstants = moduleConstants;

        angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new WPI_CANCoder(moduleConstants.cancoderId);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = MotorHelper.createFalconMotor(moduleConstants.angleMotorId, 
                moduleConstants.angleCurrentLimit,
                moduleConstants.angleInversion, moduleConstants.angleNeutralMode, 
                moduleConstants.angleP, moduleConstants.angleI, moduleConstants.angleD,
                moduleConstants.angleF,
                moduleConstants.canivoreName, SensorInitializationStrategy.BootToZero
        );
        resetToAbsolute();

        /* Drive Motor Config */
        driveMotor = MotorHelper.createFalconMotor(moduleConstants.driveMotorId, 
                moduleConstants.driveCurrentLimit,
                moduleConstants.driveInversion, moduleConstants.driveNeutralMode, 
                moduleConstants.driveP, moduleConstants.driveI, moduleConstants.driveD,
                moduleConstants.driveF,
                moduleConstants.canivoreName, SensorInitializationStrategy.BootToZero, 
                moduleConstants.openLoopRamp
        );
        driveMotor.setSelectedSensorPosition(0);

        lastAngle = getState().angle.getDegrees();
    }

    /**
     * Set the state of the swerve module.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // TODO: test optimization code
        double targetAngle = placeInAppropriate0To360Scope(
            lastAngle, desiredState.angle.getDegrees()
        );

        double targetSpeed = desiredState.velocity;

        double delta = targetAngle - lastAngle;

        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);        
        }

        driveMotor.set(ControlMode.Velocity, Conversion.MPSToFalcon(
            targetSpeed, 
            moduleConstants.wheelCircumference,
            moduleConstants.driveGearRatio
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
                moduleConstants.angleGearRatio
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
        double absolutePosition = Conversion.degreesToFalcon(getAngle(), moduleConstants.angleGearRatio);
        angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        angleEncoder.configSensorDirection(moduleConstants.cancoderInversion);
        angleEncoder.configSensorInitializationStrategy(
            SensorInitializationStrategy.BootToAbsolutePosition
        );
    }

    public SwerveModulePosition getPose() {
        double distance = Conversion.falconToM(
            driveMotor.getSelectedSensorPosition(), moduleConstants.wheelCircumference,
            moduleConstants.driveGearRatio
        );
        Rotation2d angle = Rotation2d.fromDegrees(
            Conversion.falconToDegrees(
                angleMotor.getSelectedSensorPosition(), moduleConstants.angleGearRatio
            )
        );
        return new SwerveModulePosition(distance, angle);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public double getAngle() {
        return -getCanCoder().getDegrees() + angleOffset;
    }


    /**
     * Gets current state of the swerve module.
     */
    public SwerveModulePosition getState() {
        double velocity = Conversion.falconToM(
            driveMotor.getSelectedSensorPosition(), moduleConstants.wheelCircumference,
            moduleConstants.driveGearRatio
        );

        Rotation2d angle = Rotation2d.fromDegrees(
            Conversion.falconToDegrees(
                angleMotor.getSelectedSensorPosition(), moduleConstants.angleGearRatio
            )
        );

        return new SwerveModulePosition(velocity, angle);
    }

}
