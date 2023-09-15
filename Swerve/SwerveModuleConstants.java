package SushiFrcLib.Swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.util.Units;

/**
 * Wrapper class for swerve module constants.
 */
public abstract class SwerveModuleConstants {
    public final int driveMotorId;
    public final int angleMotorId;
    public final int cancoderId;
    public final int moduleNumber;
    public final double angleOffset;

    public final static NeutralMode angleNeutralMode = NeutralMode.Coast;
    public final static IdleMode angleIdleMode = IdleMode.kCoast;
    public static NeutralMode driveNeutralMode = NeutralMode.Brake;
    public static IdleMode driveIdleMode = IdleMode.kBrake;
    
    public final static double wheelCircumference = Units.inchesToMeters(4) * Math.PI;
    
    public final double driveGearRatio;
    public final double angleGearRatio;

    public final double driveRotationsToMeters;
    public final double driveRMPToMetersPerSec;

    public final double angleRotationsToDegrees;
    public final double angleRMPToDegreesPerSec;

    public final double maxSpeed;

    public final boolean swerveTuningMode;


    /**
     * Swerve Module Constants to be used when creating swerve modules.
     */
    public SwerveModuleConstants(int moduleNumber, double angleOffset, SDSModules moduleInfo, double maxSpeed, boolean swerveTuningMode) {
        this.maxSpeed = maxSpeed;
        this.moduleNumber = moduleNumber;
        this.angleOffset = angleOffset;

        driveMotorId = (moduleNumber * 3) + 1;
        cancoderId = (moduleNumber * 3) + 2;
        angleMotorId = (moduleNumber * 3) + 3;

        driveGearRatio = moduleInfo.getDriveGearRatio();
        angleGearRatio = moduleInfo.getAngleGearRatio();

        driveRotationsToMeters = wheelCircumference / driveGearRatio;
        driveRMPToMetersPerSec = driveRotationsToMeters / 60.0;

        angleRotationsToDegrees = (360.0) / angleGearRatio;
        angleRMPToDegreesPerSec = angleRotationsToDegrees / 60.0;

        this.swerveTuningMode = swerveTuningMode;
    }

    public abstract CANSparkMax getDriveNeo();
    public abstract WPI_TalonFX getDriveFalcon();

    public abstract CANSparkMax getAngleNeo();
    public abstract WPI_TalonFX getAngleFalcon();

    public abstract WPI_CANCoder getCanCoder();
}
