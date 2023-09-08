package SushiFrcLib.Swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

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
    
    public final static double wheelCircumference = 4;
    
    private final double driveGearRatio;
    private final double angleGearRatio;


    /**
     * Swerve Module Constants to be used when creating swerve modules.
     */
    public SwerveModuleConstants(int moduleNumber, double angleOffset, SDSModules moduleInfo) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = angleOffset;

        driveMotorId = (moduleNumber * 3) + 1;
        cancoderId = (moduleNumber * 3) + 2;
        angleMotorId = (moduleNumber * 3) + 3;

        driveGearRatio = moduleInfo.getDriveGearRatio();
        angleGearRatio = moduleInfo.getAngleGearRatio();
    }

    public abstract CANSparkMax getDriveNeo();
    public abstract WPI_TalonFX getDriveFalcon();
    public abstract CANSparkMax getAngleNeo();
    public abstract WPI_TalonFX getAngleFalcon();
}
