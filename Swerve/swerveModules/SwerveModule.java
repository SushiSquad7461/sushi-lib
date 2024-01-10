package SushiFrcLib.Swerve.SwerveModules;



import com.ctre.phoenix6.hardware.CANcoder;

import SushiFrcLib.Swerve.SwerveConstants.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;


public abstract class SwerveModule {
    public SwerveModuleConstants swerveModuleConstants;
    protected CANcoder angleEncoder;

    public SwerveModule(SwerveModuleConstants moduleConstants) {
        angleEncoder = moduleConstants.getCanCoder();
        this.swerveModuleConstants = moduleConstants;
    }

    abstract public void resetToAbsolute();

    abstract public SwerveModulePosition getPose();

    abstract public void setDesiredState(SwerveModuleState desiredState);

    public void log() {}

    public Rotation2d getCanCoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public double getAbsoluteAngleDegrees() {
        return MathUtil.inputModulus(getCanCoder().getDegrees(), 0, 360);
    }
}
