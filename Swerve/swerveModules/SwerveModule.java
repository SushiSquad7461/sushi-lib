package SushiFrcLib.Swerve.swerveModules;

import com.ctre.phoenix.sensors.WPI_CANCoder;

import SushiFrcLib.Swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;


public abstract class SwerveModule {
    public int moduleNumber;

    protected double angleOffset;
    protected SwerveModuleConstants swerveModuleConstants;
    protected WPI_CANCoder angleEncoder;

    public SwerveModule(SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleConstants.moduleNumber;
        angleOffset = moduleConstants.angleOffset;
        angleEncoder = moduleConstants.getCanCoder();
        this.swerveModuleConstants = moduleConstants;
    }

    abstract public void resetToAbsolute();

    abstract public SwerveModulePosition getPose();

    abstract public void setDesiredState(SwerveModuleState desiredState);

    public void log() {}

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public double getAngle() {
        return MathUtil.inputModulus(getCanCoder().getDegrees() - angleOffset, 0, 360);
    }
}
