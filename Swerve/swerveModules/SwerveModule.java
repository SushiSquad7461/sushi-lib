package SushiFrcLib.Swerve.swerveModules;

import SushiFrcLib.Swerve.SwerveModuleConstants;
import SushiFrcLib.Swerve.SwerveModulePosition;
import SushiFrcLib.Swerve.SwerveModuleState;

public abstract class SwerveModule {
    public int moduleNumber;
    protected double angleOffset;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;
    }

    abstract public void resetToAbsolute();

    abstract public double getAngle();

    abstract public SwerveModulePosition getPose();

    abstract public void setDesiredState(SwerveModuleState desiredState);
}
